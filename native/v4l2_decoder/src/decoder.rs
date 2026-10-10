//! A V4L2 stateful (memory-to-memory) H.264 decoder, following the kernel's
//! "Memory-to-Memory Stateful Video Decoder Interface".
//!
//! Compressed access units go into mmapped OUTPUT buffers. Decoded frames
//! come back in CAPTURE buffers, each exported once as a DMA-BUF when the
//! first source-change event sets the CAPTURE queue up. A dequeued CAPTURE
//! buffer belongs to the caller until `release/1` queues it back.
//!
//! Single-threaded: one owner calls `queue_au`, `poll` and `release`. The
//! NIF runs it on a worker thread.

use std::fs::{self, OpenOptions};
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};

use crate::v4l2::*;

#[derive(Debug)]
pub enum Error {
    Io(&'static str, io::Error),
    NoDevice,
    Unsupported(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Io(what, err) => write!(f, "{what}: {err}"),
            Error::NoDevice => write!(f, "no V4L2 H.264 decoder found"),
            Error::Unsupported(why) => write!(f, "unsupported: {why}"),
        }
    }
}

impl std::error::Error for Error {}

pub type Result<T> = std::result::Result<T, Error>;

fn io<T>(what: &'static str, result: io::Result<T>) -> Result<T> {
    result.map_err(|err| Error::Io(what, err))
}

/// Decoded-frame geometry and the exported CAPTURE buffers.
#[derive(Debug)]
pub struct CaptureInfo {
    /// The decoded buffer's size (macroblock-aligned, e.g. 1920x1088).
    pub coded_width: u32,
    pub coded_height: u32,
    /// Bytes per row of luma (and of interleaved chroma).
    pub stride: u32,
    /// Where the interleaved UV plane starts in the buffer.
    pub uv_offset: u32,
    /// Each buffer's size.
    pub buffer_size: u32,
    /// The part of the picture to show (e.g. 1920x1080).
    pub visible: v4l2_rect,
    pub min_buffers: u32,
    /// One DMA-BUF per CAPTURE buffer, by index.
    pub dmabufs: Vec<OwnedFd>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// The CAPTURE queue is set up; `capture_info` is available.
    CaptureReady,
    /// A decoded frame in CAPTURE buffer `index`. Release it when done.
    /// A buffer flagged `last` may be empty (`bytesused` 0); it ends a
    /// drain. Either way it must be released.
    Frame { index: u32, pts_us: i64, bytesused: u32, error: bool, last: bool },
    /// The driver's end-of-stream event.
    Eos,
}

struct OutputBuffer {
    ptr: *mut u8,
    len: usize,
    queued: bool,
}

pub struct Decoder {
    fd: OwnedFd,
    pub device: PathBuf,
    output: Vec<OutputBuffer>,
    capture: Option<CaptureInfo>,
    capture_queued: Vec<bool>,
    extra_capture: u32,
}

// The raw pointers are mmapped OUTPUT buffers that only this struct uses.
unsafe impl Send for Decoder {}

pub struct Options {
    /// A device node, or None to find the decoder by capability.
    pub device: Option<PathBuf>,
    /// The coded size, if known; the driver learns the real one from the
    /// stream either way.
    pub width: u32,
    pub height: u32,
    /// OUTPUT buffers, and the size of each (one access unit must fit).
    pub output_buffers: u32,
    pub output_buffer_size: u32,
    /// CAPTURE buffers beyond the driver's minimum: how many decoded frames
    /// the caller may hold at once.
    pub extra_capture: u32,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            device: None,
            width: 1920,
            height: 1080,
            output_buffers: 8,
            output_buffer_size: 2 * 1024 * 1024,
            extra_capture: 4,
        }
    }
}

fn open_node(path: &Path) -> io::Result<OwnedFd> {
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_NONBLOCK | libc::O_CLOEXEC)
        .open(path)?;
    Ok(file.into())
}

fn formats(fd: RawFd, type_: u32) -> Vec<u32> {
    let mut found = Vec::new();
    for index in 0.. {
        let mut desc: v4l2_fmtdesc = zeroed();
        desc.index = index;
        desc.type_ = type_;
        if ioctl(fd, VIDIOC_ENUM_FMT, &mut desc).is_err() {
            break;
        }
        found.push(desc.pixelformat);
    }
    found
}

fn is_h264_decoder(fd: RawFd) -> bool {
    let mut cap: v4l2_capability = zeroed();
    if ioctl(fd, VIDIOC_QUERYCAP, &mut cap).is_err() {
        return false;
    }
    let caps = if cap.capabilities & CAP_DEVICE_CAPS != 0 { cap.device_caps } else { cap.capabilities };
    // An encoder takes raw frames on OUTPUT; a decoder takes H.264 there.
    caps & CAP_VIDEO_M2M_MPLANE != 0
        && caps & CAP_STREAMING != 0
        && formats(fd, BUF_TYPE_VIDEO_OUTPUT_MPLANE).contains(&PIX_FMT_H264)
        && formats(fd, BUF_TYPE_VIDEO_CAPTURE_MPLANE).contains(&PIX_FMT_NV12)
}

/// The first `/dev/video*` that is a multiplanar M2M H.264 decoder.
pub fn find_device() -> Result<(PathBuf, OwnedFd)> {
    let mut nodes: Vec<PathBuf> = io("listing /dev", fs::read_dir("/dev"))?
        .filter_map(|entry| entry.ok().map(|e| e.path()))
        .filter(|path| {
            path.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.starts_with("video"))
        })
        .collect();
    nodes.sort();

    for path in nodes {
        if let Ok(fd) = open_node(&path) {
            if is_h264_decoder(fd.as_raw_fd()) {
                return Ok((path, fd));
            }
        }
    }
    Err(Error::NoDevice)
}

impl Decoder {
    pub fn open(options: Options) -> Result<Self> {
        let (device, fd) = match options.device.clone() {
            Some(path) => {
                let fd = io("opening the device", open_node(&path))?;
                if !is_h264_decoder(fd.as_raw_fd()) {
                    return Err(Error::Unsupported(format!("{} is not an H.264 decoder", path.display())));
                }
                (path, fd)
            }
            None => find_device()?,
        };

        let mut decoder = Decoder {
            fd,
            device,
            output: Vec::new(),
            capture: None,
            capture_queued: Vec::new(),
            extra_capture: options.extra_capture,
        };

        decoder.set_up_output(&options)?;

        for type_ in [EVENT_SOURCE_CHANGE, EVENT_EOS] {
            let mut sub: v4l2_event_subscription = zeroed();
            sub.type_ = type_;
            io("subscribing to events", ioctl(decoder.raw(), VIDIOC_SUBSCRIBE_EVENT, &mut sub))?;
        }

        decoder.stream(BUF_TYPE_VIDEO_OUTPUT_MPLANE, true)?;
        Ok(decoder)
    }

    fn raw(&self) -> RawFd {
        self.fd.as_raw_fd()
    }

    fn set_up_output(&mut self, options: &Options) -> Result<()> {
        let mut fmt = v4l2_format::new(BUF_TYPE_VIDEO_OUTPUT_MPLANE);
        let mut pix: v4l2_pix_format_mplane = zeroed();
        pix.width = options.width;
        pix.height = options.height;
        pix.pixelformat = PIX_FMT_H264;
        pix.field = FIELD_NONE;
        pix.num_planes = 1;
        pix.plane_fmt[0].sizeimage = options.output_buffer_size;
        fmt.set_pix_mp(pix);
        io("setting the OUTPUT format", ioctl(self.raw(), VIDIOC_S_FMT, &mut fmt))?;

        let count = self.request_buffers(BUF_TYPE_VIDEO_OUTPUT_MPLANE, options.output_buffers)?;

        for index in 0..count {
            let (length, offset) = self.query_buffer(BUF_TYPE_VIDEO_OUTPUT_MPLANE, index)?;
            let ptr = unsafe {
                libc::mmap(
                    std::ptr::null_mut(),
                    length as usize,
                    libc::PROT_READ | libc::PROT_WRITE,
                    libc::MAP_SHARED,
                    self.raw(),
                    offset as libc::off_t,
                )
            };
            if ptr == libc::MAP_FAILED {
                return Err(Error::Io("mapping an OUTPUT buffer", io::Error::last_os_error()));
            }
            self.output.push(OutputBuffer { ptr: ptr as *mut u8, len: length as usize, queued: false });
        }
        Ok(())
    }

    fn request_buffers(&self, type_: u32, count: u32) -> Result<u32> {
        let mut req: v4l2_requestbuffers = zeroed();
        req.count = count;
        req.type_ = type_;
        req.memory = MEMORY_MMAP;
        io("requesting buffers", ioctl(self.raw(), VIDIOC_REQBUFS, &mut req))?;
        Ok(req.count)
    }

    /// A buffer's first plane: (length, mmap offset).
    fn query_buffer(&self, type_: u32, index: u32) -> Result<(u32, u32)> {
        let mut planes: [v4l2_plane; 1] = zeroed();
        let mut buf: v4l2_buffer = zeroed();
        buf.index = index;
        buf.type_ = type_;
        buf.memory = MEMORY_MMAP;
        buf.m = planes.as_mut_ptr() as u64;
        buf.length = 1;
        io("querying a buffer", ioctl(self.raw(), VIDIOC_QUERYBUF, &mut buf))?;
        Ok((planes[0].length, planes[0].m as u32))
    }

    fn stream(&self, type_: u32, on: bool) -> Result<()> {
        let mut t = type_ as libc::c_int;
        let request = if on { VIDIOC_STREAMON } else { VIDIOC_STREAMOFF };
        io("starting or stopping a queue", ioctl(self.raw(), request, &mut t))
    }

    /// The CAPTURE geometry and buffers, once `Event::CaptureReady` has
    /// been seen.
    pub fn capture_info(&self) -> Option<&CaptureInfo> {
        self.capture.as_ref()
    }

    /// Whether an OUTPUT buffer is free for `queue_au`.
    pub fn can_queue(&self) -> bool {
        self.output.iter().any(|b| !b.queued)
    }

    /// Copies one access unit into a free OUTPUT buffer and queues it.
    /// Returns false, queuing nothing, when every OUTPUT buffer is queued.
    pub fn queue_au(&mut self, data: &[u8], pts_us: i64) -> Result<bool> {
        let Some(index) = self.output.iter().position(|b| !b.queued) else {
            return Ok(false);
        };

        let fd = self.raw();
        let buffer = &mut self.output[index];
        if data.len() > buffer.len {
            return Err(Error::Unsupported(format!(
                "an access unit of {} bytes is larger than the {}-byte OUTPUT buffers",
                data.len(),
                buffer.len
            )));
        }
        unsafe { std::ptr::copy_nonoverlapping(data.as_ptr(), buffer.ptr, data.len()) };

        let mut planes: [v4l2_plane; 1] = zeroed();
        planes[0].bytesused = data.len() as u32;
        planes[0].length = buffer.len as u32;

        let mut buf: v4l2_buffer = zeroed();
        buf.index = index as u32;
        buf.type_ = BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        buf.memory = MEMORY_MMAP;
        buf.m = planes.as_mut_ptr() as u64;
        buf.length = 1;
        buf.timestamp = libc::timeval {
            tv_sec: pts_us.div_euclid(1_000_000) as libc::time_t,
            tv_usec: pts_us.rem_euclid(1_000_000) as libc::suseconds_t,
        };
        io("queuing an OUTPUT buffer", ioctl(fd, VIDIOC_QBUF, &mut buf))?;
        buffer.queued = true;
        Ok(true)
    }

    /// Hands CAPTURE buffer `index` back to the driver.
    pub fn release(&mut self, index: u32) -> Result<()> {
        let queued = self
            .capture_queued
            .get_mut(index as usize)
            .ok_or_else(|| Error::Unsupported(format!("no CAPTURE buffer {index}")))?;
        if *queued {
            return Err(Error::Unsupported(format!("CAPTURE buffer {index} is already queued")));
        }

        let mut planes: [v4l2_plane; 1] = zeroed();
        let mut buf: v4l2_buffer = zeroed();
        buf.index = index;
        buf.type_ = BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = MEMORY_MMAP;
        buf.m = planes.as_mut_ptr() as u64;
        buf.length = 1;
        io("queuing a CAPTURE buffer", ioctl(self.raw(), VIDIOC_QBUF, &mut buf))?;
        self.capture_queued[index as usize] = true;
        Ok(())
    }

    /// How many CAPTURE buffers the caller holds.
    pub fn held(&self) -> usize {
        self.capture_queued.iter().filter(|q| !**q).count()
    }

    /// Asks the decoder to finish: once everything queued is decoded, a
    /// CAPTURE buffer flagged `last` arrives.
    pub fn stop(&self) -> Result<()> {
        let mut cmd: v4l2_decoder_cmd = zeroed();
        cmd.cmd = DEC_CMD_STOP;
        io("sending the stop command", ioctl(self.raw(), VIDIOC_DECODER_CMD, &mut cmd))
    }

    /// Waits up to `timeout_ms` for something to happen, then collects
    /// everything that did: events, finished OUTPUT buffers (now free) and
    /// decoded frames.
    pub fn poll(&mut self, timeout_ms: i32) -> Result<Vec<Event>> {
        self.poll_with(-1, timeout_ms).map(|(events, _)| events)
    }

    /// `poll`, also waking when `extra` (an eventfd, say) is readable.
    /// Returns the events and whether `extra` was readable.
    pub fn poll_with(&mut self, extra: RawFd, timeout_ms: i32) -> Result<(Vec<Event>, bool)> {
        let mut pfds = [
            libc::pollfd {
                fd: self.raw(),
                events: libc::POLLIN | libc::POLLOUT | libc::POLLPRI,
                revents: 0,
            },
            // poll() ignores negative fds.
            libc::pollfd { fd: extra, events: libc::POLLIN, revents: 0 },
        ];
        let ret = unsafe { libc::poll(pfds.as_mut_ptr(), 2, timeout_ms) };
        let pfd = pfds[0];
        let extra_ready = extra >= 0 && pfds[1].revents & libc::POLLIN != 0;
        if ret < 0 {
            let err = io::Error::last_os_error();
            if err.kind() != io::ErrorKind::Interrupted {
                return Err(Error::Io("polling the device", err));
            }
        }

        let mut events = Vec::new();
        self.dequeue_events(&mut events)?;
        self.dequeue_output()?;
        self.dequeue_capture(&mut events)?;

        // The M2M core reports POLLERR while neither queue has work (before
        // the CAPTURE queue is set up, say); don't spin on it.
        if events.is_empty() && !extra_ready && pfd.revents & libc::POLLERR != 0 && timeout_ms > 0 {
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        Ok((events, extra_ready))
    }

    fn dequeue_events(&mut self, events: &mut Vec<Event>) -> Result<()> {
        loop {
            let mut ev: v4l2_event = zeroed();
            match ioctl(self.raw(), VIDIOC_DQEVENT, &mut ev) {
                Ok(()) => match ev.type_ {
                    EVENT_SOURCE_CHANGE => {
                        let changes = ev.u[0] as u32;
                        if changes & EVENT_SRC_CH_RESOLUTION != 0 {
                            if self.capture.is_some() {
                                return Err(Error::Unsupported(
                                    "a resolution change mid-stream".to_string(),
                                ));
                            }
                            self.set_up_capture()?;
                            events.push(Event::CaptureReady);
                        }
                    }
                    EVENT_EOS => events.push(Event::Eos),
                    _ => {}
                },
                // No events pending.
                Err(err) if err.raw_os_error() == Some(libc::ENOENT) => return Ok(()),
                Err(err) if err.kind() == io::ErrorKind::WouldBlock => return Ok(()),
                Err(err) => return Err(Error::Io("dequeuing an event", err)),
            }
        }
    }

    fn dequeue_output(&mut self) -> Result<()> {
        loop {
            let mut planes: [v4l2_plane; 1] = zeroed();
            let mut buf: v4l2_buffer = zeroed();
            buf.type_ = BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            buf.memory = MEMORY_MMAP;
            buf.m = planes.as_mut_ptr() as u64;
            buf.length = 1;
            match ioctl(self.raw(), VIDIOC_DQBUF, &mut buf) {
                Ok(()) => self.output[buf.index as usize].queued = false,
                Err(err) if is_drained(&err) => return Ok(()),
                Err(err) => return Err(Error::Io("dequeuing an OUTPUT buffer", err)),
            }
        }
    }

    fn dequeue_capture(&mut self, events: &mut Vec<Event>) -> Result<()> {
        if self.capture.is_none() {
            return Ok(());
        }
        loop {
            let mut planes: [v4l2_plane; 1] = zeroed();
            let mut buf: v4l2_buffer = zeroed();
            buf.type_ = BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = MEMORY_MMAP;
            buf.m = planes.as_mut_ptr() as u64;
            buf.length = 1;
            match ioctl(self.raw(), VIDIOC_DQBUF, &mut buf) {
                Ok(()) => {
                    self.capture_queued[buf.index as usize] = false;
                    let pts_us = buf.timestamp.tv_sec as i64 * 1_000_000 + buf.timestamp.tv_usec as i64;
                    events.push(Event::Frame {
                        index: buf.index,
                        pts_us,
                        bytesused: planes[0].bytesused,
                        error: buf.flags & BUF_FLAG_ERROR != 0,
                        last: buf.flags & BUF_FLAG_LAST != 0,
                    });
                }
                // EPIPE: the last buffer has already been dequeued.
                Err(err) if is_drained(&err) || err.raw_os_error() == Some(libc::EPIPE) => {
                    return Ok(())
                }
                Err(err) => return Err(Error::Io("dequeuing a CAPTURE buffer", err)),
            }
        }
    }

    /// The first source change: read the decoded format, then allocate,
    /// export and queue the CAPTURE buffers.
    fn set_up_capture(&mut self) -> Result<()> {
        let mut fmt = v4l2_format::new(BUF_TYPE_VIDEO_CAPTURE_MPLANE);
        io("reading the CAPTURE format", ioctl(self.raw(), VIDIOC_G_FMT, &mut fmt))?;
        let pix = fmt.pix_mp();
        let (pixelformat, width, height, num_planes) = (pix.pixelformat, pix.width, pix.height, pix.num_planes);
        let plane = pix.plane_fmt[0];
        let (stride, sizeimage) = (plane.bytesperline, plane.sizeimage);

        if pixelformat != PIX_FMT_NV12 {
            // Ask for NV12 (Venus may default to its compressed format).
            let mut want = fmt;
            let mut p = pix;
            p.pixelformat = PIX_FMT_NV12;
            want.set_pix_mp(p);
            io("setting the CAPTURE format", ioctl(self.raw(), VIDIOC_S_FMT, &mut want))?;
            if want.pix_mp().pixelformat != PIX_FMT_NV12 {
                return Err(Error::Unsupported(format!(
                    "CAPTURE format {}",
                    fourcc_str(want.pix_mp().pixelformat)
                )));
            }
            return self.set_up_capture();
        }
        if num_planes != 1 {
            return Err(Error::Unsupported(format!("NV12 in {num_planes} planes")));
        }

        let mut sel: v4l2_selection = zeroed();
        sel.type_ = BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        sel.target = SEL_TGT_COMPOSE;
        let visible = match ioctl(self.raw(), VIDIOC_G_SELECTION, &mut sel) {
            Ok(()) => sel.r,
            Err(_) => v4l2_rect { left: 0, top: 0, width, height },
        };

        let mut ctrl = v4l2_control { id: CID_MIN_BUFFERS_FOR_CAPTURE, value: 0 };
        let min_buffers = match ioctl(self.raw(), VIDIOC_G_CTRL, &mut ctrl) {
            Ok(()) => ctrl.value.max(1) as u32,
            Err(_) => 4,
        };

        let count = self.request_buffers(BUF_TYPE_VIDEO_CAPTURE_MPLANE, min_buffers + self.extra_capture)?;

        let mut dmabufs = Vec::with_capacity(count as usize);
        let mut buffer_size = sizeimage;
        for index in 0..count {
            let (length, _offset) = self.query_buffer(BUF_TYPE_VIDEO_CAPTURE_MPLANE, index)?;
            buffer_size = length;
            let mut exp: v4l2_exportbuffer = zeroed();
            exp.type_ = BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            exp.index = index;
            exp.plane = 0;
            exp.flags = (libc::O_CLOEXEC | libc::O_RDONLY) as u32;
            io("exporting a CAPTURE buffer", ioctl(self.raw(), VIDIOC_EXPBUF, &mut exp))?;
            dmabufs.push(unsafe { OwnedFd::from_raw_fd(exp.fd) });
        }

        self.capture = Some(CaptureInfo {
            coded_width: width,
            coded_height: height,
            stride,
            // Single-buffer NV12: chroma follows luma at the buffer's
            // (aligned) height.
            uv_offset: stride * height,
            buffer_size,
            visible,
            min_buffers,
            dmabufs,
        });

        self.capture_queued = vec![false; count as usize];
        for index in 0..count {
            self.release(index)?;
        }
        self.stream(BUF_TYPE_VIDEO_CAPTURE_MPLANE, true)
    }
}

fn is_drained(err: &io::Error) -> bool {
    err.kind() == io::ErrorKind::WouldBlock || err.raw_os_error() == Some(libc::EINVAL)
}

impl Drop for Decoder {
    fn drop(&mut self) {
        // Stopping the queues returns every buffer; the exported DMA-BUFs
        // stay valid for anyone still holding them until they're closed.
        let _ = self.stream(BUF_TYPE_VIDEO_CAPTURE_MPLANE, false);
        let _ = self.stream(BUF_TYPE_VIDEO_OUTPUT_MPLANE, false);
        for buffer in &self.output {
            unsafe { libc::munmap(buffer.ptr as *mut libc::c_void, buffer.len) };
        }
    }
}
