//! The NIF behind `Membrane.V4L2.H264.Decoder`.
//!
//! A worker thread owns the decoder and runs its poll loop. The element
//! sends it access units and end of stream; it sends the element
//! `{:v4l2_decoder, message}`:
//!
//!   * `:need_input` - an OUTPUT buffer is free and nothing is waiting
//!   * `{:frame, %DMABufFrame{}, pts_us}` - a decoded frame, in display order
//!   * `:end_of_stream` - everything sent has been decoded
//!   * `{:error, reason}` - the worker has stopped
//!
//! Each frame carries a keepalive resource holding a duplicate of the
//! CAPTURE buffer's DMA-BUF fd and its acquire sync file. Releasing it (or
//! dropping it) hands the buffer back to the worker, which queues it to the
//! driver again; until then the driver can't write into it. A pipe wakes
//! the worker's poll() for new commands.

use std::collections::VecDeque;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use rustler::types::reference::Reference;
use rustler::{Atom, Binary, Encoder, Env, Error, LocalPid, NifResult, OwnedEnv, ResourceArc, Term};
use video_interop::{
    AbandonmentGuard, Descriptor, Layer, Modifier, Object, Plane, ReleaseDispatcher,
    is_abandonment_guard_resource, new_abandonment_guard as make_abandonment_guard,
};

use v4l2_decoder::{Decoder, Event, Options};

mod atoms {
    rustler::atoms! {
        ok,
        error,
        v4l2_decoder,
        need_input,
        frame,
        end_of_stream,
    }
}

const DRM_FORMAT_NV12: u32 = v4l2_decoder::v4l2::fourcc(b"NV12");
const DMA_BUF_SYNC_READ: u32 = 1;
const DMA_BUF_IOCTL_EXPORT_SYNC_FILE: libc::c_ulong = 0xc008_6202;
/// After the stop command, end the stream if nothing has come out for this
/// long. Venus decodes everything on stop but never flags the last buffer.
const DRAIN_IDLE: Duration = Duration::from_millis(750);

enum Command {
    Au(Vec<u8>, i64),
    Release(u32),
    EndOfStream,
    Close,
}

struct Shared {
    tx: Mutex<Sender<Command>>,
    /// A non-blocking pipe: a byte written to `wake_tx` wakes the worker,
    /// which polls `wake_rx`.
    wake_tx: OwnedFd,
    wake_rx: OwnedFd,
}

impl Shared {
    fn send(&self, command: Command) {
        if let Ok(tx) = self.tx.lock() {
            let _ = tx.send(command);
        }
        // A full pipe already has a wake-up pending.
        let byte = 1u8;
        unsafe { libc::write(self.wake_tx.as_raw_fd(), &byte as *const u8 as *const libc::c_void, 1) };
    }
}

pub struct DecoderResource {
    shared: Arc<Shared>,
    thread: Mutex<Option<JoinHandle<()>>>,
}

#[rustler::resource_impl]
impl rustler::Resource for DecoderResource {}

impl Drop for DecoderResource {
    fn drop(&mut self) {
        self.shared.send(Command::Close);
    }
}

pub struct Keepalive {
    shared: Arc<Shared>,
    index: u32,
    released: AtomicBool,
    fds: Mutex<Option<(OwnedFd, OwnedFd)>>,
}

#[rustler::resource_impl]
impl rustler::Resource for Keepalive {}

impl Keepalive {
    fn release(&self) {
        if !self.released.swap(true, Ordering::AcqRel) {
            if let Ok(mut fds) = self.fds.lock() {
                fds.take();
            }
            self.shared.send(Command::Release(self.index));
        }
    }
}

impl Drop for Keepalive {
    fn drop(&mut self) {
        self.release();
    }
}

#[derive(rustler::NifStruct)]
#[module = "Membrane.V4L2.H264.Decoder.Native.DMABufFrame"]
struct DmabufFrame {
    coded_width: u32,
    coded_height: u32,
    visible_x: u32,
    visible_y: u32,
    visible_width: u32,
    visible_height: u32,
    modifier: Modifier,
    descriptor: Descriptor,
    acquire_fence_fd: i32,
    keepalive: ResourceArc<Keepalive>,
}

fn lifecycle_error(message: impl Into<String>) -> Error {
    Error::Term(Box::new((atoms::error(), message.into())))
}

fn notify(pid: &LocalPid, build: impl for<'a> FnOnce(Env<'a>) -> Term<'a>) {
    let mut env = OwnedEnv::new();
    let _ = env.send_and_clear(pid, |env| (atoms::v4l2_decoder(), build(env)).encode(env));
}

fn dup_cloexec(fd: RawFd) -> std::io::Result<OwnedFd> {
    let duplicated = unsafe { libc::fcntl(fd, libc::F_DUPFD_CLOEXEC, 0) };
    if duplicated < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(unsafe { OwnedFd::from_raw_fd(duplicated) })
    }
}

#[repr(C)]
struct DmaBufExportSyncFile {
    flags: u32,
    fd: i32,
}

/// A sync file for the reads of `dmabuf` to wait on: whatever writes are
/// still pending on it (none, by the time V4L2 hands a buffer back, but the
/// stream format promises a sync file).
fn export_sync_file(dmabuf: RawFd) -> std::io::Result<OwnedFd> {
    let mut data = DmaBufExportSyncFile { flags: DMA_BUF_SYNC_READ, fd: -1 };
    let ret = unsafe { libc::ioctl(dmabuf, DMA_BUF_IOCTL_EXPORT_SYNC_FILE as _, &mut data) };
    if ret < 0 || data.fd < 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(unsafe { OwnedFd::from_raw_fd(data.fd) })
    }
}

fn build_frame(
    decoder: &Decoder,
    shared: &Arc<Shared>,
    index: u32,
) -> Result<DmabufFrame, String> {
    let info = decoder.capture_info().ok_or("no CAPTURE buffers")?;
    let source = info.dmabufs.get(index as usize).ok_or("bad CAPTURE index")?;
    let dmabuf = dup_cloexec(source.as_raw_fd()).map_err(|e| format!("duplicating a DMA-BUF: {e}"))?;
    let sync = export_sync_file(dmabuf.as_raw_fd()).map_err(|e| format!("exporting a sync file: {e}"))?;

    let modifier = Modifier::linear();
    let descriptor = Descriptor {
        version: 1,
        objects: vec![Object { fd: dmabuf.as_raw_fd(), size: info.buffer_size as u64, modifier }],
        layers: vec![Layer {
            fourcc: DRM_FORMAT_NV12,
            planes: vec![
                Plane { object_index: 0, offset: 0, pitch: info.stride },
                Plane { object_index: 0, offset: info.uv_offset as u64, pitch: info.stride },
            ],
        }],
    };
    descriptor.validate().map_err(|e| format!("invalid descriptor: {e}"))?;

    let acquire_fence_fd = sync.as_raw_fd();
    let keepalive = ResourceArc::new(Keepalive {
        shared: shared.clone(),
        index,
        released: AtomicBool::new(false),
        fds: Mutex::new(Some((dmabuf, sync))),
    });

    Ok(DmabufFrame {
        coded_width: info.coded_width,
        coded_height: info.coded_height,
        visible_x: info.visible.left.max(0) as u32,
        visible_y: info.visible.top.max(0) as u32,
        visible_width: info.visible.width,
        visible_height: info.visible.height,
        modifier,
        descriptor,
        acquire_fence_fd,
        keepalive,
    })
}

fn drain_wake(fd: RawFd) {
    let mut buf = [0u8; 64];
    while unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) } > 0 {}
}

fn wake_pipe() -> std::io::Result<(OwnedFd, OwnedFd)> {
    let mut fds = [0 as libc::c_int; 2];
    if unsafe { libc::pipe(fds.as_mut_ptr()) } < 0 {
        return Err(std::io::Error::last_os_error());
    }
    let (rx, tx) = unsafe { (OwnedFd::from_raw_fd(fds[0]), OwnedFd::from_raw_fd(fds[1])) };
    for fd in [rx.as_raw_fd(), tx.as_raw_fd()] {
        unsafe {
            libc::fcntl(fd, libc::F_SETFL, libc::O_NONBLOCK);
            libc::fcntl(fd, libc::F_SETFD, libc::FD_CLOEXEC);
        }
    }
    Ok((rx, tx))
}

fn run(mut decoder: Decoder, shared: Arc<Shared>, rx: Receiver<Command>, pid: LocalPid) {
    if let Err(reason) = run_loop(&mut decoder, &shared, &rx, &pid) {
        notify(&pid, |env| (atoms::error(), reason).encode(env));
    }
    // Dropping the decoder stops both queues. Frames still leased keep
    // their DMA-BUFs alive; releasing them later is harmless.
}

fn run_loop(
    decoder: &mut Decoder,
    shared: &Arc<Shared>,
    rx: &Receiver<Command>,
    pid: &LocalPid,
) -> Result<(), String> {
    let wake = shared.wake_rx.as_raw_fd();
    let mut pending: VecDeque<(Vec<u8>, i64)> = VecDeque::new();
    let mut asked = false;
    let mut eos_requested = false;
    let mut stop_sent_at: Option<Instant> = None;
    let mut finished = false;
    let mut queued_in: u64 = 0;
    let mut frames_out: u64 = 0;
    let mut last_output = Instant::now();

    loop {
        loop {
            match rx.try_recv() {
                Ok(Command::Au(data, pts)) => {
                    pending.push_back((data, pts));
                    asked = false;
                }
                Ok(Command::Release(index)) => {
                    // Fails only if the queues are being torn down.
                    let _ = decoder.release(index);
                }
                Ok(Command::EndOfStream) => eos_requested = true,
                Ok(Command::Close) | Err(TryRecvError::Disconnected) => return Ok(()),
                Err(TryRecvError::Empty) => break,
            }
        }

        while !pending.is_empty() && decoder.can_queue() {
            let (data, pts) = pending.pop_front().unwrap();
            decoder.queue_au(&data, pts).map_err(|e| e.to_string())?;
            queued_in += 1;
        }

        if !asked && !eos_requested && pending.is_empty() && decoder.can_queue() {
            notify(pid, |env| atoms::need_input().encode(env));
            asked = true;
        }

        if eos_requested && pending.is_empty() && stop_sent_at.is_none() && !finished {
            if queued_in == 0 {
                notify(pid, |env| atoms::end_of_stream().encode(env));
                finished = true;
            } else if decoder.capture_info().is_some() {
                // Before the CAPTURE queue runs, Venus ignores the stop.
                decoder.stop().map_err(|e| e.to_string())?;
                stop_sent_at = Some(Instant::now());
            }
        }

        let (events, woken) = decoder.poll_with(wake, 50).map_err(|e| e.to_string())?;
        if woken {
            drain_wake(wake);
        }

        let mut last_flagged = false;
        for event in events {
            if let Event::Frame { index, pts_us, bytesused, error, last } = event {
                last_flagged |= last;
                if error || bytesused == 0 {
                    decoder.release(index).map_err(|e| e.to_string())?;
                    if !(last && bytesused == 0) {
                        frames_out += 1;
                    }
                    continue;
                }
                let frame = build_frame(decoder, shared, index)?;
                frames_out += 1;
                last_output = Instant::now();
                notify(pid, |env| (atoms::frame(), frame, pts_us).encode(env));
            }
        }

        if let Some(stopped) = stop_sent_at {
            let idle = last_output.max(stopped).elapsed() > DRAIN_IDLE;
            if !finished && (last_flagged || frames_out >= queued_in || idle) {
                notify(pid, |env| atoms::end_of_stream().encode(env));
                finished = true;
            }
        }
    }
}

// ---------- NIFs ----------

#[rustler::nif(schedule = "DirtyIo")]
fn create(
    pid: LocalPid,
    device: String,
    extra_capture: u32,
    output_buffers: u32,
) -> NifResult<(Atom, ResourceArc<DecoderResource>)> {
    let options = Options {
        device: if device.is_empty() { None } else { Some(PathBuf::from(device)) },
        extra_capture,
        output_buffers,
        ..Options::default()
    };
    let decoder = Decoder::open(options).map_err(|e| lifecycle_error(e.to_string()))?;

    let (wake_rx, wake_tx) =
        wake_pipe().map_err(|e| lifecycle_error(format!("creating the wake-up pipe: {e}")))?;
    let (tx, rx) = mpsc::channel();
    let shared = Arc::new(Shared { tx: Mutex::new(tx), wake_tx, wake_rx });

    let worker_shared = shared.clone();
    let thread = std::thread::Builder::new()
        .name("v4l2-h264-decoder".into())
        .spawn(move || run(decoder, worker_shared, rx, pid))
        .map_err(|e| lifecycle_error(format!("starting the worker: {e}")))?;

    Ok((
        atoms::ok(),
        ResourceArc::new(DecoderResource { shared, thread: Mutex::new(Some(thread)) }),
    ))
}

#[rustler::nif]
fn decode(decoder: ResourceArc<DecoderResource>, data: Binary, pts_us: i64) -> Atom {
    decoder.shared.send(Command::Au(data.as_slice().to_vec(), pts_us));
    atoms::ok()
}

#[rustler::nif]
fn end_of_stream(decoder: ResourceArc<DecoderResource>) -> Atom {
    decoder.shared.send(Command::EndOfStream);
    atoms::ok()
}

#[rustler::nif(schedule = "DirtyIo")]
fn close(decoder: ResourceArc<DecoderResource>) -> Atom {
    decoder.shared.send(Command::Close);
    if let Some(thread) = decoder.thread.lock().ok().and_then(|mut t| t.take()) {
        let _ = thread.join();
    }
    atoms::ok()
}

#[rustler::nif]
fn release_frame(keepalive: ResourceArc<Keepalive>) -> Atom {
    keepalive.release();
    atoms::ok()
}

// The release dispatcher and abandonment guards, as in
// membrane_video_transcode: LeaseOwner releases through these when a lease
// holder goes away without releasing.

static RELEASE_DISPATCHER_QUARANTINED: AtomicBool = AtomicBool::new(false);
static RELEASE_DISPATCHER_ADMISSION: Mutex<()> = Mutex::new(());

#[rustler::nif(schedule = "DirtyIo")]
fn start_release_dispatcher() -> NifResult<(Atom, ResourceArc<ReleaseDispatcher>)> {
    let _admission = RELEASE_DISPATCHER_ADMISSION
        .lock()
        .map_err(|_| lifecycle_error("release dispatcher admission lock poisoned"))?;
    if RELEASE_DISPATCHER_QUARANTINED.load(Ordering::Acquire) {
        return Err(lifecycle_error("release dispatcher admission is disabled until cold VM restart"));
    }
    ReleaseDispatcher::start("v4l2-h264-decoder-release")
        .map(|dispatcher| (atoms::ok(), dispatcher))
        .map_err(|error| lifecycle_error(format!("could not start release dispatcher: {error}")))
}

#[rustler::nif(schedule = "DirtyIo")]
fn quarantine_release_dispatchers() -> NifResult<bool> {
    let _admission = RELEASE_DISPATCHER_ADMISSION
        .lock()
        .map_err(|_| lifecycle_error("release dispatcher admission lock poisoned"))?;
    Ok(!RELEASE_DISPATCHER_QUARANTINED.swap(true, Ordering::SeqCst))
}

#[rustler::nif]
fn release_dispatcher_quarantined() -> bool {
    RELEASE_DISPATCHER_QUARANTINED.load(Ordering::Acquire)
}

#[rustler::nif(schedule = "DirtyIo")]
fn close_release_dispatcher(
    dispatcher: ResourceArc<ReleaseDispatcher>,
    timeout_ms: u64,
) -> NifResult<(Atom, bool)> {
    dispatcher
        .close_and_join(Duration::from_millis(timeout_ms))
        .map(|()| (atoms::ok(), true))
        .map_err(|error| lifecycle_error(format!("could not close release dispatcher: {error}")))
}

#[rustler::nif]
fn new_abandonment_guard_resource<'a>(
    dispatcher: ResourceArc<ReleaseDispatcher>,
    owner: LocalPid,
    token: Term<'a>,
    holder: Reference<'a>,
) -> NifResult<(Atom, ResourceArc<AbandonmentGuard>)> {
    make_abandonment_guard(dispatcher, owner, token, holder)
        .map(|guard| (atoms::ok(), guard))
        .map_err(|error| lifecycle_error(format!("could not create abandonment guard: {error}")))
}

#[rustler::nif]
fn abandonment_guard_resource(term: Term<'_>) -> bool {
    is_abandonment_guard_resource(term)
}

rustler::init!("Elixir.Membrane.V4L2.H264.Decoder.Native");
