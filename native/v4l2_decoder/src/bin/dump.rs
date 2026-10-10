//! Decodes an H.264 Annex B file through the V4L2 decoder and writes the
//! visible part of each frame as raw NV12, read through the exported
//! DMA-BUFs, for comparison with a software decode:
//!
//!     v4l2-dec-dump in.h264 out.nv12 [--frames N] [--device /dev/videoN]
//!     ffmpeg -i in.h264 -f rawvideo -pix_fmt nv12 ref.nv12
//!
//! H.264 decoding is exact, so the two files should be identical.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::os::fd::AsRawFd;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use v4l2_decoder::v4l2::{self, dma_buf_sync};
use v4l2_decoder::{annexb, Decoder, Event, Options};

struct Mapping {
    ptr: *const u8,
    len: usize,
}

fn main() {
    if let Err(err) = run() {
        eprintln!("error: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let input = args.next().ok_or("usage: v4l2-dec-dump in.h264 out.nv12 [--frames N] [--device PATH]")?;
    let output = args.next().ok_or("missing output path")?;
    let mut max_frames = usize::MAX;
    let mut send_stop = true;
    let mut options = Options::default();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--frames" => max_frames = args.next().ok_or("--frames N")?.parse()?,
            "--no-stop" => send_stop = false,
            "--device" => options.device = Some(PathBuf::from(args.next().ok_or("--device PATH")?)),
            other => return Err(format!("unknown argument {other}").into()),
        }
    }

    let data = std::fs::read(&input)?;
    let units = annexb::access_units(&data);
    let largest = units.iter().map(|u| u.len()).max().unwrap_or(0);
    eprintln!("{input}: {} access units, largest {largest} bytes", units.len());

    let mut decoder = Decoder::open(options)?;
    eprintln!("decoder: {}", decoder.device.display());

    let mut out = BufWriter::new(File::create(&output)?);
    let mut maps: Vec<Option<Mapping>> = Vec::new();
    let started = Instant::now();
    let mut next = 0;
    let mut frames = 0;
    let mut errors = 0;
    let mut stopped = false;
    let mut last_seen = false;
    let mut eos_event = false;
    let mut last_progress = Instant::now();
    let mut first_pts = Vec::new();

    while !last_seen {
        // Feed as much as fits.
        while next < units.len().min(max_frames) && decoder.can_queue() {
            decoder.queue_au(units[next], next as i64 * 33_333)?;
            next += 1;
            last_progress = Instant::now();
        }
        if !stopped && next == units.len().min(max_frames) {
            if send_stop {
                decoder.stop()?;
            }
            stopped = true;
            eprintln!("all input queued; stop sent after {:?}", started.elapsed());
        }

        if stopped && frames == units.len().min(max_frames) && !last_seen {
            eprintln!("every access unit has come out as a frame after {:?}", started.elapsed());
            break;
        }

        for event in decoder.poll(50)? {
            last_progress = Instant::now();
            match event {
                Event::CaptureReady => {
                    let info = decoder.capture_info().unwrap();
                    eprintln!(
                        "capture: coded {}x{}, stride {}, uv offset {}, buffer {} bytes, visible {:?}, min buffers {}, buffers {}",
                        info.coded_width, info.coded_height, info.stride, info.uv_offset,
                        info.buffer_size, info.visible, info.min_buffers, info.dmabufs.len()
                    );
                    maps = (0..info.dmabufs.len()).map(|_| None).collect();
                }
                Event::Eos => eos_event = true,
                Event::Frame { index, pts_us, bytesused, error, last } => {
                    if error {
                        errors += 1;
                    } else if bytesused > 0 {
                        write_frame(&decoder, &mut maps, index, &mut out)?;
                        frames += 1;
                        if first_pts.len() < 12 {
                            first_pts.push(pts_us / 33_333);
                        }
                    }
                    decoder.release(index)?;
                    if last {
                        last_seen = true;
                    }
                }
            }
        }

        if last_progress.elapsed() > Duration::from_secs(3) {
            eprintln!("no progress for 3 s (stopped: {stopped}): giving up on the drain");
            break;
        }
    }

    out.flush()?;
    let secs = started.elapsed().as_secs_f64();
    eprintln!(
        "{frames} frames written ({errors} errors) in {secs:.2} s, {:.0} fps; LAST buffer: {last_seen}, EOS event: {eos_event}",
        frames as f64 / secs
    );
    eprintln!("first frames' input order: {first_pts:?}");

    for map in maps.into_iter().flatten() {
        unsafe { libc::munmap(map.ptr as *mut libc::c_void, map.len) };
    }
    if !last_seen {
        std::process::exit(2);
    }
    Ok(())
}

fn write_frame(
    decoder: &Decoder,
    maps: &mut [Option<Mapping>],
    index: u32,
    out: &mut impl Write,
) -> Result<(), Box<dyn std::error::Error>> {
    let info = decoder.capture_info().unwrap();
    let fd = info.dmabufs[index as usize].as_raw_fd();

    let map = match &maps[index as usize] {
        Some(map) => map,
        None => {
            let len = info.buffer_size as usize;
            let ptr = unsafe { libc::mmap(std::ptr::null_mut(), len, libc::PROT_READ, libc::MAP_SHARED, fd, 0) };
            if ptr == libc::MAP_FAILED {
                return Err(format!("mmap of DMA-BUF {index}: {}", std::io::Error::last_os_error()).into());
            }
            maps[index as usize] = Some(Mapping { ptr: ptr as *const u8, len });
            maps[index as usize].as_ref().unwrap()
        }
    };

    let mut sync = dma_buf_sync { flags: v4l2::DMA_BUF_SYNC_START | v4l2::DMA_BUF_SYNC_READ };
    v4l2::ioctl(fd, v4l2::DMA_BUF_IOCTL_SYNC, &mut sync)?;

    let buf = unsafe { std::slice::from_raw_parts(map.ptr, map.len) };
    let (left, top) = (info.visible.left as usize, info.visible.top as usize);
    let (width, height) = (info.visible.width as usize, info.visible.height as usize);
    let stride = info.stride as usize;
    for row in 0..height {
        let start = (top + row) * stride + left;
        out.write_all(&buf[start..start + width])?;
    }
    let uv = info.uv_offset as usize;
    for row in 0..height / 2 {
        let start = uv + (top / 2 + row) * stride + left;
        out.write_all(&buf[start..start + width])?;
    }

    sync.flags = v4l2::DMA_BUF_SYNC_END | v4l2::DMA_BUF_SYNC_READ;
    v4l2::ioctl(fd, v4l2::DMA_BUF_IOCTL_SYNC, &mut sync)?;
    Ok(())
}
