//! The parts of the V4L2 and DMA-BUF uAPI the decoder uses.
//!
//! Hand-written rather than generated, so the crate cross-compiles without
//! libclang. Every size and the offsets that matter were checked against
//! `linux/videodev2.h` from the nerves_system_fp3 sysroot (aarch64) with
//! static asserts, and the tests below check them again on the target.

#![allow(non_camel_case_types, dead_code)]

use std::io;
use std::os::fd::RawFd;

pub const BUF_TYPE_VIDEO_CAPTURE_MPLANE: u32 = 9;
pub const BUF_TYPE_VIDEO_OUTPUT_MPLANE: u32 = 10;
pub const MEMORY_MMAP: u32 = 1;
pub const FIELD_NONE: u32 = 1;

pub const CAP_VIDEO_M2M_MPLANE: u32 = 0x0000_4000;
pub const CAP_STREAMING: u32 = 0x0400_0000;
pub const CAP_DEVICE_CAPS: u32 = 0x8000_0000;

pub const BUF_FLAG_MAPPED: u32 = 0x0000_0001;
pub const BUF_FLAG_QUEUED: u32 = 0x0000_0002;
pub const BUF_FLAG_DONE: u32 = 0x0000_0004;
pub const BUF_FLAG_KEYFRAME: u32 = 0x0000_0008;
pub const BUF_FLAG_ERROR: u32 = 0x0000_0040;
pub const BUF_FLAG_TIMESTAMP_COPY: u32 = 0x0000_4000;
pub const BUF_FLAG_LAST: u32 = 0x0010_0000;

pub const EVENT_EOS: u32 = 2;
pub const EVENT_SOURCE_CHANGE: u32 = 5;
pub const EVENT_SRC_CH_RESOLUTION: u32 = 1;

pub const DEC_CMD_START: u32 = 0;
pub const DEC_CMD_STOP: u32 = 1;

pub const SEL_TGT_COMPOSE: u32 = 0x0100;

pub const CID_MIN_BUFFERS_FOR_CAPTURE: u32 = 0x0098_0900 + 39;

pub const fn fourcc(code: &[u8; 4]) -> u32 {
    (code[0] as u32) | (code[1] as u32) << 8 | (code[2] as u32) << 16 | (code[3] as u32) << 24
}

pub const PIX_FMT_H264: u32 = fourcc(b"H264");
pub const PIX_FMT_NV12: u32 = fourcc(b"NV12");

pub fn fourcc_str(code: u32) -> String {
    code.to_le_bytes().iter().map(|&b| b as char).collect()
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_capability {
    pub driver: [u8; 16],
    pub card: [u8; 32],
    pub bus_info: [u8; 32],
    pub version: u32,
    pub capabilities: u32,
    pub device_caps: u32,
    pub reserved: [u32; 3],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_fmtdesc {
    pub index: u32,
    pub type_: u32,
    pub flags: u32,
    pub description: [u8; 32],
    pub pixelformat: u32,
    pub mbus_code: u32,
    pub reserved: [u32; 3],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct v4l2_plane_pix_format {
    pub sizeimage: u32,
    pub bytesperline: u32,
    pub reserved: [u16; 6],
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct v4l2_pix_format_mplane {
    pub width: u32,
    pub height: u32,
    pub pixelformat: u32,
    pub field: u32,
    pub colorspace: u32,
    pub plane_fmt: [v4l2_plane_pix_format; 8],
    pub num_planes: u8,
    pub flags: u8,
    pub ycbcr_enc: u8,
    pub quantization: u8,
    pub xfer_func: u8,
    pub reserved: [u8; 7],
}

/// `struct v4l2_format`. The union holds a pointer-bearing struct, so it is
/// 8-aligned and starts at offset 8.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_format {
    pub type_: u32,
    _pad: u32,
    raw: [u64; 25],
}

impl v4l2_format {
    pub fn new(type_: u32) -> Self {
        Self { type_, _pad: 0, raw: [0; 25] }
    }

    pub fn pix_mp(&self) -> v4l2_pix_format_mplane {
        unsafe { std::ptr::read_unaligned(self.raw.as_ptr() as *const v4l2_pix_format_mplane) }
    }

    pub fn set_pix_mp(&mut self, pix: v4l2_pix_format_mplane) {
        unsafe {
            std::ptr::write_unaligned(self.raw.as_mut_ptr() as *mut v4l2_pix_format_mplane, pix)
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_requestbuffers {
    pub count: u32,
    pub type_: u32,
    pub memory: u32,
    pub capabilities: u32,
    pub flags: u8,
    pub reserved: [u8; 3],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_timecode {
    pub type_: u32,
    pub flags: u32,
    pub frames: u8,
    pub seconds: u8,
    pub minutes: u8,
    pub hours: u8,
    pub userbits: [u8; 4],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_plane {
    pub bytesused: u32,
    pub length: u32,
    /// `mem_offset` (MMAP) in the low 32 bits.
    pub m: u64,
    pub data_offset: u32,
    pub reserved: [u32; 11],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_buffer {
    pub index: u32,
    pub type_: u32,
    pub bytesused: u32,
    pub flags: u32,
    pub field: u32,
    pub timestamp: libc::timeval,
    pub timecode: v4l2_timecode,
    pub sequence: u32,
    pub memory: u32,
    /// For multiplanar buffers, a pointer to the `v4l2_plane` array.
    pub m: u64,
    pub length: u32,
    pub reserved2: u32,
    pub request_fd: i32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_exportbuffer {
    pub type_: u32,
    pub index: u32,
    pub plane: u32,
    pub flags: u32,
    pub fd: i32,
    pub reserved: [u32; 11],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_event_subscription {
    pub type_: u32,
    pub id: u32,
    pub flags: u32,
    pub reserved: [u32; 5],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_event {
    pub type_: u32,
    _pad: u32,
    /// The event's union; for SOURCE_CHANGE, `changes` is the first u32.
    pub u: [u64; 8],
    pub pending: u32,
    pub sequence: u32,
    pub timestamp: libc::timespec,
    pub id: u32,
    pub reserved: [u32; 8],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_decoder_cmd {
    pub cmd: u32,
    pub flags: u32,
    pub raw: [u32; 16],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct v4l2_rect {
    pub left: i32,
    pub top: i32,
    pub width: u32,
    pub height: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_selection {
    pub type_: u32,
    pub target: u32,
    pub flags: u32,
    pub r: v4l2_rect,
    pub reserved: [u32; 9],
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct v4l2_control {
    pub id: u32,
    pub value: i32,
}

/// `struct dma_buf_sync` from `linux/dma-buf.h`.
#[repr(C)]
pub struct dma_buf_sync {
    pub flags: u64,
}

pub const DMA_BUF_SYNC_READ: u64 = 1;
pub const DMA_BUF_SYNC_START: u64 = 0;
pub const DMA_BUF_SYNC_END: u64 = 4;

// ---------- ioctl numbers ----------

const fn ioc(dir: u32, ty: u8, nr: u8, size: usize) -> libc::c_ulong {
    ((dir << 30) | ((size as u32) << 16) | ((ty as u32) << 8) | nr as u32) as libc::c_ulong
}

const W: u32 = 1;
const R: u32 = 2;
const RW: u32 = 3;

use std::mem::size_of;

pub const VIDIOC_QUERYCAP: libc::c_ulong = ioc(R, b'V', 0, size_of::<v4l2_capability>());
pub const VIDIOC_ENUM_FMT: libc::c_ulong = ioc(RW, b'V', 2, size_of::<v4l2_fmtdesc>());
pub const VIDIOC_G_FMT: libc::c_ulong = ioc(RW, b'V', 4, size_of::<v4l2_format>());
pub const VIDIOC_S_FMT: libc::c_ulong = ioc(RW, b'V', 5, size_of::<v4l2_format>());
pub const VIDIOC_REQBUFS: libc::c_ulong = ioc(RW, b'V', 8, size_of::<v4l2_requestbuffers>());
pub const VIDIOC_QUERYBUF: libc::c_ulong = ioc(RW, b'V', 9, size_of::<v4l2_buffer>());
pub const VIDIOC_QBUF: libc::c_ulong = ioc(RW, b'V', 15, size_of::<v4l2_buffer>());
pub const VIDIOC_EXPBUF: libc::c_ulong = ioc(RW, b'V', 16, size_of::<v4l2_exportbuffer>());
pub const VIDIOC_DQBUF: libc::c_ulong = ioc(RW, b'V', 17, size_of::<v4l2_buffer>());
pub const VIDIOC_STREAMON: libc::c_ulong = ioc(W, b'V', 18, size_of::<libc::c_int>());
pub const VIDIOC_STREAMOFF: libc::c_ulong = ioc(W, b'V', 19, size_of::<libc::c_int>());
pub const VIDIOC_G_CTRL: libc::c_ulong = ioc(RW, b'V', 27, size_of::<v4l2_control>());
pub const VIDIOC_DQEVENT: libc::c_ulong = ioc(R, b'V', 89, size_of::<v4l2_event>());
pub const VIDIOC_SUBSCRIBE_EVENT: libc::c_ulong =
    ioc(W, b'V', 90, size_of::<v4l2_event_subscription>());
pub const VIDIOC_G_SELECTION: libc::c_ulong = ioc(RW, b'V', 94, size_of::<v4l2_selection>());
pub const VIDIOC_DECODER_CMD: libc::c_ulong = ioc(RW, b'V', 96, size_of::<v4l2_decoder_cmd>());

pub const DMA_BUF_IOCTL_SYNC: libc::c_ulong = ioc(W, b'b', 0, size_of::<dma_buf_sync>());

/// Runs an ioctl, retrying on EINTR.
pub fn ioctl<T>(fd: RawFd, request: libc::c_ulong, arg: &mut T) -> io::Result<()> {
    loop {
        let ret = unsafe { libc::ioctl(fd, request as _, arg as *mut T) };
        if ret == 0 {
            return Ok(());
        }
        let err = io::Error::last_os_error();
        if err.kind() != io::ErrorKind::Interrupted {
            return Err(err);
        }
    }
}

pub fn zeroed<T>() -> T {
    unsafe { std::mem::zeroed() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::{offset_of, size_of};

    #[test]
    fn layouts_match_videodev2() {
        assert_eq!(size_of::<v4l2_capability>(), 104);
        assert_eq!(size_of::<v4l2_fmtdesc>(), 64);
        assert_eq!(size_of::<v4l2_plane_pix_format>(), 20);
        assert_eq!(size_of::<v4l2_pix_format_mplane>(), 192);
        assert_eq!(size_of::<v4l2_format>(), 208);
        assert_eq!(size_of::<v4l2_requestbuffers>(), 20);
        assert_eq!(size_of::<v4l2_plane>(), 64);
        assert_eq!(offset_of!(v4l2_plane, data_offset), 16);
        assert_eq!(size_of::<v4l2_buffer>(), 88);
        assert_eq!(offset_of!(v4l2_buffer, timestamp), 24);
        assert_eq!(offset_of!(v4l2_buffer, timecode), 40);
        assert_eq!(offset_of!(v4l2_buffer, sequence), 56);
        assert_eq!(offset_of!(v4l2_buffer, m), 64);
        assert_eq!(offset_of!(v4l2_buffer, length), 72);
        assert_eq!(size_of::<v4l2_exportbuffer>(), 64);
        assert_eq!(offset_of!(v4l2_exportbuffer, fd), 16);
        assert_eq!(size_of::<v4l2_event_subscription>(), 32);
        assert_eq!(size_of::<v4l2_event>(), 136);
        assert_eq!(offset_of!(v4l2_event, id), 96);
        assert_eq!(size_of::<v4l2_decoder_cmd>(), 72);
        assert_eq!(size_of::<v4l2_selection>(), 64);
        assert_eq!(offset_of!(v4l2_selection, r), 12);
        assert_eq!(VIDIOC_QUERYCAP, 0x8068_5600);
        assert_eq!(VIDIOC_DQEVENT, 0x8088_5659);
        assert_eq!(VIDIOC_QBUF, 0xc058_560f);
        assert_eq!(VIDIOC_G_FMT, 0xc0d0_5604);
        assert_eq!(VIDIOC_DECODER_CMD, 0xc048_5660);
        assert_eq!(DMA_BUF_IOCTL_SYNC, 0x4008_6200);
    }
}
