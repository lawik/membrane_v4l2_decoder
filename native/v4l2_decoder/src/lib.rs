//! H.264 decoding through a V4L2 stateful memory-to-memory decoder (the
//! Fairphone 3's Venus), with decoded frames exported as DMA-BUFs.
//!
//! The NIF behind `Membrane.V4L2.H264.Decoder` is the `v4l2_decoder_nif`
//! crate next to this one.

pub mod annexb;
pub mod decoder;
pub mod v4l2;

pub use decoder::{CaptureInfo, Decoder, Event, Options};
