# membrane_v4l2_decoder

`Membrane.V4L2.H264.Decoder`: H.264 decoding through a V4L2 stateful
(memory-to-memory) hardware decoder, with each decoded frame handed on as a
leased NV12 DMA-BUF `%VideoInterop.Frame{}`, the same output as
membrane_video_transcode's decoder in `:dmabuf` mode. Pair it with
`Membrane.VideoInterop.Sink` to show frames in an Emerge video target
without copying them.

Written for the Fairphone 3's Qualcomm Venus (msm8953, nerves_system_fp3),
but nothing in it is Venus-specific apart from two workarounds.

## Layout

| Path | What |
|---|---|
| `lib/membrane/v4l2/h264/decoder.ex` | The element: leases, stream format, demand |
| `native/v4l2_decoder` | The decoder in Rust (V4L2 ioctls, DMA-BUF export), and `v4l2-dec-dump` |
| `native/v4l2_decoder_nif` | The NIF: a worker thread that owns the device |
| `dev/soak.ex` | A soak test to run on the device |

## How it works

A worker thread owns the device and polls it. Access units go into mmapped
OUTPUT buffers. On the first source-change event the CAPTURE buffers are
allocated (the driver's minimum plus `max_in_flight`) and each is exported
once as a DMA-BUF. A dequeued CAPTURE buffer becomes a frame whose
keepalive holds a duplicate of its fd and a sync file; releasing the lease
sends the index back to the worker, which queues the buffer again. Input is
demanded only while an OUTPUT buffer is free, so when every frame is leased
the pipeline waits rather than dropping data.

The device is found by capability: the first multiplanar M2M node whose
OUTPUT queue takes H.264 and whose CAPTURE queue gives NV12.

## Measured on the Fairphone 3

* Decoder `qcom-venus` at `/dev/video0` (encoder at `/dev/video1`; numbers
  can move). OUTPUT: H.264, VP8, VP9, HEVC, VC-1, MPEG-2/4, H.263, up to
  4096x4096. CAPTURE: NV12 or QC08C.
* NV12 in one buffer, chroma at `stride * coded_height`. 720p decodes to
  1280x736, 1080p to 1920x1088 (3,145,728-byte buffers). Output is
  bit-exact with FFmpeg's software decoder (600 frames compared).
* About 150 fps at 1080p. 10,200 frames through the element with every
  lease released: no leaked fds, no lost buffers.

## Workarounds

* **End of stream.** After `V4L2_DEC_CMD_STOP`, Venus decodes everything
  but never returns a buffer flagged `V4L2_BUF_FLAG_LAST`, nor the EOS
  event (FFmpeg's `h264_v4l2m2m` hangs on the same thing). Without the stop
  it holds back the last few frames, so the stop is needed. The stream
  ends when every access unit sent has come back as a frame, or after
  750 ms without output. The fix belongs in the kernel driver
  (`vdec_decoder_cmd` in `drivers/media/platform/qcom/venus/vdec.c`).
* **Visible area.** Venus reports the compose rectangle as the full
  decoded size for 1080p (1088 lines), so the visible rectangle comes from
  the input `%Membrane.H264{}` stream format.

A resolution change after the first is not supported yet; the element
fails with a clear error.

## Building

`mix compile` builds the NIF with Rustler. In a Nerves build it is
cross-compiled with the system's toolchain (the mapping from Nerves
toolchains to Rust targets is membrane_video_transcode's).

The command-line tool, for checking a decoder outside the BEAM:

    cd native/v4l2_decoder
    cargo build --release --target aarch64-unknown-linux-gnu
    v4l2-dec-dump in.h264 out.nv12 [--frames N] [--no-stop] [--device /dev/videoN]

(cross-linking needs `.cargo/config.toml` pointing at the Nerves toolchain
and sysroot; it's machine-specific and not checked in).
