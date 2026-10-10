defmodule Membrane.V4L2.H264.Decoder do
  @moduledoc """
  Decodes H.264 with a V4L2 stateful (memory-to-memory) hardware decoder,
  such as the Fairphone 3's Qualcomm Venus, into leased NV12 DMA-BUF
  `%VideoInterop.Frame{}` buffers.

  The output matches `Membrane.H264.Decoder` from membrane_video_transcode
  in `:dmabuf` mode, so `Membrane.VideoInterop.Sink` can hand the frames to
  an Emerge video target without a copy.

  Input is Annex B access units (`Membrane.H264.Parser` with
  `output_alignment: :au, output_stream_structure: :annexb`). Frames come
  out in display order with the input's pts.

  A native worker thread owns the device. Each decoded frame is one of the
  driver's CAPTURE buffers, leased downstream through a
  `VideoInterop.LeaseOwner`; the driver gets it back when the lease is
  released. Input is only demanded while the driver has room for it, so
  when every buffer is leased the pipeline waits rather than dropping
  access units.

  Two things the driver gets wrong on the Fairphone 3 are worked around:

    * Venus reports the decoded size as the visible area (1088 lines for
      1080p), so the picture size comes from the input stream format.
    * Venus decodes everything after the stop command but never flags the
      last buffer, so the stream ends when every access unit sent has come
      back as a frame, or after a short idle period.

  Each frame describes just the picture: its size is the visible size
  (1280x720, not the decoder's 1280x736 buffer) and its visible rectangle
  is the whole frame, as Emerge's video targets require. The planes keep
  the driver's layout, so the rows of padding between luma and chroma
  simply aren't part of the image.

  The first resolution change is handled; a later one fails the element.
  """

  use Membrane.Filter

  require Membrane.Logger

  alias Membrane.Buffer
  alias Membrane.H264
  alias Membrane.V4L2.H264.Decoder.{Native, ReleaseDispatcherCustodian}
  alias VideoInterop.{Format, Frame, LeaseOwner, Rect, SyncFile}
  alias VideoInterop.DMABuf.Format, as: StorageFormat

  @nv12_fourcc :binary.decode_unsigned("NV12", :little)

  def_options(
    device: [
      spec: :auto | Path.t(),
      default: :auto,
      description: """
      The decoder's device node, or `:auto` to find the first multiplanar
      memory-to-memory device that takes H.264 (`/dev/videoN` numbers move
      between boots).
      """
    ],
    max_in_flight: [
      spec: pos_integer(),
      default: 4,
      description: """
      How many decoded frames may be leased downstream at once, beyond what
      the decoder itself keeps for reference.
      """
    ],
    output_buffers: [
      spec: pos_integer(),
      default: 8,
      description: "Compressed input buffers in the driver."
    ]
  )

  def_input_pad(:input,
    flow_control: :manual,
    demand_unit: :buffers,
    accepted_format: %H264{alignment: :au, stream_structure: :annexb}
  )

  def_output_pad(:output,
    flow_control: :push,
    accepted_format: %Format{storage: %StorageFormat{fourcc: @nv12_fourcc}}
  )

  @impl true
  def handle_init(_ctx, opts) do
    {[],
     %{
       opts: opts,
       decoder: nil,
       lease_owner: nil,
       dispatcher: nil,
       custodian: nil,
       input_format: nil,
       output_format: nil,
       playing?: false,
       wants_input?: false,
       next_pts: 0
     }}
  end

  @impl true
  def handle_setup(_ctx, state) do
    device = if state.opts.device == :auto, do: "", else: to_string(state.opts.device)

    with :ok <- check_dispatcher(),
         {:ok, custodian, dispatcher} <- ReleaseDispatcherCustodian.start(self()),
         {:ok, owner} <- start_lease_owner(dispatcher),
         :ok <- ReleaseDispatcherCustodian.watch(custodian, owner),
         {:ok, decoder} <-
           Native.create(
             self(),
             device,
             state.opts.max_in_flight,
             state.opts.output_buffers
           ) do
      {[],
       %{
         state
         | decoder: decoder,
           lease_owner: owner,
           dispatcher: dispatcher,
           custodian: custodian
       }}
    else
      {:error, reason} -> raise "could not start the V4L2 H.264 decoder: #{inspect(reason)}"
    end
  end

  @impl true
  def handle_playing(_ctx, state) do
    actions = if state.wants_input?, do: [demand: {:input, 1}], else: []
    {actions, %{state | playing?: true, wants_input?: false}}
  end

  @impl true
  def handle_stream_format(:input, format, _ctx, state),
    do: {[], %{state | input_format: format}}

  @impl true
  def handle_buffer(:input, %Buffer{} = buffer, _ctx, state) do
    pts = buffer.pts || buffer.dts || state.next_pts
    :ok = Native.decode(state.decoder, buffer.payload, Membrane.Time.as_microseconds(pts, :round))
    {[], %{state | next_pts: pts + 1}}
  end

  @impl true
  def handle_end_of_stream(:input, _ctx, state) do
    :ok = Native.end_of_stream(state.decoder)
    {[], state}
  end

  @impl true
  def handle_info({:v4l2_decoder, :need_input}, _ctx, state) do
    if state.playing?,
      do: {[demand: {:input, 1}], state},
      else: {[], %{state | wants_input?: true}}
  end

  def handle_info({:v4l2_decoder, {:frame, native, pts_us}}, _ctx, state) do
    picture = picture(native, state.input_format)
    {format_actions, state} = maybe_stream_format(native, picture, state)

    case LeaseOwner.issue(state.lease_owner, native.keepalive) do
      {:ok, lease} ->
        frame = %Frame{
          coded_width: picture.width,
          coded_height: picture.height,
          visible_rect: %Rect{x: 0, y: 0, width: picture.width, height: picture.height},
          format: state.output_format,
          storage: crop(native.descriptor, picture),
          acquire_sync: %SyncFile{acquire_fence_fd: native.acquire_fence_fd},
          lease: lease
        }

        buffer = %Buffer{pts: Membrane.Time.microseconds(pts_us), payload: frame}
        {format_actions ++ [buffer: {:output, buffer}], state}

      {:error, {:caller_owned, reason}} ->
        :ok = Native.release_frame(native.keepalive)
        raise "could not lease a decoded frame: #{inspect(reason)}"

      {:error, reason} ->
        raise "could not lease a decoded frame: #{inspect(reason)}"
    end
  end

  def handle_info({:v4l2_decoder, :end_of_stream}, _ctx, state),
    do: {[end_of_stream: :output], state}

  def handle_info({:v4l2_decoder, {:error, reason}}, _ctx, _state),
    do: raise("V4L2 H.264 decoder failed: #{reason}")

  def handle_info({:video_interop_lease_release_failed, owner, _token, _meta, reason}, _ctx, %{
        lease_owner: owner
      }),
      do: raise("releasing a decoded frame failed: #{inspect(reason)}")

  def handle_info(_message, _ctx, state), do: {[], state}

  # Doesn't wait for frames still downstream: the lease owner outlives this
  # element until they're all back (released, or abandoned when whatever
  # held them dies), and the custodian closes the dispatcher after that.
  @impl true
  def handle_terminate_request(_ctx, state) do
    if state.decoder, do: :ok = Native.close(state.decoder)
    if state.custodian, do: ReleaseDispatcherCustodian.retire(state.custodian)
    {[terminate: :normal], %{state | decoder: nil}}
  end

  # The part of the decoded buffer that is the picture. Venus reports the
  # decoded (macroblock-aligned) size as the visible area; the parser's
  # stream format has the real one.
  defp picture(native, %H264{width: width, height: height})
       when is_integer(width) and is_integer(height) and width > 0 and height > 0 do
    %Rect{
      x: native.visible_x,
      y: native.visible_y,
      width: min(width, native.visible_width),
      height: min(height, native.visible_height)
    }
  end

  defp picture(native, _format) do
    %Rect{
      x: native.visible_x,
      y: native.visible_y,
      width: native.visible_width,
      height: native.visible_height
    }
  end

  # Moves the NV12 planes' offsets to the picture's top-left corner (a
  # no-op for Venus, whose pictures start at 0,0).
  defp crop(descriptor, %Rect{x: 0, y: 0}), do: descriptor

  defp crop(descriptor, %Rect{x: x, y: y}) do
    layers =
      for layer <- descriptor.layers do
        [luma, chroma] = layer.planes

        %{
          layer
          | planes: [
              %{luma | offset: luma.offset + y * luma.pitch + x},
              %{chroma | offset: chroma.offset + div(y, 2) * chroma.pitch + x}
            ]
        }
      end

    %{descriptor | layers: layers}
  end

  defp maybe_stream_format(native, picture, %{output_format: nil} = state) do
    format = %Format{
      width: picture.width,
      height: picture.height,
      framerate: framerate(state.input_format),
      storage: %StorageFormat{fourcc: @nv12_fourcc, modifier: native.modifier},
      acquire_sync: :sync_file
    }

    :ok = VideoInterop.validate(format)
    {[stream_format: {:output, format}], %{state | output_format: format}}
  end

  defp maybe_stream_format(native, picture, state) do
    if picture.width != state.output_format.width or
         picture.height != state.output_format.height do
      :ok = Native.release_frame(native.keepalive)
      raise "the decoded size changed mid-stream, which isn't supported yet"
    end

    {[], state}
  end

  defp framerate(%H264{framerate: {num, den}}) when num > 0 and den > 0, do: {num, den}
  defp framerate(_format), do: nil

  defp check_dispatcher do
    if Native.release_dispatcher_quarantined(),
      do: {:error, :release_dispatcher_quarantined},
      else: :ok
  end

  defp start_lease_owner(dispatcher) do
    LeaseOwner.start_link(
      producer: self(),
      notify: self(),
      # Bounded by the driver's CAPTURE buffers anyway; this is a backstop.
      max_active: 64,
      release: {Native, :release_frame, []},
      release_retry: {:exponential, initial_ms: 10, max_ms: 1_000, max_attempts: :infinity},
      abandonment_guard_factory: fn owner, token, holder ->
        Native.new_abandonment_guard(dispatcher, owner, token, holder)
      end
    )
  end
end
