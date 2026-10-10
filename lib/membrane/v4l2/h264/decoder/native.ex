defmodule Membrane.V4L2.H264.Decoder.Native do
  @moduledoc false
  # The NIF: native/v4l2_decoder_nif, on the decoder in native/v4l2_decoder.

  @rustler_opts Mix.Project.config()[:rustler_opts]

  defmodule DMABufFrame do
    @moduledoc false
    @enforce_keys [
      :coded_width,
      :coded_height,
      :visible_x,
      :visible_y,
      :visible_width,
      :visible_height,
      :modifier,
      :descriptor,
      :acquire_fence_fd,
      :keepalive
    ]
    defstruct @enforce_keys
  end

  use Rustler,
      Keyword.merge([otp_app: :membrane_v4l2_decoder, crate: "v4l2_decoder_nif"], @rustler_opts)

  def create(_pid, _device, _extra_capture, _output_buffers),
    do: :erlang.nif_error(:nif_not_loaded)

  def decode(_decoder, _data, _pts_us), do: :erlang.nif_error(:nif_not_loaded)
  def end_of_stream(_decoder), do: :erlang.nif_error(:nif_not_loaded)
  def close(_decoder), do: :erlang.nif_error(:nif_not_loaded)
  def release_frame(_keepalive), do: :erlang.nif_error(:nif_not_loaded)

  def start_release_dispatcher, do: :erlang.nif_error(:nif_not_loaded)
  def quarantine_release_dispatchers, do: :erlang.nif_error(:nif_not_loaded)
  def release_dispatcher_quarantined, do: :erlang.nif_error(:nif_not_loaded)
  def close_release_dispatcher(_dispatcher, _timeout_ms), do: :erlang.nif_error(:nif_not_loaded)

  def new_abandonment_guard(dispatcher, owner, token, holder) do
    with {:ok, resource} <- new_abandonment_guard_resource(dispatcher, owner, token, holder) do
      {:ok, VideoInterop.AbandonmentGuard.new(resource, __MODULE__)}
    end
  end

  def new_abandonment_guard_resource(_dispatcher, _owner, _token, _holder),
    do: :erlang.nif_error(:nif_not_loaded)

  @behaviour VideoInterop.AbandonmentGuard
  @impl true
  def video_interop_abandonment_guard?(resource), do: abandonment_guard_resource(resource)

  def abandonment_guard_resource(_resource), do: :erlang.nif_error(:nif_not_loaded)
end
