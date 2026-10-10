defmodule Membrane.V4L2.H264.Decoder.Soak do
  @moduledoc """
  A soak test to run on the device: decodes an Annex B file `loops` times
  as fast as the decoder goes, releasing every frame, and reports how many
  came out, how fast, and whether file descriptors leaked.

      Membrane.V4L2.H264.Decoder.Soak.run("/data/videos/t1080.h264",
        loops: 34, width: 1920, height: 1080)
  """

  alias Membrane.V4L2.H264.Decoder.Soak.{Sink, Source}

  defmodule Source do
    @moduledoc false
    use Membrane.Source

    def_options(path: [], loops: [], width: [], height: [])

    def_output_pad(:output,
      flow_control: :manual,
      accepted_format: Membrane.H264
    )

    @impl true
    def handle_init(_ctx, opts) do
      units = opts.path |> File.read!() |> Membrane.V4L2.H264.Decoder.Soak.access_units()
      {[],
       %{units: units, queue: units, loops_left: opts.loops - 1, n: 0, opts: opts, done?: false}}
    end

    @impl true
    def handle_playing(_ctx, state) do
      format = %Membrane.H264{
        alignment: :au,
        stream_structure: :annexb,
        width: state.opts.width,
        height: state.opts.height,
        framerate: {30, 1}
      }

      {[stream_format: {:output, format}], state}
    end

    @impl true
    def handle_demand(:output, size, :buffers, _ctx, state), do: emit(size, state, [])

    defp emit(_size, %{done?: true} = state, _acc), do: {[], state}
    defp emit(0, state, acc), do: {[buffer: {:output, Enum.reverse(acc)}], state}

    defp emit(_size, %{queue: [], loops_left: 0} = state, acc),
      do:
        {[buffer: {:output, Enum.reverse(acc)}, end_of_stream: :output],
         %{state | done?: true}}

    defp emit(size, %{queue: []} = state, acc),
      do: emit(size, %{state | queue: state.units, loops_left: state.loops_left - 1}, acc)

    defp emit(size, %{queue: [unit | rest]} = state, acc) do
      buffer = %Membrane.Buffer{payload: unit, pts: Membrane.Time.milliseconds(state.n * 33)}
      emit(size - 1, %{state | queue: rest, n: state.n + 1}, [buffer | acc])
    end
  end

  defmodule Sink do
    @moduledoc false
    use Membrane.Sink

    def_input_pad(:input, accepted_format: VideoInterop.Format)

    @impl true
    def handle_init(_ctx, _opts), do: {[], %{frames: 0, invalid: 0, first: nil}}

    @impl true
    def handle_buffer(:input, buffer, _ctx, state) do
      frame = buffer.payload
      invalid = if VideoInterop.validate(frame) == :ok, do: 0, else: 1
      first = state.first || Map.take(frame, [:coded_width, :coded_height, :visible_rect])
      :ok = VideoInterop.release(frame)
      {[], %{state | frames: state.frames + 1, invalid: state.invalid + invalid, first: first}}
    end

    @impl true
    def handle_end_of_stream(:input, _ctx, state),
      do: {[notify_parent: {:done, state}], state}
  end

  defmodule HoldingSink do
    @moduledoc false
    # Keeps every frame it gets, as a pipeline stopped mid-play does.
    use Membrane.Sink

    def_input_pad(:input, accepted_format: VideoInterop.Format)

    @impl true
    def handle_init(_ctx, _opts), do: {[], %{held: []}}

    @impl true
    def handle_buffer(:input, buffer, _ctx, state) do
      held = [buffer.payload | state.held]
      if length(held) == 3, do: {[notify_parent: :holding], %{state | held: held}}, else: {[], %{state | held: held}}
    end
  end

  defmodule Pipeline do
    @moduledoc false
    use Membrane.Pipeline

    @impl true
    def handle_init(_ctx, opts) do
      spec =
        child(:source, struct(Source, Map.take(opts, [:path, :loops, :width, :height])))
        |> child(:decoder, Membrane.V4L2.H264.Decoder)
        |> child(:sink, Map.get(opts, :sink, Sink))

      {[spec: spec], %{reporter: opts.reporter}}
    end

    @impl true
    def handle_child_notification(message, _child, _ctx, state) do
      send(state.reporter, {:soak, message})
      {[], state}
    end
  end

  def run(path, opts) do
    fds_before = open_fds()
    started = System.monotonic_time(:millisecond)

    {:ok, _supervisor, pipeline} =
      Membrane.Pipeline.start(Pipeline, %{
        path: path,
        loops: Keyword.get(opts, :loops, 1),
        width: Keyword.fetch!(opts, :width),
        height: Keyword.fetch!(opts, :height),
        reporter: self()
      })

    result =
      receive do
        {:soak, {:done, sink}} -> {:ok, sink}
      after
        Keyword.get(opts, :timeout, 600_000) -> {:error, :timeout}
      end

    elapsed = System.monotonic_time(:millisecond) - started
    :ok = Membrane.Pipeline.terminate(pipeline)
    Process.sleep(500)

    %{result: result, ms: elapsed, fds_before: fds_before, fds_after: open_fds()}
  end

  @doc """
  Stops a pipeline while its sink still holds decoded frames, then plays
  the file again: the second run must start and finish (stopping must
  neither wait for the frames nor leave the decoder unusable).
  """
  def restart(path, opts) do
    {:ok, _supervisor, pipeline} =
      Membrane.Pipeline.start(Pipeline, %{
        path: path,
        loops: 1,
        width: Keyword.fetch!(opts, :width),
        height: Keyword.fetch!(opts, :height),
        reporter: self(),
        sink: HoldingSink
      })

    receive do
      {:soak, :holding} -> :ok
    after
      10_000 -> raise "no frames"
    end

    {stop_us, :ok} = :timer.tc(fn -> Membrane.Pipeline.terminate(pipeline, timeout: 10_000) end)
    second = run(path, opts)
    %{stop_ms: div(stop_us, 1000), second: second}
  end

  defp open_fds, do: "/proc/self/fd" |> File.ls!() |> length()

  @doc "Splits an Annex B stream into access units (see annexb.rs)."
  def access_units(data) do
    starts =
      for {pos, 3} <- :binary.matches(data, <<0, 0, 1>>) do
        code = if pos > 0 and :binary.at(data, pos - 1) == 0, do: pos - 1, else: pos
        {code, pos + 3}
      end

    nals =
      starts
      |> Enum.zip(tl(starts) ++ [{byte_size(data), nil}])
      |> Enum.map(fn {{code, header}, {next, _}} -> {code, header, next} end)
      |> Enum.filter(fn {_code, header, stop} -> header < stop end)

    {units, current, _seen_vcl} =
      Enum.reduce(nals, {[], nil, false}, fn {code, header, stop}, {units, start, seen_vcl} ->
        type = Bitwise.band(:binary.at(data, header), 0x1F)
        vcl = type in [1, 5]

        first_mb_zero =
          vcl and header + 1 < stop and Bitwise.band(:binary.at(data, header + 1), 0x80) != 0

        starts_au = type in [6, 7, 8, 9, 14, 15, 16, 17, 18] or first_mb_zero

        if seen_vcl and starts_au do
          {[binary_part(data, start, code - start) | units], code, vcl}
        else
          {units, start || code, seen_vcl or vcl}
        end
      end)

    Enum.reverse([binary_part(data, current, byte_size(data) - current) | units])
  end
end
