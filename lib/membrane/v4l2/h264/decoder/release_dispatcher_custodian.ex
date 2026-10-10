defmodule Membrane.V4L2.H264.Decoder.ReleaseDispatcherCustodian do
  @moduledoc false
  # Owns one decoder element's release dispatcher (the native thread that
  # runs abandonment releases), and closes it when the frames are back.
  #
  # The element's `VideoInterop.LeaseOwner` outlives the element until every
  # lease is released: explicitly, or through its abandonment guard when a
  # process holding a frame dies. So the element doesn't wait for frames at
  # shutdown (a frame stuck in a dying pipeline would hold it up). It
  # retires, or dies, and this process waits for the lease owner to finish,
  # then closes the dispatcher.
  #
  # After `@max_wait_ms` it closes the dispatcher anyway: guards dropped
  # after that are inert, and explicit releases still work. Only a
  # dispatcher that fails to close quarantines new ones (until the VM
  # restarts), as its native state can't be trusted then.

  use GenServer

  require Logger

  alias Membrane.V4L2.H264.Decoder.Native

  @close_timeout_ms 5_000
  @max_wait_ms 120_000

  @doc "Starts a custodian for the calling element, with its dispatcher."
  @spec start(pid()) :: {:ok, pid(), reference()} | {:error, term()}
  def start(owner) when owner == self() do
    with {:ok, custodian} <- GenServer.start(__MODULE__, owner),
         {:ok, dispatcher} <- call(custodian, :dispatcher) do
      {:ok, custodian, dispatcher}
    end
  end

  def start(_owner), do: {:error, :owner_must_be_caller}

  @doc "The lease owner whose leases must be back before closing."
  def watch(custodian, lease_owner), do: GenServer.cast(custodian, {:watch, lease_owner})

  @doc "The element is done with the dispatcher; close it once leases drain."
  def retire(custodian), do: GenServer.cast(custodian, :retire)

  @impl true
  def init(owner) do
    case Native.start_release_dispatcher() do
      {:ok, dispatcher} ->
        {:ok,
         %{
           owner: owner,
           owner_monitor: Process.monitor(owner),
           dispatcher: dispatcher,
           lease_owner: nil,
           retired?: false
         }}

      {:error, reason} ->
        {:stop, {:release_dispatcher_start_failed, reason}}
    end
  end

  @impl true
  def handle_call(:dispatcher, {owner, _tag}, %{owner: owner} = state),
    do: {:reply, {:ok, state.dispatcher}, state}

  def handle_call(:dispatcher, _from, state), do: {:reply, {:error, :not_owner}, state}

  @impl true
  def handle_cast({:watch, lease_owner}, state) do
    Process.monitor(lease_owner)
    {:noreply, %{state | lease_owner: lease_owner}}
  end

  def handle_cast(:retire, state), do: retired(state)

  @impl true
  def handle_info({:DOWN, ref, :process, _pid, _reason}, %{owner_monitor: ref} = state),
    do: retired(state)

  def handle_info({:DOWN, _ref, :process, pid, _reason}, %{lease_owner: pid} = state),
    do: close(%{state | lease_owner: nil})

  def handle_info(:max_wait, state) do
    Logger.warning("[v4l2 decoder] decoded frames still out after #{@max_wait_ms} ms; closing")
    close(state)
  end

  def handle_info(_message, state), do: {:noreply, state}

  defp retired(%{retired?: true} = state), do: {:noreply, state}

  defp retired(state) do
    state = %{state | retired?: true}

    if state.lease_owner && Process.alive?(state.lease_owner) do
      Process.send_after(self(), :max_wait, @max_wait_ms)
      {:noreply, state}
    else
      close(state)
    end
  end

  defp close(%{retired?: false} = state), do: {:noreply, state}

  defp close(state) do
    case Native.close_release_dispatcher(state.dispatcher, @close_timeout_ms) do
      {:ok, true} ->
        {:stop, :normal, state}

      error ->
        Logger.error("[v4l2 decoder] release dispatcher didn't close: #{inspect(error)}")
        _newly_quarantined? = Native.quarantine_release_dispatchers()
        {:stop, :normal, state}
    end
  end

  defp call(custodian, request) do
    GenServer.call(custodian, request, @close_timeout_ms)
  catch
    :exit, reason -> {:error, {:custodian_call_failed, reason}}
  end
end
