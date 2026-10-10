defmodule Membrane.V4L2Decoder.MixProject do
  use Mix.Project

  @version "0.1.0"

  def project do
    [
      app: :membrane_v4l2_decoder,
      version: @version,
      elixir: "~> 1.17",
      description:
        "H.264 decoding through V4L2 stateful (memory-to-memory) hardware decoders, " <>
          "with leased NV12 DMA-BUF VideoInterop frames.",
      start_permanent: Mix.env() == :prod,
      elixirc_paths: elixirc_paths(Mix.env()),
      deps: deps(),
      rustler_opts: rustler_opts(System.get_env("NERVES_SDK_SYSROOT"))
    ]
  end

  def application, do: [extra_applications: [:logger]]

  # dev/ has a soak test to run on the device.
  defp elixirc_paths(:prod), do: ["lib"]
  defp elixirc_paths(_env), do: ["lib", "dev"]

  defp deps do
    [
      {:membrane_core, "~> 1.2"},
      {:membrane_h264_format, "~> 0.6 or ~> 0.7"},
      {:rustler, "~> 0.38.0", runtime: false},
      {:video_interop, "~> 0.1.2"}
    ]
  end

  # Nerves builds cross-compile the NIF with the system's toolchain. The
  # mapping from Nerves toolchains to Rust targets is
  # membrane_video_transcode's.
  @nerves_rust_targets %{
    "armv6-nerves-linux-gnueabihf" => "arm-unknown-linux-gnueabihf",
    "armv7-nerves-linux-gnueabihf" => "armv7-unknown-linux-gnueabihf",
    "aarch64-nerves-linux-gnu" => "aarch64-unknown-linux-gnu",
    "x86_64-nerves-linux-musl" => "x86_64-unknown-linux-musl"
  }

  # Only cross-compiled (Nerves) builds build the NIF: it needs Linux and
  # V4L2. Rustler writes to the source tree's priv/native, which every
  # project using this as a path dependency shares, so a host build would
  # overwrite the target's library.
  defp rustler_opts(nil), do: [skip_compilation?: true]

  defp rustler_opts(sysroot) do
    cc = System.get_env("CC")

    target =
      cc
      |> Path.basename()
      |> String.split("-")
      |> Enum.drop(-1)
      |> Enum.join("-")
      |> then(&Map.fetch!(@nerves_rust_targets, &1))

    key = target |> String.upcase() |> String.replace("-", "_")

    [
      target: target,
      env: [
        {"CARGO_TARGET_#{key}_LINKER", cc},
        {"CARGO_TARGET_#{key}_RUSTFLAGS", "-C link-arg=--sysroot=#{sysroot}"},
        {"CFLAGS", ""},
        {"CC", ""}
      ]
    ]
  end
end
