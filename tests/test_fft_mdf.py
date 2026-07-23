import math
import os
import subprocess
import sys
from pathlib import Path

import matplotlib
# matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
from cocotb_tools.runner import Ghdl


FFT_SIZE = 1024
SAMPLE_WIDTH = 16
SPS_VALUES = [1, 2, 4, 8]
ROOT = Path(__file__).resolve().parents[1]
HDLMAKE_MANIFEST = ROOT / "Manifest.py"


def clip_signed(value: int, width: int) -> int:
    min_v = -(1 << (width - 1))
    max_v = (1 << (width - 1)) - 1
    if value < min_v:
        return min_v
    if value > max_v:
        return max_v
    return value


def wrap_to_width(value: int, width: int) -> int:
    mask = (1 << width) - 1
    wrapped = value & mask
    if wrapped & (1 << (width - 1)):
        wrapped -= 1 << width
    return wrapped


def clip_to_sample_width(values: np.ndarray, sample_width: int) -> np.ndarray:
    min_v = -(1 << (sample_width - 1))
    max_v = (1 << (sample_width - 1)) - 1
    return np.clip(values, min_v, max_v).astype(np.int64)


def bit_reverse_permute(values: np.ndarray) -> np.ndarray:
    n = len(values)
    if n <= 1:
        return values
    bits = int(math.log2(n))
    order = np.array([int(format(i, f"0{bits}b")[::-1], 2) for i in range(n)], dtype=np.int64)
    return values[order]


def generate_sine_wave_samples(count: int, sample_width: int, frequency: float) -> tuple[np.ndarray, np.ndarray]:
    amplitude = min(32, (1 << (sample_width - 1)) - 1)
    t = np.arange(count, dtype=np.float64)
    phase = 2.0 * np.pi * frequency * t / max(1, count)
    samples_i = np.round(np.cos(phase) * amplitude).astype(np.int64)
    samples_q = np.round(np.sin(phase) * amplitude).astype(np.int64)
    return samples_i, samples_q


def fft_reference(samples_i, samples_q, sample_width: int, twiddle_fraction_bits: int):
    complex_input = samples_i.astype(np.complex128) + 1j * samples_q.astype(np.complex128)
    n = len(complex_input)
    frame = complex_input.astype(np.complex128).copy()
    twiddle_scale = 2 ** twiddle_fraction_bits
    twiddle_stages = int(math.log2(n))
    stride = 2
    for stage in range(1, twiddle_stages + 1):
        half_stride = stride // 2
        for group in range(n // stride):
            for pair in range(half_stride):
                idx_a = group * stride + pair
                idx_b = idx_a + half_stride
                phase = -2.0 * np.pi * pair / (2 ** stage)
                twiddle_re = int(round(np.cos(phase) * twiddle_scale))
                twiddle_im = int(round(np.sin(phase) * twiddle_scale))
                a = frame[idx_a]
                b = frame[idx_b]
                b_scaled = ((b.real * twiddle_re - b.imag * twiddle_im) / twiddle_scale) + 1j * (
                    (b.real * twiddle_im + b.imag * twiddle_re) / twiddle_scale
                )
                frame[idx_a] = a + b_scaled
                frame[idx_b] = a - b_scaled
        stride *= 2

    spectrum = bit_reverse_permute(frame)
    return clip_to_sample_width(np.real(spectrum), sample_width), clip_to_sample_width(np.imag(spectrum), sample_width)


def pack_words(samples, width):
    packed = 0
    mask = (1 << width) - 1
    for lane, value in enumerate(samples):
        packed |= (int(value) & mask) << (lane * width)
    return int(packed)


def unpack_words(value, width, count):
    out = []
    value = int(value)
    mask = (1 << width) - 1
    for lane in range(count):
        sample = (value >> (lane * width)) & mask
        if sample & (1 << (width - 1)):
            sample -= 1 << width
        out.append(int(sample))
    return out


async def drive_frame(dut, sps, samples_i, samples_q, sample_width: int):
    width = sample_width
    words = (FFT_SIZE + sps - 1) // sps
    unpacked_i = []
    unpacked_q = []

    for word_idx in range(words):
        lane_samples_i = []
        lane_samples_q = []
        for lane in range(sps):
            idx = word_idx * sps + lane
            if idx < FFT_SIZE:
                lane_samples_i.append(samples_i[idx])
                lane_samples_q.append(samples_q[idx])
            else:
                lane_samples_i.append(0)
                lane_samples_q.append(0)

        packed_i = pack_words(lane_samples_i, width)
        packed_q = pack_words(lane_samples_q, width)
        unpacked_i.append(lane_samples_i)
        unpacked_q.append(lane_samples_q)

        dut.Sample_In_I.value = packed_i
        dut.Sample_In_Q.value = packed_q
        dut.Sample_In_V.value = 1
        await RisingEdge(dut.Clk)

    dut.Sample_In_V.value = 0
    await RisingEdge(dut.Clk)
    return unpacked_i, unpacked_q


async def collect_outputs(dut, sps, sample_width: int):
    width = sample_width
    words = (FFT_SIZE + sps - 1) // sps
    out_i = []
    out_q = []

    for _ in range(words):
        while True:
            await RisingEdge(dut.Clk)
            if int(dut.Data_Out_V.value):
                out_i.append(unpack_words(int(dut.Data_Out_I.value), width, sps))
                out_q.append(unpack_words(int(dut.Data_Out_Q.value), width, sps))
                break

    flat_i = [sample for word in out_i for sample in word]
    flat_q = [sample for word in out_q for sample in word]
    return np.array(flat_i[:FFT_SIZE], dtype=np.int64), np.array(flat_q[:FFT_SIZE], dtype=np.int64)


@cocotb.test()
async def test_fft_matches_reference(dut):
    clock = Clock(dut.Clk, 10, unit="ns")
    cocotb.start_soon(clock.start())

    dut.Rst.value = 1
    await Timer(20, unit="ns")
    dut.Rst.value = 0
    await RisingEdge(dut.Clk)

    sps = dut.G_SAMPLES_PER_CLK.value.integer
    twiddle_fraction_bits = int(dut.G_TWIDDLE_WIDTH.value.integer) - 2
    samples_i, samples_q = generate_sine_wave_samples(int(dut.G_FFT_SIZE.value), int(dut.G_SAMPLE_WIDTH.value), 1.0)

    await drive_frame(dut, sps, samples_i, samples_q, int(dut.G_SAMPLE_WIDTH.value))
    got_i, got_q = await collect_outputs(dut, sps, int(dut.G_SAMPLE_WIDTH.value))
    want_i, want_q = fft_reference(
        samples_i.astype(int),
        samples_q.astype(int),
        int(dut.G_SAMPLE_WIDTH.value),
        twiddle_fraction_bits,
    )

    plot_dir = ROOT / "tests" / "sim_build" / f"sps_{sps}"
    plot_dir.mkdir(parents=True, exist_ok=True)
    plot_path = plot_dir / "fft_comparison.png"

    sample_count = min(len(got_i), len(want_i))
    print(f"sps={sps} got_i[:16]={got_i[:16]}")
    print(f"sps={sps} want_i[:16]={want_i[:16]}")
    print(f"sps={sps} got_q[:16]={got_q[:16]}")
    print(f"sps={sps} want_q[:16]={want_q[:16]}")
    print(f"sps={sps} got_i_max_idx={int(np.argmax(np.abs(got_i)))} got_q_max_idx={int(np.argmax(np.abs(got_q)))}")
    print(f"sps={sps} want_i_max_idx={int(np.argmax(np.abs(want_i)))} want_q_max_idx={int(np.argmax(np.abs(want_q)))}")
    x = np.arange(sample_count)
    plt.figure(figsize=(10, 6))
    plt.plot(x, got_i[:sample_count], label="DUT I", marker="o", linewidth=1.0)
    plt.plot(x, want_i[:sample_count], label="Reference I", linestyle="--", linewidth=1.2)
    plt.plot(x, got_q[:sample_count], label="DUT Q", marker="x", linewidth=1.0)
    plt.plot(x, want_q[:sample_count], label="Reference Q", linestyle=":", linewidth=1.2)
    plt.xlabel("Sample index")
    plt.ylabel("Value")
    plt.title(f"FFT comparison for SPS={sps}")
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_path, dpi=150)
    plt.close()

    got_complex = got_i.astype(np.complex128) + 1j * got_q.astype(np.complex128)
    want_complex = want_i.astype(np.complex128) + 1j * want_q.astype(np.complex128)

    got_complex = got_complex / max(1.0, float(np.max(np.abs(got_complex))))
    want_complex = want_complex / max(1.0, float(np.max(np.abs(want_complex))))

    correlation = np.correlate(got_complex, want_complex, mode="full")
    shift = int(np.argmax(np.abs(correlation)) - (len(want_complex) - 1))
    aligned_got_complex = np.roll(got_complex, shift)
    max_diff = float(np.max(np.abs(aligned_got_complex - want_complex)))

    print(f"sps={sps} spectrum_shift={shift} max_complex_diff={max_diff}")
    assert max_diff <= 0.05, (
        f"sps={sps} complex spectrum deviates too far from the reference: max diff {max_diff}"
    )

    dut._log.info("FFT CoCoTB simulation passed")


def load_sources_from_hdlmake(root: Path) -> list[Path]:
    if not HDLMAKE_MANIFEST.exists():
        raise AssertionError(f"Missing hdlmake manifest: {HDLMAKE_MANIFEST}")

    result = subprocess.run(
        ["hdlmake", "-a", "list-files"],
        cwd=root,
        text=True,
        capture_output=True,
        check=True,
    )
    lines = (result.stdout + "\n" + result.stderr).splitlines()
    sources: list[Path] = []
    for raw_line in lines:
        line = raw_line.strip()
        if not line:
            continue
        lower_line = line.lower()
        if not (lower_line.endswith(".vhd") or lower_line.endswith(".vhdl")):
            continue
        path = Path(line)
        if not path.is_absolute():
            path = (root / path).resolve()
        if path.exists():
            sources.append(path)
    if not sources:
        raise AssertionError("hdlmake returned no VHDL files")
    return sources


def main() -> None:
    root = ROOT
    sim_build = root / "tests" / "sim_build"
    sim_build.mkdir(parents=True, exist_ok=True)

    source_files = load_sources_from_hdlmake(root)

    runner = Ghdl()
    for sps in SPS_VALUES:
        parameters = {
            "G_SAMPLE_WIDTH": SAMPLE_WIDTH,
            "G_TWIDDLE_WIDTH": 16,
            "G_FFT_SIZE": FFT_SIZE,
            "G_SAMPLES_PER_CLK": sps,
            "G_DATA_WIDTH": SAMPLE_WIDTH,
            "G_DATA_OUT_WIDTH": SAMPLE_WIDTH,
        }
        build_dir = sim_build / f"sps_{sps}"
        build_dir.mkdir(parents=True, exist_ok=True)

        runner.build(
            sources=[str(path) for path in source_files],
            build_dir=build_dir,
            hdl_toplevel="fft_mdf_cocotb_wrapper",
            parameters=parameters,
        )
        runner.test(
            test_module="test_fft_mdf",
            hdl_toplevel="fft_mdf_cocotb_wrapper",
            build_dir=build_dir,
            test_dir=build_dir,
            hdl_toplevel_lang="vhdl",
            parameters=parameters,
            extra_env={"FFT_SPS": str(sps)},
        )


if __name__ == "__main__":
    main()
