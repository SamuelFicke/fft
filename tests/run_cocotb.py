#!/usr/bin/env python3
import os
import sys
from pathlib import Path

from cocotb_tools.runner import Ghdl

ROOT = Path(__file__).resolve().parent
VHDL_SOURCES = [
    ROOT / "../fft_pkg.vhd",
    ROOT / "../fft_twiddle.vhd",
    ROOT / "../signed_multiplier.vhd",
    ROOT / "../fft_mdf.vhd",
    ROOT / "fft_mdf_cocotb_wrapper.vhd",
]


def main() -> None:
    os.environ.setdefault("TOPLEVEL_LANG", "vhdl")
    os.environ.setdefault("TOPLEVEL", "fft_mdf_cocotb_wrapper")
    os.environ.setdefault("MODULE", "test_fft_mdf")
    os.environ.setdefault("COCOTB_TEST_MODULES", "test_fft_mdf")

    sim_build = ROOT / "sim_build"
    sim_build.mkdir(exist_ok=True)

    runner = Ghdl()
    runner.build(
        sources=[str(path) for path in VHDL_SOURCES],
        build_dir=sim_build,
        hdl_toplevel="fft_mdf_cocotb_wrapper",
    )
    runner.test(
        test_module="test_fft_mdf",
        hdl_toplevel="fft_mdf_cocotb_wrapper",
        build_dir=sim_build,
        test_dir=sim_build,
        hdl_toplevel_lang="vhdl",
    )


if __name__ == "__main__":
    main()
