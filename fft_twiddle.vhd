library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

library work;
  use work.fft_pkg.all;

package fft_twiddle is
  function fft_twiddle_value(stage : integer; index : integer; is_forward : boolean; twiddle_width : integer; twiddle_fraction_bits : integer) return t_complex_sample;
end package fft_twiddle;

package body fft_twiddle is
  function fft_twiddle_value(stage : integer; index : integer; is_forward : boolean; twiddle_width : integer; twiddle_fraction_bits : integer) return t_complex_sample is
    variable phase : real;
    variable tw_re : integer;
    variable tw_im : integer;
    variable sign_factor : real;
    variable denom : integer;
  begin
    if is_forward then
      sign_factor := -1.0;
    else
      sign_factor := 1.0;
    end if;
    if stage <= 0 then
      denom := 1;
    else
      denom := 2 ** stage;
    end if;
    phase := sign_factor * 2.0 * math_pi * real(index) / real(denom);
    tw_re := integer(round(cos(phase) * real(2 ** twiddle_fraction_bits)));
    tw_im := integer(round(sin(phase) * real(2 ** twiddle_fraction_bits)));
    return (
      re => resize(fft_clip_signed(tw_re, twiddle_width), C_FFT_INTERNAL_WIDTH),
      im => resize(fft_clip_signed(tw_im, twiddle_width), C_FFT_INTERNAL_WIDTH)
    );
  end function;
end package body fft_twiddle;
