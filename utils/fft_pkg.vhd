library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

package fft_pkg is
  constant C_FFT_INTERNAL_WIDTH : integer := 32;

  subtype t_fft_word is signed(C_FFT_INTERNAL_WIDTH - 1 downto 0);

  type t_complex_sample is record
    re : t_fft_word;
    im : t_fft_word;
  end record;

  type t_complex_array is array (natural range <>) of t_complex_sample;

  function fft_output_width(sample_width : integer; fft_size : integer) return integer;
  function fft_is_power_of_two(value : integer) return boolean;
  function fft_clip_signed(value : integer; width : integer) return signed;
  function fft_resize_saturate(value : signed; width : integer) return signed;
  function fft_add_complex(a : t_complex_sample; b : t_complex_sample) return t_complex_sample;
  function fft_sub_complex(a : t_complex_sample; b : t_complex_sample) return t_complex_sample;
  function fft_bit_reverse_permute(frame_in : t_complex_array) return t_complex_array;
end package fft_pkg;

package body fft_pkg is
  function fft_output_width(sample_width : integer; fft_size : integer) return integer is
    variable growth : integer;
  begin
    if fft_size <= 1 then
      growth := 0;
    else
      growth := integer(log2(real(fft_size)));
    end if;
    return sample_width + growth;
  end function;

  function fft_is_power_of_two(value : integer) return boolean is
  begin
    if value <= 0 then
      return false;
    end if;
    return value = 2 ** integer(log2(real(value)));
  end function;

  function fft_clip_signed(value : integer; width : integer) return signed is
    variable v_min : integer;
    variable v_max : integer;
  begin
    v_min := -(2 ** (width - 1));
    v_max := (2 ** (width - 1)) - 1;
    if value < v_min then
      return to_signed(v_min, width);
    elsif value > v_max then
      return to_signed(v_max, width);
    else
      return to_signed(value, width);
    end if;
  end function;

  function fft_resize_saturate(value : signed; width : integer) return signed is
    variable result : signed(width - 1 downto 0);
    variable truncated_sign : std_logic;
  begin
    result := resize(value, width);
    truncated_sign := result(result'left);

    if value'length > width then
      for idx in width to value'length - 1 loop
        if value(idx) /= truncated_sign then
          if value(value'left) = '0' then
            return fft_clip_signed((2 ** (width - 1)) - 1, width);
          else
            return fft_clip_signed(-(2 ** (width - 1)), width);
          end if;
        end if;
      end loop;
    end if;

    return result;
  end function;

  function fft_add_complex(a : t_complex_sample; b : t_complex_sample) return t_complex_sample is
  begin
    return (re => a.re + b.re, im => a.im + b.im);
  end function;

  function fft_sub_complex(a : t_complex_sample; b : t_complex_sample) return t_complex_sample is
  begin
    return (re => a.re - b.re, im => a.im - b.im);
  end function;

  function fft_bit_reverse_permute(frame_in : t_complex_array) return t_complex_array is
    variable frame_out : t_complex_array(0 to frame_in'length - 1);
    variable bits : integer;
    variable reversed_idx : integer;
  begin
    if frame_in'length <= 1 then
      return frame_in;
    end if;

    bits := integer(log2(real(frame_in'length)));
    for idx in 0 to frame_in'length - 1 loop
      reversed_idx := 0;
      for bit in 0 to bits - 1 loop
        if ((idx / (2 ** bit)) mod 2) = 1 then
          reversed_idx := reversed_idx + (2 ** (bits - 1 - bit));
        end if;
      end loop;
      frame_out(reversed_idx) := frame_in(idx);
    end loop;

    return frame_out;
  end function;
end package body fft_pkg;
