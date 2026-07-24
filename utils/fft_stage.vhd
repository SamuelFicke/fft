library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.fft_pkg.all;
  use work.fft_twiddle.all;

entity fft_stage is
  generic (
    G_STAGE_INDEX     : integer := 1;
    G_FFT_SIZE        : integer := 8;
    G_SAMPLES_PER_CLK : integer := 1;
    G_BUTTERFLY_LANES : integer := 1;
    G_TWIDDLE_WIDTH   : integer := 16;
    G_IS_FORWARD      : boolean := true;
    G_INTERNAL_WIDTH  : integer := C_FFT_INTERNAL_WIDTH;
    G_MUL_PIPE_STAGES : integer := 1
  );
  port (
    Clk             : in  std_logic;
    Rst             : in  std_logic;
    Sample_In       : in  t_complex_sample;
    Sample_In_V     : in  std_logic;
    Sample_In_Ready : out std_logic;
    Sample_Out      : out t_complex_sample;
    Sample_Out_V    : out std_logic;
    Sample_Out_Ready : in std_logic
  );
end entity fft_stage;

architecture rtl of fft_stage is
  function min_int(lhs : integer; rhs : integer) return integer is
  begin
    if lhs < rhs then
      return lhs;
    end if;
    return rhs;
  end function;

  constant C_STRIDE                : integer := 2 ** G_STAGE_INDEX;
  constant C_HALF_STRIDE           : integer := C_STRIDE / 2;
  constant C_BUTTERFLIES_PER_STAGE : integer := G_FFT_SIZE / 2;
  constant C_BUTTERFLY_LANES       : integer := min_int(C_BUTTERFLIES_PER_STAGE, G_BUTTERFLY_LANES);
  constant C_TWIDDLE_FRACTION_BITS : integer := G_TWIDDLE_WIDTH - 2;
  constant C_ZERO_SAMPLE           : t_complex_sample := ((others => '0'), (others => '0'));

  signal Input_Frame_R      : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Input_Count_R      : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Frame_Ready_R      : std_logic := '0';
  signal Output_Frame_R     : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Output_Ready_R     : std_logic := '0';
  signal Output_Count_R     : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Delay_Buffer_R     : t_complex_array(0 to C_HALF_STRIDE - 1) := (others => C_ZERO_SAMPLE);
  signal Delay_Index_R      : integer range 0 to C_HALF_STRIDE - 1 := 0;
  signal Sample_Out_V_R     : std_logic := '0';
begin
  assert G_STAGE_INDEX > 0 report "G_STAGE_INDEX must be greater than zero" severity failure;
  assert G_FFT_SIZE > 1 report "G_FFT_SIZE must be greater than 1" severity failure;
  assert C_BUTTERFLY_LANES > 0 report "C_BUTTERFLY_LANES must be greater than zero" severity failure;

  Sample_In_Ready <= '1' when Frame_Ready_R = '0' and Output_Ready_R = '0' else '0';
  Sample_Out_V <= Sample_Out_V_R;

  process(Clk)
    variable stage_frame_v      : t_complex_array(0 to G_FFT_SIZE - 1);
    variable twiddle_v          : t_complex_sample;
    variable stage_butterfly_idx_v : integer;
    variable stage_group_idx_v  : integer;
    variable stage_pair_idx_v   : integer;
    variable idx_a_v            : integer;
    variable idx_b_v            : integer;
    variable a_re_v             : signed(G_INTERNAL_WIDTH - 1 downto 0);
    variable a_im_v             : signed(G_INTERNAL_WIDTH - 1 downto 0);
    variable b_re_v             : signed(G_INTERNAL_WIDTH - 1 downto 0);
    variable b_im_v             : signed(G_INTERNAL_WIDTH - 1 downto 0);
    variable tw_re_v            : signed(G_INTERNAL_WIDTH - 1 downto 0);
    variable tw_im_v            : signed(G_INTERNAL_WIDTH - 1 downto 0);
    variable mul_re_wide        : signed(2 * G_INTERNAL_WIDTH downto 0);
    variable mul_im_wide        : signed(2 * G_INTERNAL_WIDTH downto 0);
    variable sum_re_wide        : signed(2 * G_INTERNAL_WIDTH downto 0);
    variable sum_im_wide        : signed(2 * G_INTERNAL_WIDTH downto 0);
    variable diff_re_wide       : signed(2 * G_INTERNAL_WIDTH downto 0);
    variable diff_im_wide       : signed(2 * G_INTERNAL_WIDTH downto 0);
  begin
    if rising_edge(Clk) then
      if Rst = '1' then
        Input_Frame_R <= (others => C_ZERO_SAMPLE);
        Input_Count_R <= 0;
        Frame_Ready_R <= '0';
        Output_Frame_R <= (others => C_ZERO_SAMPLE);
        Output_Ready_R <= '0';
        Output_Count_R <= 0;
        Delay_Buffer_R <= (others => C_ZERO_SAMPLE);
        Delay_Index_R <= 0;
        Sample_Out_V_R <= '0';
      else
        Sample_Out_V_R <= '0';

        if Sample_In_V = '1' and Frame_Ready_R = '0' and Output_Ready_R = '0' then
          Input_Frame_R(Input_Count_R) <= Sample_In;
          Delay_Buffer_R(Delay_Index_R) <= Sample_In;
          if Delay_Index_R = C_HALF_STRIDE - 1 then
            Delay_Index_R <= 0;
          else
            Delay_Index_R <= Delay_Index_R + 1;
          end if;

          if Input_Count_R = G_FFT_SIZE - 1 then
            Frame_Ready_R <= '1';
            Input_Count_R <= 0;
          else
            Input_Count_R <= Input_Count_R + 1;
          end if;
        end if;

        if Frame_Ready_R = '1' and Output_Ready_R = '0' then
          stage_frame_v := Input_Frame_R;
          for butterfly_idx in 0 to C_BUTTERFLIES_PER_STAGE - 1 loop
            stage_butterfly_idx_v := butterfly_idx;
            stage_group_idx_v := stage_butterfly_idx_v / C_HALF_STRIDE;
            stage_pair_idx_v := stage_butterfly_idx_v mod C_HALF_STRIDE;
            idx_a_v := stage_group_idx_v * C_STRIDE + stage_pair_idx_v;
            idx_b_v := idx_a_v + C_HALF_STRIDE;
            twiddle_v := fft_twiddle_value(G_STAGE_INDEX, stage_pair_idx_v, G_IS_FORWARD, G_TWIDDLE_WIDTH, C_TWIDDLE_FRACTION_BITS);

            a_re_v := stage_frame_v(idx_a_v).re;
            a_im_v := stage_frame_v(idx_a_v).im;
            b_re_v := stage_frame_v(idx_b_v).re;
            b_im_v := stage_frame_v(idx_b_v).im;
            tw_re_v := twiddle_v.re;
            tw_im_v := twiddle_v.im;

            mul_re_wide := resize(shift_right((b_re_v * tw_re_v) - (b_im_v * tw_im_v), C_TWIDDLE_FRACTION_BITS), 2 * G_INTERNAL_WIDTH + 1);
            mul_im_wide := resize(shift_right((b_re_v * tw_im_v) + (b_im_v * tw_re_v), C_TWIDDLE_FRACTION_BITS), 2 * G_INTERNAL_WIDTH + 1);
            sum_re_wide := resize(a_re_v, 2 * G_INTERNAL_WIDTH + 1) + mul_re_wide;
            sum_im_wide := resize(a_im_v, 2 * G_INTERNAL_WIDTH + 1) + mul_im_wide;
            diff_re_wide := resize(a_re_v, 2 * G_INTERNAL_WIDTH + 1) - mul_re_wide;
            diff_im_wide := resize(a_im_v, 2 * G_INTERNAL_WIDTH + 1) - mul_im_wide;

            stage_frame_v(idx_a_v).re := fft_resize_saturate(resize(sum_re_wide, G_INTERNAL_WIDTH), G_INTERNAL_WIDTH);
            stage_frame_v(idx_a_v).im := fft_resize_saturate(resize(sum_im_wide, G_INTERNAL_WIDTH), G_INTERNAL_WIDTH);
            stage_frame_v(idx_b_v).re := fft_resize_saturate(resize(diff_re_wide, G_INTERNAL_WIDTH), G_INTERNAL_WIDTH);
            stage_frame_v(idx_b_v).im := fft_resize_saturate(resize(diff_im_wide, G_INTERNAL_WIDTH), G_INTERNAL_WIDTH);
          end loop;

          Output_Frame_R <= stage_frame_v;
          Output_Ready_R <= '1';
          Output_Count_R <= 0;
          Frame_Ready_R <= '0';
        end if;

        if Output_Ready_R = '1' and Sample_Out_Ready = '1' and Output_Count_R < G_FFT_SIZE - 1 then
          Sample_Out <= Output_Frame_R(Output_Count_R);
          Sample_Out_V_R <= '1';
          Output_Count_R <= Output_Count_R + 1;
        elsif Output_Ready_R = '1' and Sample_Out_Ready = '1' and Output_Count_R = G_FFT_SIZE - 1 then
          Sample_Out <= Output_Frame_R(Output_Count_R);
          Sample_Out_V_R <= '1';
          Output_Ready_R <= '0';
        end if;
      end if;
    end if;
  end process;
end architecture rtl;
