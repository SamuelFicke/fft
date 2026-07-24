library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.fft_pkg.all;
  use work.fft_twiddle.all;

entity fft_stage is
  generic (
    G_STAGE_INDEX       : integer := 1;
    G_FFT_SIZE          : integer := 8;
    G_SAMPLES_PER_CLK   : integer := 1;
    G_BUTTERFLY_LANES   : integer := 1;
    G_TWIDDLE_WIDTH     : integer := 16;
    G_IS_FORWARD        : boolean := true;
    G_INTERNAL_WIDTH    : integer := C_FFT_INTERNAL_WIDTH;
    G_MUL_PIPE_STAGES   : integer := 1
  );
  port (
    Clk         : in  std_logic;
    Rst         : in  std_logic;
    Frame_In    : in  t_complex_array(0 to G_FFT_SIZE - 1);
    Frame_In_V  : in  std_logic;
    Frame_In_Ready : out std_logic;
    Frame_Out   : out t_complex_array(0 to G_FFT_SIZE - 1);
    Frame_Out_V : out std_logic;
    Frame_Out_Ready : in std_logic
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

  constant C_STRIDE               : integer := 2 ** G_STAGE_INDEX;
  constant C_HALF_STRIDE          : integer := C_STRIDE / 2;
  constant C_BUTTERFLIES_PER_STAGE: integer := G_FFT_SIZE / 2;
  constant C_BUTTERFLY_LANES      : integer := min_int(C_BUTTERFLIES_PER_STAGE, G_BUTTERFLY_LANES);
  constant C_TWIDDLE_FRACTION_BITS: integer := G_TWIDDLE_WIDTH - 2;

  subtype t_fft_word is signed(G_INTERNAL_WIDTH - 1 downto 0);
  type t_fft_word_array is array (natural range <>) of t_fft_word;
  type t_logic_array is array (natural range <>) of std_logic;
  type t_index_array is array (natural range <>) of integer range 0 to G_FFT_SIZE - 1;

  signal Active_R           : std_logic := '0';
  signal Working_Frame_R    : t_complex_array(0 to G_FFT_SIZE - 1) := (others => ((others => '0'), (others => '0')));
  signal Issued_R           : integer range 0 to C_BUTTERFLIES_PER_STAGE := 0;
  signal Completed_R        : integer range 0 to C_BUTTERFLIES_PER_STAGE := 0;

  signal Frame_Out_R        : t_complex_array(0 to G_FFT_SIZE - 1) := (others => ((others => '0'), (others => '0')));
  signal Frame_Out_V_R      : std_logic := '0';

  signal Butterfly_In_V_R    : t_logic_array(0 to C_BUTTERFLY_LANES - 1) := (others => '0');
  signal Butterfly_A_Re_R    : t_fft_word_array(0 to C_BUTTERFLY_LANES - 1) := (others => (others => '0'));
  signal Butterfly_A_Im_R    : t_fft_word_array(0 to C_BUTTERFLY_LANES - 1) := (others => (others => '0'));
  signal Butterfly_B_Re_R    : t_fft_word_array(0 to C_BUTTERFLY_LANES - 1) := (others => (others => '0'));
  signal Butterfly_B_Im_R    : t_fft_word_array(0 to C_BUTTERFLY_LANES - 1) := (others => (others => '0'));
  signal Butterfly_Tw_Re_R   : t_fft_word_array(0 to C_BUTTERFLY_LANES - 1) := (others => (others => '0'));
  signal Butterfly_Tw_Im_R   : t_fft_word_array(0 to C_BUTTERFLY_LANES - 1) := (others => (others => '0'));
  signal Butterfly_In_Index_A_R : t_index_array(0 to C_BUTTERFLY_LANES - 1) := (others => 0);
  signal Butterfly_In_Index_B_R : t_index_array(0 to C_BUTTERFLY_LANES - 1) := (others => 0);

  signal Butterfly_Sum_Re_R  : t_fft_word_array(0 to C_BUTTERFLY_LANES - 1) := (others => (others => '0'));
  signal Butterfly_Sum_Im_R  : t_fft_word_array(0 to C_BUTTERFLY_LANES - 1) := (others => (others => '0'));
  signal Butterfly_Diff_Re_R : t_fft_word_array(0 to C_BUTTERFLY_LANES - 1) := (others => (others => '0'));
  signal Butterfly_Diff_Im_R : t_fft_word_array(0 to C_BUTTERFLY_LANES - 1) := (others => (others => '0'));
  signal Butterfly_Out_Index_A_R : t_index_array(0 to C_BUTTERFLY_LANES - 1) := (others => 0);
  signal Butterfly_Out_Index_B_R : t_index_array(0 to C_BUTTERFLY_LANES - 1) := (others => 0);
  signal Butterfly_Out_V_R   : t_logic_array(0 to C_BUTTERFLY_LANES - 1) := (others => '0');

  function twiddle_value(index : integer) return t_complex_sample is
  begin
    return fft_twiddle_value(G_STAGE_INDEX, index, G_IS_FORWARD, G_TWIDDLE_WIDTH, C_TWIDDLE_FRACTION_BITS);
  end function;
begin
  assert G_STAGE_INDEX > 0 report "G_STAGE_INDEX must be greater than zero" severity failure;
  assert G_FFT_SIZE > 1 report "G_FFT_SIZE must be greater than 1" severity failure;
  assert C_BUTTERFLY_LANES > 0 report "C_BUTTERFLY_LANES must be greater than zero" severity failure;

  Frame_In_Ready <= not Active_R;
  Frame_Out <= Frame_Out_R;
  Frame_Out_V <= Frame_Out_V_R;

  gen_butterflies : for lane_idx in 0 to C_BUTTERFLY_LANES - 1 generate
  begin
    butterfly_inst : entity work.fft_butterfly
      generic map (
        G_DATA_WIDTH          => G_INTERNAL_WIDTH,
        G_TWIDDLE_SHIFT_RIGHT => C_TWIDDLE_FRACTION_BITS,
        G_MUL_PIPE_STAGES     => G_MUL_PIPE_STAGES,
        G_MAX_INDEX           => G_FFT_SIZE
      )
      port map (
        Clk         => Clk,
        Rst         => Rst,
        Ce          => '1',
        In_V        => Butterfly_In_V_R(lane_idx),
        A_Re        => Butterfly_A_Re_R(lane_idx),
        A_Im        => Butterfly_A_Im_R(lane_idx),
        B_Re        => Butterfly_B_Re_R(lane_idx),
        B_Im        => Butterfly_B_Im_R(lane_idx),
        Tw_Re       => Butterfly_Tw_Re_R(lane_idx),
        Tw_Im       => Butterfly_Tw_Im_R(lane_idx),
        In_Index_A  => Butterfly_In_Index_A_R(lane_idx),
        In_Index_B  => Butterfly_In_Index_B_R(lane_idx),
        Sum_Re      => Butterfly_Sum_Re_R(lane_idx),
        Sum_Im      => Butterfly_Sum_Im_R(lane_idx),
        Diff_Re     => Butterfly_Diff_Re_R(lane_idx),
        Diff_Im     => Butterfly_Diff_Im_R(lane_idx),
        Out_Index_A => Butterfly_Out_Index_A_R(lane_idx),
        Out_Index_B => Butterfly_Out_Index_B_R(lane_idx),
        Out_V       => Butterfly_Out_V_R(lane_idx)
      );
  end generate;

  process(Clk)
    variable working_frame_v      : t_complex_array(0 to G_FFT_SIZE - 1);
    variable completed_count_v    : integer range 0 to C_BUTTERFLIES_PER_STAGE;
    variable issue_count_v        : integer range 0 to C_BUTTERFLIES_PER_STAGE;
    variable stage_butterfly_idx_v: integer;
    variable stage_group_idx_v    : integer;
    variable stage_pair_idx_v     : integer;
    variable idx_a_v              : integer;
    variable idx_b_v              : integer;
    variable twiddle_v            : t_complex_sample;
  begin
    if rising_edge(Clk) then
      if Rst = '1' then
        Active_R <= '0';
        Working_Frame_R <= (others => ((others => '0'), (others => '0')));
        Issued_R <= 0;
        Completed_R <= 0;
        Frame_Out_R <= (others => ((others => '0'), (others => '0')));
        Frame_Out_V_R <= '0';
        Butterfly_In_V_R <= (others => '0');
      else
        working_frame_v := Working_Frame_R;
        completed_count_v := Completed_R;
        issue_count_v := 0;
        Butterfly_In_V_R <= (others => '0');

        if Frame_Out_V_R = '1' and Frame_Out_Ready = '1' then
          Frame_Out_V_R <= '0';
        end if;

        for lane_idx in 0 to C_BUTTERFLY_LANES - 1 loop
          if Butterfly_Out_V_R(lane_idx) = '1' then
            working_frame_v(Butterfly_Out_Index_A_R(lane_idx)).re := Butterfly_Sum_Re_R(lane_idx);
            working_frame_v(Butterfly_Out_Index_A_R(lane_idx)).im := Butterfly_Sum_Im_R(lane_idx);
            working_frame_v(Butterfly_Out_Index_B_R(lane_idx)).re := Butterfly_Diff_Re_R(lane_idx);
            working_frame_v(Butterfly_Out_Index_B_R(lane_idx)).im := Butterfly_Diff_Im_R(lane_idx);
            completed_count_v := completed_count_v + 1;
          end if;
        end loop;

        if Active_R = '0' then
          Working_Frame_R <= working_frame_v;
          Completed_R <= completed_count_v;
          Issued_R <= 0;
          if Frame_In_V = '1' then
            Working_Frame_R <= Frame_In;
            Active_R <= '1';
            Issued_R <= 0;
            Completed_R <= 0;
          end if;
        else
          if completed_count_v = C_BUTTERFLIES_PER_STAGE then
            assert not (Frame_Out_V_R = '1' and Frame_Out_Ready = '0')
              report "Stage output buffer overflow"
              severity failure;
            Frame_Out_R <= working_frame_v;
            Frame_Out_V_R <= '1';
            Working_Frame_R <= working_frame_v;
            Active_R <= '0';
            Issued_R <= 0;
            Completed_R <= 0;
          else
            for lane_idx in 0 to C_BUTTERFLY_LANES - 1 loop
              if (Issued_R + issue_count_v) < C_BUTTERFLIES_PER_STAGE then
                stage_butterfly_idx_v := Issued_R + issue_count_v;
                stage_group_idx_v := stage_butterfly_idx_v / C_HALF_STRIDE;
                stage_pair_idx_v := stage_butterfly_idx_v mod C_HALF_STRIDE;
                idx_a_v := stage_group_idx_v * C_STRIDE + stage_pair_idx_v;
                idx_b_v := idx_a_v + C_HALF_STRIDE;
                twiddle_v := twiddle_value(stage_pair_idx_v);

                Butterfly_A_Re_R(lane_idx) <= working_frame_v(idx_a_v).re;
                Butterfly_A_Im_R(lane_idx) <= working_frame_v(idx_a_v).im;
                Butterfly_B_Re_R(lane_idx) <= working_frame_v(idx_b_v).re;
                Butterfly_B_Im_R(lane_idx) <= working_frame_v(idx_b_v).im;
                Butterfly_Tw_Re_R(lane_idx) <= twiddle_v.re;
                Butterfly_Tw_Im_R(lane_idx) <= twiddle_v.im;
                Butterfly_In_Index_A_R(lane_idx) <= idx_a_v;
                Butterfly_In_Index_B_R(lane_idx) <= idx_b_v;
                Butterfly_In_V_R(lane_idx) <= '1';
                issue_count_v := issue_count_v + 1;
              end if;
            end loop;

            Working_Frame_R <= working_frame_v;
            Issued_R <= Issued_R + issue_count_v;
            Completed_R <= completed_count_v;
          end if;
        end if;
      end if;
    end if;
  end process;
end architecture rtl;
