library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.fft_pkg.all;

entity fft_butterfly is
  generic (
    G_DATA_WIDTH         : integer := C_FFT_INTERNAL_WIDTH;
    G_TWIDDLE_SHIFT_RIGHT: integer := 0;
    G_MUL_PIPE_STAGES    : integer := 1;
    G_MAX_INDEX          : integer := 1024
  );
  port (
    Clk   : in  std_logic;
    Rst   : in  std_logic;
    Ce    : in  std_logic := '1';
    In_V  : in  std_logic;

    A_Re  : in  signed(G_DATA_WIDTH - 1 downto 0);
    A_Im  : in  signed(G_DATA_WIDTH - 1 downto 0);
    B_Re  : in  signed(G_DATA_WIDTH - 1 downto 0);
    B_Im  : in  signed(G_DATA_WIDTH - 1 downto 0);
    Tw_Re : in  signed(G_DATA_WIDTH - 1 downto 0);
    Tw_Im : in  signed(G_DATA_WIDTH - 1 downto 0);
    In_Index_A : in integer range 0 to G_MAX_INDEX - 1;
    In_Index_B : in integer range 0 to G_MAX_INDEX - 1;

    Sum_Re  : out signed(G_DATA_WIDTH - 1 downto 0);
    Sum_Im  : out signed(G_DATA_WIDTH - 1 downto 0);
    Diff_Re : out signed(G_DATA_WIDTH - 1 downto 0);
    Diff_Im : out signed(G_DATA_WIDTH - 1 downto 0);
    Out_Index_A : out integer range 0 to G_MAX_INDEX - 1;
    Out_Index_B : out integer range 0 to G_MAX_INDEX - 1;
    Out_V   : out std_logic
  );
end entity fft_butterfly;

architecture rtl of fft_butterfly is
  constant C_BUTTERFLY_LATENCY : integer := G_MUL_PIPE_STAGES + 4;

  type t_word_pipe is array (natural range <>) of signed(G_DATA_WIDTH - 1 downto 0);
  type t_index_pipe is array (natural range <>) of integer range 0 to G_MAX_INDEX - 1;

  signal A_Re_Pipe : t_word_pipe(0 to C_BUTTERFLY_LATENCY - 1) := (others => (others => '0'));
  signal A_Im_Pipe : t_word_pipe(0 to C_BUTTERFLY_LATENCY - 1) := (others => (others => '0'));
  signal Idx_A_Pipe : t_index_pipe(0 to C_BUTTERFLY_LATENCY - 1) := (others => 0);
  signal Idx_B_Pipe : t_index_pipe(0 to C_BUTTERFLY_LATENCY - 1) := (others => 0);

  signal Mul_Re : signed(G_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal Mul_Im : signed(G_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal Mul_V  : std_logic := '0';

  signal Sum_Re_R  : signed(G_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal Sum_Im_R  : signed(G_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal Diff_Re_R : signed(G_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal Diff_Im_R : signed(G_DATA_WIDTH - 1 downto 0) := (others => '0');
  signal Out_Index_A_R : integer range 0 to G_MAX_INDEX - 1 := 0;
  signal Out_Index_B_R : integer range 0 to G_MAX_INDEX - 1 := 0;
  signal Out_V_R   : std_logic := '0';
begin
  b_twiddle_mul : entity work.complex_multiplier
    generic map (
      G_DATA_WIDTH         => G_DATA_WIDTH,
      G_OUTPUT_WIDTH       => G_DATA_WIDTH,
      G_SHIFT_RIGHT        => G_TWIDDLE_SHIFT_RIGHT,
      G_OUTPUT_PIPE_STAGES => G_MUL_PIPE_STAGES
    )
    port map (
      Clk        => Clk,
      Rst        => Rst,
      Ce         => Ce,
      Data_V     => In_V,
      A_Re       => B_Re,
      A_Im       => B_Im,
      B_Re       => Tw_Re,
      B_Im       => Tw_Im,
      Product_Re => Mul_Re,
      Product_Im => Mul_Im,
      Product_V  => Mul_V
    );

  process (Clk)
    variable sum_re_wide  : signed(G_DATA_WIDTH downto 0);
    variable sum_im_wide  : signed(G_DATA_WIDTH downto 0);
    variable diff_re_wide : signed(G_DATA_WIDTH downto 0);
    variable diff_im_wide : signed(G_DATA_WIDTH downto 0);
  begin
    if rising_edge(Clk) then
      if Rst = '1' then
        A_Re_Pipe <= (others => (others => '0'));
        A_Im_Pipe <= (others => (others => '0'));
        Idx_A_Pipe <= (others => 0);
        Idx_B_Pipe <= (others => 0);
        Sum_Re_R <= (others => '0');
        Sum_Im_R <= (others => '0');
        Diff_Re_R <= (others => '0');
        Diff_Im_R <= (others => '0');
        Out_Index_A_R <= 0;
        Out_Index_B_R <= 0;
        Out_V_R <= '0';
      elsif Ce = '1' then
        Out_V_R <= '0';

        A_Re_Pipe(0) <= A_Re;
        A_Im_Pipe(0) <= A_Im;
        Idx_A_Pipe(0) <= In_Index_A;
        Idx_B_Pipe(0) <= In_Index_B;

        for idx in 1 to C_BUTTERFLY_LATENCY - 1 loop
          A_Re_Pipe(idx) <= A_Re_Pipe(idx - 1);
          A_Im_Pipe(idx) <= A_Im_Pipe(idx - 1);
          Idx_A_Pipe(idx) <= Idx_A_Pipe(idx - 1);
          Idx_B_Pipe(idx) <= Idx_B_Pipe(idx - 1);
        end loop;

        if Mul_V = '1' then
          sum_re_wide  := resize(A_Re_Pipe(C_BUTTERFLY_LATENCY - 1), sum_re_wide'length) + resize(Mul_Re, sum_re_wide'length);
          sum_im_wide  := resize(A_Im_Pipe(C_BUTTERFLY_LATENCY - 1), sum_im_wide'length) + resize(Mul_Im, sum_im_wide'length);
          diff_re_wide := resize(A_Re_Pipe(C_BUTTERFLY_LATENCY - 1), diff_re_wide'length) - resize(Mul_Re, diff_re_wide'length);
          diff_im_wide := resize(A_Im_Pipe(C_BUTTERFLY_LATENCY - 1), diff_im_wide'length) - resize(Mul_Im, diff_im_wide'length);

          Sum_Re_R  <= fft_resize_saturate(sum_re_wide, G_DATA_WIDTH);
          Sum_Im_R  <= fft_resize_saturate(sum_im_wide, G_DATA_WIDTH);
          Diff_Re_R <= fft_resize_saturate(diff_re_wide, G_DATA_WIDTH);
          Diff_Im_R <= fft_resize_saturate(diff_im_wide, G_DATA_WIDTH);
          Out_Index_A_R <= Idx_A_Pipe(C_BUTTERFLY_LATENCY - 1);
          Out_Index_B_R <= Idx_B_Pipe(C_BUTTERFLY_LATENCY - 1);
          Out_V_R   <= '1';
        end if;
      end if;
    end if;
  end process;

  Sum_Re  <= Sum_Re_R;
  Sum_Im  <= Sum_Im_R;
  Diff_Re <= Diff_Re_R;
  Diff_Im <= Diff_Im_R;
  Out_Index_A <= Out_Index_A_R;
  Out_Index_B <= Out_Index_B_R;
  Out_V   <= Out_V_R;
end architecture rtl;
