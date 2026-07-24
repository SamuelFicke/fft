library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.fft_pkg.all;

entity complex_multiplier is
  generic (
    G_DATA_WIDTH        : integer := C_FFT_INTERNAL_WIDTH;
    G_OUTPUT_WIDTH      : integer := C_FFT_INTERNAL_WIDTH;
    G_SHIFT_RIGHT       : integer := 0;
    G_OUTPUT_PIPE_STAGES: integer := 1
  );
  port (
    Clk        : in  std_logic;
    Rst        : in  std_logic;
    Ce         : in  std_logic := '1';
    Data_V     : in  std_logic;
    A_Re       : in  signed(G_DATA_WIDTH - 1 downto 0);
    A_Im       : in  signed(G_DATA_WIDTH - 1 downto 0);
    B_Re       : in  signed(G_DATA_WIDTH - 1 downto 0);
    B_Im       : in  signed(G_DATA_WIDTH - 1 downto 0);
    Product_Re : out signed(G_OUTPUT_WIDTH - 1 downto 0);
    Product_Im : out signed(G_OUTPUT_WIDTH - 1 downto 0);
    Product_V  : out std_logic
  );
end entity complex_multiplier;

architecture rtl of complex_multiplier is
  subtype t_out_word is signed(G_OUTPUT_WIDTH - 1 downto 0);

  signal Mul_RR_W : t_out_word := (others => '0');
  signal Mul_II_W : t_out_word := (others => '0');
  signal Mul_RI_W : t_out_word := (others => '0');
  signal Mul_IR_W : t_out_word := (others => '0');

  signal Mul_RR_V_W : std_logic := '0';
  signal Mul_II_V_W : std_logic := '0';
  signal Mul_RI_V_W : std_logic := '0';
  signal Mul_IR_V_W : std_logic := '0';

  signal Product_Re_R : t_out_word := (others => '0');
  signal Product_Im_R : t_out_word := (others => '0');
  signal Product_V_R  : std_logic := '0';
begin
  mul_rr : entity work.signed_multiplier
    generic map (
      G_A_WIDTH            => G_DATA_WIDTH,
      G_B_WIDTH            => G_DATA_WIDTH,
      G_OUTPUT_WIDTH       => G_OUTPUT_WIDTH,
      G_OUTPUT_SHIFT_RIGHT => G_SHIFT_RIGHT,
      G_OUTPUT_PIPE_STAGES => G_OUTPUT_PIPE_STAGES,
      G_SATURATE_OUTPUT    => true
    )
    port map (
      Clk       => Clk,
      Rst       => Rst,
      Ce        => Ce,
      Data_A    => A_Re,
      Data_B    => B_Re,
      Data_V    => Data_V,
      Product   => Mul_RR_W,
      Product_V => Mul_RR_V_W
    );

  mul_ii : entity work.signed_multiplier
    generic map (
      G_A_WIDTH            => G_DATA_WIDTH,
      G_B_WIDTH            => G_DATA_WIDTH,
      G_OUTPUT_WIDTH       => G_OUTPUT_WIDTH,
      G_OUTPUT_SHIFT_RIGHT => G_SHIFT_RIGHT,
      G_OUTPUT_PIPE_STAGES => G_OUTPUT_PIPE_STAGES,
      G_SATURATE_OUTPUT    => true
    )
    port map (
      Clk       => Clk,
      Rst       => Rst,
      Ce        => Ce,
      Data_A    => A_Im,
      Data_B    => B_Im,
      Data_V    => Data_V,
      Product   => Mul_II_W,
      Product_V => Mul_II_V_W
    );

  mul_ri : entity work.signed_multiplier
    generic map (
      G_A_WIDTH            => G_DATA_WIDTH,
      G_B_WIDTH            => G_DATA_WIDTH,
      G_OUTPUT_WIDTH       => G_OUTPUT_WIDTH,
      G_OUTPUT_SHIFT_RIGHT => G_SHIFT_RIGHT,
      G_OUTPUT_PIPE_STAGES => G_OUTPUT_PIPE_STAGES,
      G_SATURATE_OUTPUT    => true
    )
    port map (
      Clk       => Clk,
      Rst       => Rst,
      Ce        => Ce,
      Data_A    => A_Re,
      Data_B    => B_Im,
      Data_V    => Data_V,
      Product   => Mul_RI_W,
      Product_V => Mul_RI_V_W
    );

  mul_ir : entity work.signed_multiplier
    generic map (
      G_A_WIDTH            => G_DATA_WIDTH,
      G_B_WIDTH            => G_DATA_WIDTH,
      G_OUTPUT_WIDTH       => G_OUTPUT_WIDTH,
      G_OUTPUT_SHIFT_RIGHT => G_SHIFT_RIGHT,
      G_OUTPUT_PIPE_STAGES => G_OUTPUT_PIPE_STAGES,
      G_SATURATE_OUTPUT    => true
    )
    port map (
      Clk       => Clk,
      Rst       => Rst,
      Ce        => Ce,
      Data_A    => A_Im,
      Data_B    => B_Re,
      Data_V    => Data_V,
      Product   => Mul_IR_W,
      Product_V => Mul_IR_V_W
    );

  process (Clk)
    variable re_wide : signed(G_OUTPUT_WIDTH downto 0);
    variable im_wide : signed(G_OUTPUT_WIDTH downto 0);
  begin
    if rising_edge(Clk) then
      if Rst = '1' then
        Product_Re_R <= (others => '0');
        Product_Im_R <= (others => '0');
        Product_V_R  <= '0';
      elsif Ce = '1' then
        Product_V_R <= '0';
        if Mul_RR_V_W = '1' and Mul_II_V_W = '1' and Mul_RI_V_W = '1' and Mul_IR_V_W = '1' then
          re_wide := resize(Mul_RR_W, re_wide'length) - resize(Mul_II_W, re_wide'length);
          im_wide := resize(Mul_RI_W, im_wide'length) + resize(Mul_IR_W, im_wide'length);
          Product_Re_R <= fft_resize_saturate(re_wide, G_OUTPUT_WIDTH);
          Product_Im_R <= fft_resize_saturate(im_wide, G_OUTPUT_WIDTH);
          Product_V_R  <= '1';
        end if;
      end if;
    end if;
  end process;

  Product_Re <= Product_Re_R;
  Product_Im <= Product_Im_R;
  Product_V  <= Product_V_R;
end architecture rtl;
