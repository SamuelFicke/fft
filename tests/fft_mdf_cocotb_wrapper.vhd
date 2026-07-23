library ieee;
  use ieee.std_logic_1164.all;

entity fft_mdf_cocotb_wrapper is
  generic (
    G_SAMPLE_WIDTH    : integer := 16;
    G_TWIDDLE_WIDTH   : integer := 16;
    G_FFT_SIZE        : integer := 1024;
    G_SAMPLES_PER_CLK : integer := 1;
    G_DATA_WIDTH      : integer := 16;
    G_DATA_OUT_WIDTH  : integer := 16
  );
  port (
    Clk : in std_logic;
    Rst : in std_logic;

    Sample_In_I : in std_logic_vector(G_DATA_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0);
    Sample_In_Q : in std_logic_vector(G_DATA_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0);
    Sample_In_V : in std_logic;
    Data_Out_I  : out std_logic_vector(G_DATA_OUT_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0);
    Data_Out_Q  : out std_logic_vector(G_DATA_OUT_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0);
    Data_Out_V  : out std_logic
  );
end entity fft_mdf_cocotb_wrapper;

architecture rtl of fft_mdf_cocotb_wrapper is
begin
  uut : entity work.polyphase_decimator
    generic map (
      G_SAMPLE_WIDTH    => G_SAMPLE_WIDTH,
      G_TWIDDLE_WIDTH   => G_TWIDDLE_WIDTH,
      G_FFT_SIZE        => G_FFT_SIZE,
      G_SAMPLES_PER_CLK => G_SAMPLES_PER_CLK,
      G_DATA_WIDTH      => G_DATA_WIDTH,
      G_DATA_OUT_WIDTH  => G_DATA_OUT_WIDTH,
      G_IS_FORWARD      => true
    )
    port map (
      Clk         => Clk,
      Rst         => Rst,
      Sample_In_I => Sample_In_I,
      Sample_In_Q => Sample_In_Q,
      Sample_In_V => Sample_In_V,
      Data_Out_I  => Data_Out_I,
      Data_Out_Q  => Data_Out_Q,
      Data_Out_V  => Data_Out_V
    );
end architecture rtl;
