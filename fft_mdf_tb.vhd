library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

entity fft_mdf_tb is
end entity fft_mdf_tb;

architecture sim of fft_mdf_tb is
  constant C_SAMPLE_WIDTH : integer := 8;
  constant C_TWIDDLE_WIDTH : integer := 8;
  constant C_FFT_SIZE : integer := 8;
  constant C_SAMPLES_PER_CLK : integer := 1;

  signal Clk : std_logic := '0';
  signal Rst : std_logic := '1';
  signal Sample_In_I : std_logic_vector(C_SAMPLE_WIDTH * C_SAMPLES_PER_CLK - 1 downto 0);
  signal Sample_In_Q : std_logic_vector(C_SAMPLE_WIDTH * C_SAMPLES_PER_CLK - 1 downto 0);
  signal Sample_In_V : std_logic := '0';
  signal Data_Out_I : std_logic_vector(C_SAMPLE_WIDTH * C_SAMPLES_PER_CLK - 1 downto 0);
  signal Data_Out_Q : std_logic_vector(C_SAMPLE_WIDTH * C_SAMPLES_PER_CLK - 1 downto 0);
  signal Data_Out_V : std_logic;

  function to_integer_vector(value : std_logic_vector) return integer is
  begin
    return to_integer(signed(value));
  end function;

begin
  uut : entity work.polyphase_decimator
    generic map (
      G_SAMPLE_WIDTH => C_SAMPLE_WIDTH,
      G_TWIDDLE_WIDTH => C_TWIDDLE_WIDTH,
      G_FFT_SIZE => C_FFT_SIZE,
      G_SAMPLES_PER_CLK => C_SAMPLES_PER_CLK,
      G_DATA_WIDTH => C_SAMPLE_WIDTH,
      G_DATA_OUT_WIDTH => C_SAMPLE_WIDTH
    )
    port map (
      Clk => Clk,
      Rst => Rst,
      Sample_In_I => Sample_In_I,
      Sample_In_Q => Sample_In_Q,
      Sample_In_V => Sample_In_V,
      Data_Out_I => Data_Out_I,
      Data_Out_Q => Data_Out_Q,
      Data_Out_V => Data_Out_V
    );

  clk_gen : process
  begin
    loop
      Clk <= '0';
      wait for 5 ns;
      Clk <= '1';
      wait for 5 ns;
    end loop;
  end process;

  stim : process
  begin
    Rst <= '1';
    Sample_In_I <= (others => '0');
    Sample_In_Q <= (others => '0');
    Sample_In_V <= '0';
    wait for 20 ns;
    Rst <= '0';

    for i in 0 to C_FFT_SIZE - 1 loop
      if i = 0 then
        Sample_In_I <= std_logic_vector(to_signed(1, C_SAMPLE_WIDTH));
      else
        Sample_In_I <= (others => '0');
      end if;
      Sample_In_Q <= (others => '0');
      Sample_In_V <= '1';
      wait until rising_edge(Clk);
    end loop;

    Sample_In_V <= '0';
    wait until Data_Out_V = '1';
    wait for 20 ns;

    assert to_integer_vector(Data_Out_I) = 1 report "First FFT output should be 1" severity failure;
    assert to_integer_vector(Data_Out_Q) = 0 report "First FFT output imag should be 0" severity failure;

    report "FFT test passed";
    wait;
  end process;
end architecture sim;
