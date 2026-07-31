library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

library work;
  use work.fft_pkg.all;
  use work.fft_twiddle.all;

entity fft_mdf is
  generic (
    G_SAMPLE_WIDTH    : integer := 16;
    G_TWIDDLE_WIDTH   : integer := 16;
    G_FFT_SIZE        : integer := 8;
    G_SAMPLES_PER_CLK : integer := 1;
    G_DATA_WIDTH      : integer := 16;
    G_IS_FORWARD      : boolean := true
  );
  port (
    Clk         : in  std_logic;
    Rst         : in  std_logic;

    Sample_In_I : in  std_logic_vector(G_DATA_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0);
    Sample_In_Q : in  std_logic_vector(G_DATA_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0);
    Sample_In_V : in  std_logic;

    Data_Out_I  : out std_logic_vector(((G_SAMPLE_WIDTH + integer(log2(real(G_FFT_SIZE)))) * G_SAMPLES_PER_CLK) - 1 downto 0);
    Data_Out_Q  : out std_logic_vector(((G_SAMPLE_WIDTH + integer(log2(real(G_FFT_SIZE)))) * G_SAMPLES_PER_CLK) - 1 downto 0);
    Data_Out_V  : out std_logic
  );
end entity fft_mdf;

architecture rtl of fft_mdf is
  function min_int(lhs : integer; rhs : integer) return integer is
  begin
    if lhs < rhs then
      return lhs;
    end if;
    return rhs;
  end function;

  function max_int(lhs : integer; rhs : integer) return integer is
  begin
    if lhs > rhs then
      return lhs;
    end if;
    return rhs;
  end function;

  constant C_LOG2_N      : integer := integer(log2(real(G_FFT_SIZE)));
  constant C_STAGES      : integer := C_LOG2_N;
  constant C_OUTPUT_WIDTH : integer := G_SAMPLE_WIDTH + C_LOG2_N;
  constant C_STAGE_BUTTERFLY_LANES : integer := min_int(G_FFT_SIZE / 2, max_int(1, C_STAGES * G_SAMPLES_PER_CLK));
  constant C_ZERO_SAMPLE : t_complex_sample := ((others => '0'), (others => '0'));
  type t_logic_array is array (natural range <>) of std_logic;

  signal Input_Lane_Index_R      : integer range 0 to G_SAMPLES_PER_CLK - 1 := 0;
  signal Input_Word_Buffer_Valid_R : std_logic := '0';

  signal Stage_Sample           : t_complex_array(0 to C_STAGES) := (others => C_ZERO_SAMPLE);
  signal Stage_Valid         : t_logic_array(0 to C_STAGES) := (others => '0');
  signal Stage_Ready         : t_logic_array(0 to C_STAGES) := (others => '0');

  signal Output_Lane_Count_R : integer range 0 to G_SAMPLES_PER_CLK - 1 := 0;
  signal Data_Out_I_R        : std_logic_vector(C_OUTPUT_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0) := (others => '0');
  signal Data_Out_Q_R        : std_logic_vector(C_OUTPUT_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0) := (others => '0');
  signal Data_Out_V_R        : std_logic := '0';

begin
  assert G_FFT_SIZE > 0 report "G_FFT_SIZE must be greater than zero" severity failure;
  assert G_FFT_SIZE >= 2 report "G_FFT_SIZE must be at least 2" severity failure;
  assert G_FFT_SIZE <= 65536 report "G_FFT_SIZE must be less than or equal to 65536" severity failure;
  assert G_SAMPLES_PER_CLK > 0 report "G_SAMPLES_PER_CLK must be greater than zero" severity failure;
  assert G_SAMPLES_PER_CLK <= 32 report "G_SAMPLES_PER_CLK must be less than or equal to 32" severity failure;
  assert fft_is_power_of_two(G_FFT_SIZE) report "G_FFT_SIZE must be a power of two" severity failure;
  assert fft_is_power_of_two(G_SAMPLES_PER_CLK) report "G_SAMPLES_PER_CLK must be a power of two" severity failure;
  assert G_DATA_WIDTH > 0 report "G_DATA_WIDTH must be greater than zero" severity failure;
  assert C_STAGE_BUTTERFLY_LANES > 0 report "Calculated stage butterfly lane count must be greater than zero" severity failure;
  assert C_OUTPUT_WIDTH > 0 report "Calculated FFT output width must be greater than zero" severity failure;
  assert G_SAMPLE_WIDTH > 0 report "G_SAMPLE_WIDTH must be greater than zero" severity failure;
  assert G_TWIDDLE_WIDTH > 0 report "G_TWIDDLE_WIDTH must be greater than zero" severity failure;
  assert G_TWIDDLE_WIDTH >= 2 report "G_TWIDDLE_WIDTH must be at least 2 bits" severity failure;

  gen_stages : for stage_idx in 0 to C_STAGES - 1 generate
  begin
    stage_inst : entity work.fft_stage
      generic map (
        G_STAGE_INDEX     => stage_idx + 1,
        G_FFT_SIZE        => G_FFT_SIZE,
        G_SAMPLES_PER_CLK => G_SAMPLES_PER_CLK,
        G_BUTTERFLY_LANES => C_STAGE_BUTTERFLY_LANES,
        G_TWIDDLE_WIDTH   => G_TWIDDLE_WIDTH,
        G_IS_FORWARD      => G_IS_FORWARD,
        G_INTERNAL_WIDTH  => C_FFT_INTERNAL_WIDTH,
        G_MUL_PIPE_STAGES => 1
      )
      port map (
        Clk              => Clk,
        Rst              => Rst,
        Sample_In        => Stage_Sample(stage_idx),
        Sample_In_V      => Stage_Valid(stage_idx),
        Sample_In_Ready  => Stage_Ready(stage_idx),
        Sample_Out       => Stage_Sample(stage_idx + 1),
        Sample_Out_V     => Stage_Valid(stage_idx + 1),
        Sample_Out_Ready => Stage_Ready(stage_idx + 1)
      );
  end generate;

  Stage_Ready(C_STAGES) <= '1';

  input_proc : process(Clk)
    variable sample_word_i : std_logic_vector(G_DATA_WIDTH - 1 downto 0);
    variable sample_word_q : std_logic_vector(G_DATA_WIDTH - 1 downto 0);
  begin
    if rising_edge(Clk) then
      if Rst = '1' then
        Input_Lane_Index_R <= 0;
        Stage_Valid(0) <= '0';
        Stage_Sample(0) <= C_ZERO_SAMPLE;
      else
        Stage_Valid(0) <= '0';

        if Sample_In_V = '1' and Stage_Ready(0) = '1' then
          sample_word_i := Sample_In_I(((Input_Lane_Index_R + 1) * G_DATA_WIDTH) - 1 downto Input_Lane_Index_R * G_DATA_WIDTH);
          sample_word_q := Sample_In_Q(((Input_Lane_Index_R + 1) * G_DATA_WIDTH) - 1 downto Input_Lane_Index_R * G_DATA_WIDTH);

          Stage_Sample(0).re <= resize(signed(sample_word_i), C_FFT_INTERNAL_WIDTH);
          Stage_Sample(0).im <= resize(signed(sample_word_q), C_FFT_INTERNAL_WIDTH);
          Stage_Valid(0) <= '1';

          if Input_Lane_Index_R = G_SAMPLES_PER_CLK - 1 then
            Input_Lane_Index_R <= 0;
          else
            Input_Lane_Index_R <= Input_Lane_Index_R + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  output_proc : process(Clk)
  begin
    if rising_edge(Clk) then
      if Rst = '1' then
        Output_Lane_Count_R <= 0;
        Data_Out_I_R <= (others => '0');
        Data_Out_Q_R <= (others => '0');
        Data_Out_V_R <= '0';
      else
        Data_Out_V_R <= '0';

        if Stage_Valid(C_STAGES) = '1' then
          Data_Out_I_R(((Output_Lane_Count_R + 1) * C_OUTPUT_WIDTH) - 1 downto Output_Lane_Count_R * C_OUTPUT_WIDTH) <=
            std_logic_vector(fft_resize_saturate(Stage_Sample(C_STAGES).re, C_OUTPUT_WIDTH));
          Data_Out_Q_R(((Output_Lane_Count_R + 1) * C_OUTPUT_WIDTH) - 1 downto Output_Lane_Count_R * C_OUTPUT_WIDTH) <=
            std_logic_vector(fft_resize_saturate(Stage_Sample(C_STAGES).im, C_OUTPUT_WIDTH));

          if Output_Lane_Count_R = G_SAMPLES_PER_CLK - 1 then
            Output_Lane_Count_R <= 0;
            Data_Out_V_R <= '1';
          else
            Output_Lane_Count_R <= Output_Lane_Count_R + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  Data_Out_I <= Data_Out_I_R;
  Data_Out_Q <= Data_Out_Q_R;
  Data_Out_V <= Data_Out_V_R;
end architecture rtl;
