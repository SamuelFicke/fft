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
  constant C_INPUT_WORDS : integer := (G_FFT_SIZE + G_SAMPLES_PER_CLK - 1) / G_SAMPLES_PER_CLK;
  constant C_OUTPUT_WIDTH : integer := G_SAMPLE_WIDTH + C_LOG2_N;
  constant C_STAGE_BUTTERFLY_LANES : integer := min_int(G_FFT_SIZE / 2, max_int(1, C_STAGES * G_SAMPLES_PER_CLK));
  constant C_ZERO_SAMPLE : t_complex_sample := ((others => '0'), (others => '0'));
  type t_stage_frame_array is array (natural range <>) of t_complex_array(0 to G_FFT_SIZE - 1);
  type t_logic_array is array (natural range <>) of std_logic;

  signal Frame_A_Buffer_R    : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Frame_B_Buffer_R    : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Input_Word_Count_R  : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Output_Word_Count_R : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Frame_A_Ready_R     : std_logic := '0';
  signal Frame_B_Ready_R     : std_logic := '0';
  signal Capture_Buffer_R    : std_logic := '0';
  signal Stage0_Frame_In_R   : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Stage0_Frame_In_V_R : std_logic := '0';
  signal Stage_Frame_R       : t_stage_frame_array(0 to C_STAGES - 1) := (others => (others => C_ZERO_SAMPLE));
  signal Stage_Frame_V_R     : t_logic_array(0 to C_STAGES - 1) := (others => '0');
  signal Stage_In_Ready_R    : t_logic_array(0 to C_STAGES - 1) := (others => '0');
  signal Stage_Out_Ready_R   : t_logic_array(0 to C_STAGES - 1) := (others => '0');

  signal Output_Active_R     : std_logic := '0';
  signal Output_Frame_R      : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Pending_Output_Frame_R : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Pending_Output_Ready_R : std_logic := '0';
  signal Final_Stage_Ready_S : std_logic;

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

  Final_Stage_Ready_S <= '1' when (Output_Active_R = '0' or Pending_Output_Ready_R = '0') else '0';
  Stage_Out_Ready_R(C_STAGES - 1) <= Final_Stage_Ready_S;

  gen_interstage_ready : for stage_idx in 0 to C_STAGES - 2 generate
  begin
    Stage_Out_Ready_R(stage_idx) <= Stage_In_Ready_R(stage_idx + 1);
  end generate;

  gen_stage_0 : if C_STAGES > 0 generate
  begin
    stage_0_inst : entity work.fft_stage
      generic map (
        G_STAGE_INDEX     => 1,
        G_FFT_SIZE        => G_FFT_SIZE,
        G_SAMPLES_PER_CLK => G_SAMPLES_PER_CLK,
        G_BUTTERFLY_LANES => C_STAGE_BUTTERFLY_LANES,
        G_TWIDDLE_WIDTH   => G_TWIDDLE_WIDTH,
        G_IS_FORWARD      => G_IS_FORWARD,
        G_INTERNAL_WIDTH  => C_FFT_INTERNAL_WIDTH,
        G_MUL_PIPE_STAGES => 1
      )
      port map (
        Clk            => Clk,
        Rst            => Rst,
        Frame_In       => Stage0_Frame_In_R,
        Frame_In_V     => Stage0_Frame_In_V_R,
        Frame_In_Ready => Stage_In_Ready_R(0),
        Frame_Out      => Stage_Frame_R(0),
        Frame_Out_V    => Stage_Frame_V_R(0),
        Frame_Out_Ready => Stage_Out_Ready_R(0)
      );
  end generate;

  gen_stages : for stage_idx in 1 to C_STAGES - 1 generate
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
        Clk            => Clk,
        Rst            => Rst,
        Frame_In       => Stage_Frame_R(stage_idx - 1),
        Frame_In_V     => Stage_Frame_V_R(stage_idx - 1),
        Frame_In_Ready => Stage_In_Ready_R(stage_idx),
        Frame_Out      => Stage_Frame_R(stage_idx),
        Frame_Out_V    => Stage_Frame_V_R(stage_idx),
        Frame_Out_Ready => Stage_Out_Ready_R(stage_idx)
      );
  end generate;

  process(Clk)
    variable sample_index     : integer;
    variable sample_word_i    : std_logic_vector(G_DATA_WIDTH - 1 downto 0);
    variable sample_word_q    : std_logic_vector(G_DATA_WIDTH - 1 downto 0);
  begin
    if rising_edge(Clk) then
      if Rst = '1' then
        Input_Word_Count_R <= 0;
        Output_Word_Count_R <= 0;
        Data_Out_I_R <= (others => '0');
        Data_Out_Q_R <= (others => '0');
        Data_Out_V_R <= '0';
        Frame_A_Buffer_R <= (others => C_ZERO_SAMPLE);
        Frame_B_Buffer_R <= (others => C_ZERO_SAMPLE);
        Frame_A_Ready_R <= '0';
        Frame_B_Ready_R <= '0';
        Capture_Buffer_R <= '0';
        Stage0_Frame_In_R <= (others => C_ZERO_SAMPLE);
        Stage0_Frame_In_V_R <= '0';
        Output_Active_R <= '0';
        Output_Frame_R <= (others => C_ZERO_SAMPLE);
        Pending_Output_Frame_R <= (others => C_ZERO_SAMPLE);
        Pending_Output_Ready_R <= '0';
      else
        Stage0_Frame_In_V_R <= '0';
        Data_Out_V_R <= '0';

        if Sample_In_V = '1' then
          for lane_idx in 0 to G_SAMPLES_PER_CLK - 1 loop
            sample_index := Input_Word_Count_R * G_SAMPLES_PER_CLK + lane_idx;
            if sample_index < G_FFT_SIZE then
              sample_word_i := Sample_In_I(((lane_idx + 1) * G_DATA_WIDTH) - 1 downto lane_idx * G_DATA_WIDTH);
              sample_word_q := Sample_In_Q(((lane_idx + 1) * G_DATA_WIDTH) - 1 downto lane_idx * G_DATA_WIDTH);
              if Capture_Buffer_R = '0' then
                Frame_A_Buffer_R(sample_index).re <= resize(signed(sample_word_i), C_FFT_INTERNAL_WIDTH);
                Frame_A_Buffer_R(sample_index).im <= resize(signed(sample_word_q), C_FFT_INTERNAL_WIDTH);
              else
                Frame_B_Buffer_R(sample_index).re <= resize(signed(sample_word_i), C_FFT_INTERNAL_WIDTH);
                Frame_B_Buffer_R(sample_index).im <= resize(signed(sample_word_q), C_FFT_INTERNAL_WIDTH);
              end if;
            end if;
          end loop;

          if Input_Word_Count_R = C_INPUT_WORDS - 1 then
            if Capture_Buffer_R = '0' then
              assert Frame_A_Ready_R = '0' report "Input frame overflow on capture buffer A" severity failure;
              Frame_A_Ready_R <= '1';
            else
              assert Frame_B_Ready_R = '0' report "Input frame overflow on capture buffer B" severity failure;
              Frame_B_Ready_R <= '1';
            end if;
            Input_Word_Count_R <= 0;
            if Capture_Buffer_R = '0' then
              Capture_Buffer_R <= '1';
            else
              Capture_Buffer_R <= '0';
            end if;
          else
            Input_Word_Count_R <= Input_Word_Count_R + 1;
          end if;
        end if;

        if Stage_In_Ready_R(0) = '1' then
          if Frame_A_Ready_R = '1' then
            Frame_A_Ready_R <= '0';
            Stage0_Frame_In_R <= fft_bit_reverse_permute(Frame_A_Buffer_R);
            Stage0_Frame_In_V_R <= '1';
          elsif Frame_B_Ready_R = '1' then
            Frame_B_Ready_R <= '0';
            Stage0_Frame_In_R <= fft_bit_reverse_permute(Frame_B_Buffer_R);
            Stage0_Frame_In_V_R <= '1';
          end if;
        end if;

        if Stage_Frame_V_R(C_STAGES - 1) = '1' and Final_Stage_Ready_S = '1' then
          if Output_Active_R = '0' then
            Output_Frame_R <= Stage_Frame_R(C_STAGES - 1);
            Output_Active_R <= '1';
            Output_Word_Count_R <= 0;
          else
            Pending_Output_Frame_R <= Stage_Frame_R(C_STAGES - 1);
            Pending_Output_Ready_R <= '1';
          end if;
        end if;

        if Output_Active_R = '1' then
          Data_Out_V_R <= '1';
          for lane_idx in 0 to G_SAMPLES_PER_CLK - 1 loop
            sample_index := Output_Word_Count_R * G_SAMPLES_PER_CLK + lane_idx;
            if sample_index < G_FFT_SIZE then
              Data_Out_I_R(((lane_idx + 1) * C_OUTPUT_WIDTH) - 1 downto lane_idx * C_OUTPUT_WIDTH) <= std_logic_vector(fft_resize_saturate(Output_Frame_R(sample_index).re, C_OUTPUT_WIDTH));
              Data_Out_Q_R(((lane_idx + 1) * C_OUTPUT_WIDTH) - 1 downto lane_idx * C_OUTPUT_WIDTH) <= std_logic_vector(fft_resize_saturate(Output_Frame_R(sample_index).im, C_OUTPUT_WIDTH));
            else
              Data_Out_I_R(((lane_idx + 1) * C_OUTPUT_WIDTH) - 1 downto lane_idx * C_OUTPUT_WIDTH) <= (others => '0');
              Data_Out_Q_R(((lane_idx + 1) * C_OUTPUT_WIDTH) - 1 downto lane_idx * C_OUTPUT_WIDTH) <= (others => '0');
            end if;
          end loop;

          if Output_Word_Count_R = C_INPUT_WORDS - 1 then
            if Pending_Output_Ready_R = '1' then
              Output_Frame_R <= Pending_Output_Frame_R;
              Pending_Output_Ready_R <= '0';
              Output_Word_Count_R <= 0;
              Output_Active_R <= '1';
            else
              Output_Word_Count_R <= 0;
              Output_Active_R <= '0';
            end if;
          else
            Output_Word_Count_R <= Output_Word_Count_R + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  Data_Out_I <= Data_Out_I_R;
  Data_Out_Q <= Data_Out_Q_R;
  Data_Out_V <= Data_Out_V_R;
end architecture rtl;
