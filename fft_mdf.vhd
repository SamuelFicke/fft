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
  constant C_OUTPUT_WORDS : integer := (G_FFT_SIZE + G_SAMPLES_PER_CLK - 1) / G_SAMPLES_PER_CLK;
  constant C_STAGE_BUTTERFLY_LANES : integer := min_int(G_FFT_SIZE / 2, max_int(1, C_STAGES * G_SAMPLES_PER_CLK));
  constant C_ZERO_SAMPLE : t_complex_sample := ((others => '0'), (others => '0'));
  type t_pipeline_state is (S_IDLE, S_STREAM_INPUT, S_STREAM_OUTPUT, S_EMIT_OUTPUT);
  type t_logic_array is array (natural range <>) of std_logic;
  type t_stage_frame_array is array (natural range <>) of t_complex_array(0 to G_FFT_SIZE - 1);

  signal Frame_Buffer_R         : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Input_Word_Count_R     : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Frame_Ready_R          : std_logic := '0';

  signal Stage_Input_Frame_R    : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Stage_Input_Count_R    : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Stage_Output_Count_R   : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Stage_Frame_R          : t_stage_frame_array(0 to C_STAGES - 1) := (others => (others => C_ZERO_SAMPLE));
  signal Output_Frame_R         : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Pipeline_State_R       : t_pipeline_state := S_IDLE;
  signal Current_Stage_R        : integer range 0 to C_STAGES - 1 := 0;

  signal Stage_Input_Sample_R   : t_complex_array(0 to C_STAGES - 1) := (others => C_ZERO_SAMPLE);
  signal Stage_Input_Valid_R    : t_logic_array(0 to C_STAGES - 1) := (others => '0');
  signal Stage_Input_Ready_S    : t_logic_array(0 to C_STAGES - 1) := (others => '0');
  signal Stage_Output_Sample_R  : t_complex_array(0 to C_STAGES - 1) := (others => C_ZERO_SAMPLE);
  signal Stage_Output_Valid_R   : t_logic_array(0 to C_STAGES - 1) := (others => '0');
  signal Stage_Output_Ready_R   : t_logic_array(0 to C_STAGES - 1) := (others => '0');

  signal Output_Sample_Count_R  : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Output_Lane_Count_R    : integer range 0 to G_SAMPLES_PER_CLK - 1 := 0;
  signal Data_Out_I_R           : std_logic_vector(C_OUTPUT_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0) := (others => '0');
  signal Data_Out_Q_R           : std_logic_vector(C_OUTPUT_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0) := (others => '0');
  signal Data_Out_V_R           : std_logic := '0';

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
        Clk             => Clk,
        Rst             => Rst,
        Sample_In       => Stage_Input_Sample_R(stage_idx),
        Sample_In_V     => Stage_Input_Valid_R(stage_idx),
        Sample_In_Ready => Stage_Input_Ready_S(stage_idx),
        Sample_Out      => Stage_Output_Sample_R(stage_idx),
        Sample_Out_V    => Stage_Output_Valid_R(stage_idx),
        Sample_Out_Ready => Stage_Output_Ready_R(stage_idx)
      );
  end generate;

  process(Clk)
    variable sample_index         : integer;
    variable sample_word_i        : std_logic_vector(G_DATA_WIDTH - 1 downto 0);
    variable sample_word_q        : std_logic_vector(G_DATA_WIDTH - 1 downto 0);
    variable lane_idx             : integer;
  begin
    if rising_edge(Clk) then
      if Rst = '1' then
        Input_Word_Count_R <= 0;
        Frame_Ready_R <= '0';
        Stage_Input_Count_R <= 0;
        Stage_Output_Count_R <= 0;
        Stage_Input_Frame_R <= (others => C_ZERO_SAMPLE);
        Stage_Frame_R <= (others => (others => C_ZERO_SAMPLE));
        Output_Frame_R <= (others => C_ZERO_SAMPLE);
        Pipeline_State_R <= S_IDLE;
        Current_Stage_R <= 0;
        Stage_Input_Valid_R <= (others => '0');
        Stage_Input_Sample_R <= (others => C_ZERO_SAMPLE);
        Output_Sample_Count_R <= 0;
        Output_Lane_Count_R <= 0;
        Data_Out_I_R <= (others => '0');
        Data_Out_Q_R <= (others => '0');
        Data_Out_V_R <= '0';
      else
        Stage_Input_Valid_R <= (others => '0');
        Data_Out_V_R <= '0';
        Stage_Output_Ready_R <= (others => '0');

        if Sample_In_V = '1' and Frame_Ready_R = '0' and Pipeline_State_R = S_IDLE then
          for lane_idx in 0 to G_SAMPLES_PER_CLK - 1 loop
            sample_index := Input_Word_Count_R * G_SAMPLES_PER_CLK + lane_idx;
            if sample_index < G_FFT_SIZE then
              sample_word_i := Sample_In_I(((lane_idx + 1) * G_DATA_WIDTH) - 1 downto lane_idx * G_DATA_WIDTH);
              sample_word_q := Sample_In_Q(((lane_idx + 1) * G_DATA_WIDTH) - 1 downto lane_idx * G_DATA_WIDTH);
              Frame_Buffer_R(sample_index).re <= resize(signed(sample_word_i), C_FFT_INTERNAL_WIDTH);
              Frame_Buffer_R(sample_index).im <= resize(signed(sample_word_q), C_FFT_INTERNAL_WIDTH);
            end if;
          end loop;

          if Input_Word_Count_R = C_INPUT_WORDS - 1 then
            Frame_Ready_R <= '1';
            Input_Word_Count_R <= 0;
          else
            Input_Word_Count_R <= Input_Word_Count_R + 1;
          end if;
        end if;

        if Pipeline_State_R = S_IDLE and Frame_Ready_R = '1' then
          Stage_Input_Frame_R <= fft_bit_reverse_permute(Frame_Buffer_R);
          Stage_Input_Count_R <= 0;
          Stage_Output_Count_R <= 0;
          Current_Stage_R <= 0;
          Pipeline_State_R <= S_STREAM_INPUT;
          Frame_Ready_R <= '0';
        end if;

        if Pipeline_State_R = S_STREAM_INPUT then
          if Stage_Input_Count_R < G_FFT_SIZE and Stage_Input_Ready_S(Current_Stage_R) = '1' then
            Stage_Input_Sample_R(Current_Stage_R) <= Stage_Input_Frame_R(Stage_Input_Count_R);
            Stage_Input_Valid_R(Current_Stage_R) <= '1';
            if Stage_Input_Count_R = G_FFT_SIZE - 1 then
              Stage_Input_Count_R <= 0;
              Pipeline_State_R <= S_STREAM_OUTPUT;
              Stage_Output_Count_R <= 0;
            else
              Stage_Input_Count_R <= Stage_Input_Count_R + 1;
            end if;
          end if;
        end if;

        if Pipeline_State_R = S_STREAM_OUTPUT then
          Stage_Output_Ready_R(Current_Stage_R) <= '1';
          if Stage_Output_Valid_R(Current_Stage_R) = '1' then
            Stage_Frame_R(Current_Stage_R)(Stage_Output_Count_R) <= Stage_Output_Sample_R(Current_Stage_R);
            if Stage_Output_Count_R = G_FFT_SIZE - 1 then
              if Current_Stage_R = C_STAGES - 1 then
                Output_Frame_R <= Stage_Frame_R(Current_Stage_R);
                Output_Sample_Count_R <= 0;
                Output_Lane_Count_R <= 0;
                Pipeline_State_R <= S_EMIT_OUTPUT;
              else
                Stage_Input_Frame_R <= Stage_Frame_R(Current_Stage_R);
                Stage_Input_Count_R <= 0;
                Current_Stage_R <= Current_Stage_R + 1;
                Pipeline_State_R <= S_STREAM_INPUT;
              end if;
            else
              Stage_Output_Count_R <= Stage_Output_Count_R + 1;
            end if;
          end if;
        end if;

        if Pipeline_State_R = S_EMIT_OUTPUT then
          if Output_Sample_Count_R < G_FFT_SIZE then
            if Output_Lane_Count_R = 0 then
              Data_Out_I_R <= (others => '0');
              Data_Out_Q_R <= (others => '0');
            end if;

            for lane_idx in 0 to G_SAMPLES_PER_CLK - 1 loop
              sample_index := Output_Sample_Count_R + lane_idx;
              if sample_index < G_FFT_SIZE then
                Data_Out_I_R(((lane_idx + 1) * C_OUTPUT_WIDTH) - 1 downto lane_idx * C_OUTPUT_WIDTH) <= std_logic_vector(fft_resize_saturate(Output_Frame_R(sample_index).re, C_OUTPUT_WIDTH));
                Data_Out_Q_R(((lane_idx + 1) * C_OUTPUT_WIDTH) - 1 downto lane_idx * C_OUTPUT_WIDTH) <= std_logic_vector(fft_resize_saturate(Output_Frame_R(sample_index).im, C_OUTPUT_WIDTH));
              end if;
            end loop;
            Data_Out_V_R <= '1';

            if Output_Sample_Count_R + G_SAMPLES_PER_CLK >= G_FFT_SIZE then
              Output_Sample_Count_R <= 0;
              Pipeline_State_R <= S_IDLE;
            else
              Output_Sample_Count_R <= Output_Sample_Count_R + G_SAMPLES_PER_CLK;
            end if;
          end if;
        end if;
      end if;
    end if;
  end process;

  Data_Out_I <= Data_Out_I_R;
  Data_Out_Q <= Data_Out_Q_R;
  Data_Out_V <= Data_Out_V_R;
end architecture rtl;
