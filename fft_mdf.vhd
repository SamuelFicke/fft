library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

library work;
  use work.fft_pkg.all;
  use work.fft_twiddle.all;

entity polyphase_decimator is
  generic (
    G_SAMPLE_WIDTH    : integer := 16;
    G_TWIDDLE_WIDTH   : integer := 16;
    G_FFT_SIZE        : integer := 8;
    G_SAMPLES_PER_CLK : integer := 1;
    G_DATA_WIDTH      : integer := 16;
    G_DATA_OUT_WIDTH  : integer := 16;
    G_IS_FORWARD      : boolean := true
  );
  port (
    Clk         : in  std_logic;
    Rst         : in  std_logic;

    Sample_In_I : in  std_logic_vector(G_DATA_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0);
    Sample_In_Q : in  std_logic_vector(G_DATA_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0);
    Sample_In_V : in  std_logic;

    Data_Out_I  : out std_logic_vector(G_DATA_OUT_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0);
    Data_Out_Q  : out std_logic_vector(G_DATA_OUT_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0);
    Data_Out_V  : out std_logic
  );
end entity polyphase_decimator;

architecture rtl of polyphase_decimator is
  type t_state is (ST_IDLE, ST_CAPTURE, ST_PROCESS, ST_OUTPUT);

  type t_butterfly_result is record
    sum : t_complex_sample;
    diff : t_complex_sample;
  end record;

  constant C_LOG2_N      : integer := integer(log2(real(G_FFT_SIZE)));
  constant C_STAGES      : integer := C_LOG2_N;
  constant C_INPUT_WORDS : integer := (G_FFT_SIZE + G_SAMPLES_PER_CLK - 1) / G_SAMPLES_PER_CLK;
  constant C_ZERO_SAMPLE : t_complex_sample := ((others => '0'), (others => '0'));
  constant C_TWIDDLE_FRACTION_BITS : integer := G_TWIDDLE_WIDTH - 2;
  constant C_TWIDDLE_SCALE_FACTOR  : integer := 2 ** C_TWIDDLE_FRACTION_BITS;
  constant C_TWIDDLE_SCALE_SHIFT   : integer := C_TWIDDLE_FRACTION_BITS;

  signal State_R             : t_state := ST_IDLE;
  signal Frame_R             : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Frame_A_Buffer_R    : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Frame_B_Buffer_R    : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Input_Word_Count_R  : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Output_Word_Count_R : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Frame_A_Ready_R     : std_logic := '0';
  signal Frame_B_Ready_R     : std_logic := '0';
  signal Capture_Buffer_R    : std_logic := '0';
  signal Data_Out_I_R        : std_logic_vector(G_DATA_OUT_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0) := (others => '0');
  signal Data_Out_Q_R        : std_logic_vector(G_DATA_OUT_WIDTH * G_SAMPLES_PER_CLK - 1 downto 0) := (others => '0');
  signal Data_Out_V_R        : std_logic := '0';
  signal Stage_Counter_R     : integer range 0 to C_STAGES - 1 := 0;
  signal Group_Counter_R     : integer range 0 to (G_FFT_SIZE / 2) - 1 := 0;
  signal Pair_Counter_R      : integer range 0 to (G_FFT_SIZE / 2) - 1 := 0;
  signal Butterfly_Phase_R   : std_logic := '0';
  signal Working_Frame_R     : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);
  signal Stage_Idx_R         : integer := 0;
  signal Stride_R            : integer := 1;
  signal Half_Stride_R       : integer := 0;
  signal Group_Idx_R         : integer := 0;
  signal Pair_Idx_R          : integer := 0;
  signal Index_A_R           : integer := 0;
  signal Index_B_R           : integer := 0;
  signal Twiddle_R           : t_complex_sample := C_ZERO_SAMPLE;
  signal Stage_Output_Frame_R : t_complex_array(0 to G_FFT_SIZE - 1) := (others => C_ZERO_SAMPLE);

  -- Shared launch valid for the four butterfly multipliers. They all consume the same
  -- operand pair for a given butterfly, but each produces a different product term.
  signal Mul_Butterfly_Op_Valid_R : std_logic := '0';
  subtype t_mul_word is signed(C_FFT_INTERNAL_WIDTH - 1 downto 0);
  type t_mul_word_array is array (natural range <>) of t_mul_word;
  signal Mul_A_R : t_mul_word_array(0 to 3) := (others => (others => '0'));
  signal Mul_B_R : t_mul_word_array(0 to 3) := (others => (others => '0'));
  signal Mul_Product_R : t_mul_word_array(0 to 3) := (others => (others => '0'));
  signal Mul_Product_Valid_R : std_logic_vector(0 to 3) := (others => '0');

  -- The MDF structure is expressed as a sequence of radix-2 butterfly stages.
  -- Each stage has a delay-like buffer and a twiddle factor that is applied to the
  -- lower half of the current butterfly pair.
  function butterfly(a : t_complex_sample; b : t_complex_sample; tw : t_complex_sample) return t_butterfly_result is
    variable b_scaled : t_complex_sample;
    variable ar, ai, br, bi : integer;
    variable rr, ri : integer;
  begin
    ar := to_integer(a.re);
    ai := to_integer(a.im);
    br := to_integer(tw.re);
    bi := to_integer(tw.im);
    rr := (to_integer(b.re) * br) - (to_integer(b.im) * bi);
    ri := (to_integer(b.re) * bi) + (to_integer(b.im) * br);

    b_scaled.re := resize(fft_clip_signed(rr / C_TWIDDLE_SCALE_FACTOR, G_SAMPLE_WIDTH), C_FFT_INTERNAL_WIDTH);
    b_scaled.im := resize(fft_clip_signed(ri / C_TWIDDLE_SCALE_FACTOR, G_SAMPLE_WIDTH), C_FFT_INTERNAL_WIDTH);

    return (
      sum => fft_add_complex(a, b_scaled),
      diff => fft_sub_complex(a, b_scaled)
    );
  end function;

  function twiddle_value(stage : integer; index : integer) return t_complex_sample is
  begin
    return fft_twiddle_value(stage, index, G_IS_FORWARD, G_TWIDDLE_WIDTH, C_TWIDDLE_FRACTION_BITS);
  end function;

  function fft_transform_frame(frame_in : t_complex_array) return t_complex_array is
    variable frame : t_complex_array(0 to frame_in'length - 1);
    variable stride : integer;
    variable half_stride : integer;
    variable group_idx : integer;
    variable pair_idx : integer;
    variable idx_a : integer;
    variable idx_b : integer;
    variable twiddle : t_complex_sample;
    variable butterfly_res : t_butterfly_result;
  begin
    frame := frame_in;
    stride := 2;
    for stage_idx in 1 to C_STAGES loop
      half_stride := stride / 2;
      for group_idx in 0 to (G_FFT_SIZE / stride) - 1 loop
        for pair_idx in 0 to half_stride - 1 loop
          idx_a := group_idx * stride + pair_idx;
          idx_b := idx_a + half_stride;
          twiddle := twiddle_value(stage_idx, pair_idx);
          butterfly_res := butterfly(frame(idx_a), frame(idx_b), twiddle);
          frame(idx_a) := butterfly_res.sum;
          frame(idx_b) := butterfly_res.diff;
        end loop;
      end loop;
      stride := stride * 2;
    end loop;
    return fft_bit_reverse_permute(frame);
  end function;

begin
  assert G_FFT_SIZE > 0 report "G_FFT_SIZE must be greater than zero" severity failure;
  assert G_FFT_SIZE <= 65536 report "G_FFT_SIZE must be less than or equal to 65536" severity failure;
  assert G_SAMPLES_PER_CLK > 0 report "G_SAMPLES_PER_CLK must be greater than zero" severity failure;
  assert G_SAMPLES_PER_CLK <= 32 report "G_SAMPLES_PER_CLK must be less than or equal to 32" severity failure;
  assert fft_is_power_of_two(G_FFT_SIZE) report "G_FFT_SIZE must be a power of two" severity failure;
  assert fft_is_power_of_two(G_SAMPLES_PER_CLK) report "G_SAMPLES_PER_CLK must be a power of two" severity failure;
  assert G_DATA_WIDTH > 0 report "G_DATA_WIDTH must be greater than zero" severity failure;
  assert G_DATA_OUT_WIDTH > 0 report "G_DATA_OUT_WIDTH must be greater than zero" severity failure;
  assert G_SAMPLE_WIDTH > 0 report "G_SAMPLE_WIDTH must be greater than zero" severity failure;
  assert G_TWIDDLE_WIDTH > 0 report "G_TWIDDLE_WIDTH must be greater than zero" severity failure;
  assert G_TWIDDLE_WIDTH >= 2 report "G_TWIDDLE_WIDTH must be at least 2 bits" severity failure;

  gen_multipliers : for mul_idx in 0 to 3 generate
  begin
    mul_inst : entity work.signed_multiplier
      generic map (
        G_A_WIDTH            => C_FFT_INTERNAL_WIDTH,
        G_B_WIDTH            => C_FFT_INTERNAL_WIDTH,
        G_OUTPUT_WIDTH       => C_FFT_INTERNAL_WIDTH,
        G_OUTPUT_SHIFT_RIGHT => C_TWIDDLE_SCALE_SHIFT,
        G_OUTPUT_PIPE_STAGES => 1
      )
      port map (
        Clk                  => Clk,
        Rst                  => Rst,
        Ce                   => '1',
        Data_A               => Mul_A_R(mul_idx),
        Data_B               => Mul_B_R(mul_idx),
        Data_V               => Mul_Butterfly_Op_Valid_R,
        Product              => Mul_Product_R(mul_idx),
        Product_V            => Mul_Product_Valid_R(mul_idx)
      );
  end generate;

  process(Clk)
    variable frame_to_process : t_complex_array(0 to G_FFT_SIZE - 1);
    variable lane_idx         : integer;
    variable sample_index     : integer;
    variable sample_word_i    : std_logic_vector(G_DATA_WIDTH - 1 downto 0);
    variable sample_word_q    : std_logic_vector(G_DATA_WIDTH - 1 downto 0);
  begin
    if rising_edge(Clk) then
      if Rst = '1' then
        State_R <= ST_IDLE;
        Input_Word_Count_R <= 0;
        Output_Word_Count_R <= 0;
        Data_Out_I_R <= (others => '0');
        Data_Out_Q_R <= (others => '0');
        Data_Out_V_R <= '0';
        Frame_R <= (others => C_ZERO_SAMPLE);
        Frame_A_Buffer_R <= (others => C_ZERO_SAMPLE);
        Frame_B_Buffer_R <= (others => C_ZERO_SAMPLE);
        Frame_A_Ready_R <= '0';
        Frame_B_Ready_R <= '0';
        Capture_Buffer_R <= '0';
      else
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
              Frame_A_Ready_R <= '1';
            else
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

        case State_R is
          when ST_IDLE =>
            Data_Out_V_R <= '0';
            if Frame_A_Ready_R = '1' then
              frame_to_process := Frame_A_Buffer_R;
              Frame_A_Ready_R <= '0';
              Frame_R <= fft_transform_frame(frame_to_process);
              State_R <= ST_OUTPUT;
              Output_Word_Count_R <= 0;
            elsif Frame_B_Ready_R = '1' then
              frame_to_process := Frame_B_Buffer_R;
              Frame_B_Ready_R <= '0';
              Frame_R <= fft_transform_frame(frame_to_process);
              State_R <= ST_OUTPUT;
              Output_Word_Count_R <= 0;
            end if;

          when ST_OUTPUT =>
            Data_Out_V_R <= '1';
            for lane_idx in 0 to G_SAMPLES_PER_CLK - 1 loop
              sample_index := Output_Word_Count_R * G_SAMPLES_PER_CLK + lane_idx;
              if sample_index < G_FFT_SIZE then
                Data_Out_I_R(((lane_idx + 1) * G_DATA_OUT_WIDTH) - 1 downto lane_idx * G_DATA_OUT_WIDTH) <= std_logic_vector(resize(Frame_R(sample_index).re, G_DATA_OUT_WIDTH));
                Data_Out_Q_R(((lane_idx + 1) * G_DATA_OUT_WIDTH) - 1 downto lane_idx * G_DATA_OUT_WIDTH) <= std_logic_vector(resize(Frame_R(sample_index).im, G_DATA_OUT_WIDTH));
              else
                Data_Out_I_R(((lane_idx + 1) * G_DATA_OUT_WIDTH) - 1 downto lane_idx * G_DATA_OUT_WIDTH) <= (others => '0');
                Data_Out_Q_R(((lane_idx + 1) * G_DATA_OUT_WIDTH) - 1 downto lane_idx * G_DATA_OUT_WIDTH) <= (others => '0');
              end if;
            end loop;

            if Output_Word_Count_R = C_INPUT_WORDS - 1 then
              State_R <= ST_IDLE;
              Output_Word_Count_R <= 0;
            else
              Output_Word_Count_R <= Output_Word_Count_R + 1;
            end if;

          when others =>
            State_R <= ST_IDLE;
        end case;
      end if;
    end if;
  end process;

  Data_Out_I <= Data_Out_I_R;
  Data_Out_Q <= Data_Out_Q_R;
  Data_Out_V <= Data_Out_V_R;
end architecture rtl;
