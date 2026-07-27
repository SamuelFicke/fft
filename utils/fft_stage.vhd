library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

use work.fft_pkg.all;
use work.fft_twiddle.all;

-- A single pipelined FFT stage, modeled after the MDF stage described in
-- "A Survey on Pipelined FFT Hardware Architectures" (Garrido, 2021):
-- the stage is a self-contained pipelined unit that keeps processing
-- continuously. Internally it uses a ping-pong pair of frame buffers so
-- that while one frame is being drained (streamed through the real
-- pipelined butterfly datapath and streamed out), the next frame can
-- already be received. Chaining several of these stages (as fft_mdf does)
-- yields multiple frames in flight concurrently, one per stage, instead of
-- a single state machine that reuses one stage at a time.
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

  function max_int(lhs : integer; rhs : integer) return integer is
  begin
    if lhs > rhs then
      return lhs;
    end if;
    return rhs;
  end function;

  constant C_STRIDE                : integer := 2 ** G_STAGE_INDEX;
  constant C_HALF_STRIDE           : integer := C_STRIDE / 2;
  constant C_BUTTERFLIES_PER_STAGE : integer := G_FFT_SIZE / 2;
  constant C_LANES                 : integer := min_int(C_BUTTERFLIES_PER_STAGE, max_int(1, G_BUTTERFLY_LANES));
  constant C_TWIDDLE_FRACTION_BITS : integer := G_TWIDDLE_WIDTH - 2;
  constant C_ZERO_SAMPLE           : t_complex_sample := ((others => '0'), (others => '0'));

  type t_buffer_pair is array (0 to 1) of t_complex_array(0 to G_FFT_SIZE - 1);
  type t_lane_index_range is array (0 to C_LANES - 1) of integer;
  type t_lane_word is array (0 to C_LANES - 1) of signed(G_INTERNAL_WIDTH - 1 downto 0);
  type t_lane_bit is array (0 to C_LANES - 1) of std_logic;

  signal Buf_R              : t_buffer_pair := (others => (others => C_ZERO_SAMPLE));
  signal Fill_Sel_R         : integer range 0 to 1 := 0;
  signal Drain_Sel_R        : integer range 0 to 1 := 0;
  signal Input_Count_R      : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Pending_Swap_R     : std_logic := '0';
  signal Drain_Busy_R       : std_logic := '0';

  signal Submit_Count_R     : integer range 0 to C_BUTTERFLIES_PER_STAGE := 0;
  signal Submitting_R       : std_logic := '0';
  signal Completed_Count_R  : integer range 0 to C_BUTTERFLIES_PER_STAGE := 0;
  signal Streaming_R        : std_logic := '0';
  signal Output_Count_R     : integer range 0 to G_FFT_SIZE - 1 := 0;
  signal Sample_Out_R       : t_complex_sample := C_ZERO_SAMPLE;
  signal Sample_Out_V_R     : std_logic := '0';

  -- Per-lane butterfly datapath signals (combinational drives into the
  -- pipelined fft_butterfly instances, based on registered submit state).
  signal Lane_In_V          : t_lane_bit := (others => '0');
  signal Lane_A_Re, Lane_A_Im, Lane_B_Re, Lane_B_Im, Lane_Tw_Re, Lane_Tw_Im : t_lane_word;
  signal Lane_Idx_A, Lane_Idx_B : t_lane_index_range;

  signal Lane_Sum_Re, Lane_Sum_Im, Lane_Diff_Re, Lane_Diff_Im : t_lane_word;
  signal Lane_Out_Idx_A, Lane_Out_Idx_B : t_lane_index_range;
  signal Lane_Out_V         : t_lane_bit;
begin
  assert G_STAGE_INDEX > 0 report "G_STAGE_INDEX must be greater than zero" severity failure;
  assert G_FFT_SIZE > 1 report "G_FFT_SIZE must be greater than 1" severity failure;
  assert C_LANES > 0 report "C_LANES must be greater than zero" severity failure;

  Sample_In_Ready <= not Pending_Swap_R;
  Sample_Out <= Sample_Out_R;
  Sample_Out_V <= Sample_Out_V_R;

  -- Drive lane butterfly inputs combinationally from the current submit
  -- pointer(s); one pair of samples is dispatched per lane per cycle while
  -- Submitting_R is asserted.
  gen_lane_drive : for lane in 0 to C_LANES - 1 generate
    signal butterfly_idx : integer := 0;
    signal group_idx      : integer := 0;
    signal pair_idx        : integer := 0;
    signal idx_a           : integer := 0;
    signal idx_b           : integer := 0;
    signal twiddle_s       : t_complex_sample := C_ZERO_SAMPLE;
    signal lane_active     : boolean := false;
  begin
    lane_active <= Submitting_R = '1' and (Submit_Count_R + lane) < C_BUTTERFLIES_PER_STAGE;
    butterfly_idx <= (Submit_Count_R + lane) when (Submit_Count_R + lane) < C_BUTTERFLIES_PER_STAGE else 0;
    group_idx <= butterfly_idx / C_HALF_STRIDE;
    pair_idx  <= butterfly_idx mod C_HALF_STRIDE;
    idx_a <= (group_idx * C_STRIDE + pair_idx) mod G_FFT_SIZE;
    idx_b <= (group_idx * C_STRIDE + pair_idx + C_HALF_STRIDE) mod G_FFT_SIZE;
    twiddle_s <= fft_twiddle_value(G_STAGE_INDEX, pair_idx, G_IS_FORWARD, G_TWIDDLE_WIDTH, C_TWIDDLE_FRACTION_BITS);

    Lane_In_V(lane) <= '1' when lane_active else '0';
    Lane_A_Re(lane) <= Buf_R(Drain_Sel_R)(idx_a).re;
    Lane_A_Im(lane) <= Buf_R(Drain_Sel_R)(idx_a).im;
    Lane_B_Re(lane) <= Buf_R(Drain_Sel_R)(idx_b).re;
    Lane_B_Im(lane) <= Buf_R(Drain_Sel_R)(idx_b).im;
    Lane_Tw_Re(lane) <= twiddle_s.re;
    Lane_Tw_Im(lane) <= twiddle_s.im;
    Lane_Idx_A(lane) <= idx_a;
    Lane_Idx_B(lane) <= idx_b;

    lane_butterfly : entity work.fft_butterfly
      generic map (
        G_DATA_WIDTH          => G_INTERNAL_WIDTH,
        G_TWIDDLE_SHIFT_RIGHT => C_TWIDDLE_FRACTION_BITS,
        G_MUL_PIPE_STAGES     => G_MUL_PIPE_STAGES,
        G_MAX_INDEX           => G_FFT_SIZE
      )
      port map (
        Clk        => Clk,
        Rst        => Rst,
        Ce         => '1',
        In_V       => Lane_In_V(lane),
        A_Re       => Lane_A_Re(lane),
        A_Im       => Lane_A_Im(lane),
        B_Re       => Lane_B_Re(lane),
        B_Im       => Lane_B_Im(lane),
        Tw_Re      => Lane_Tw_Re(lane),
        Tw_Im      => Lane_Tw_Im(lane),
        In_Index_A => Lane_Idx_A(lane),
        In_Index_B => Lane_Idx_B(lane),
        Sum_Re     => Lane_Sum_Re(lane),
        Sum_Im     => Lane_Sum_Im(lane),
        Diff_Re    => Lane_Diff_Re(lane),
        Diff_Im    => Lane_Diff_Im(lane),
        Out_Index_A => Lane_Out_Idx_A(lane),
        Out_Index_B => Lane_Out_Idx_B(lane),
        Out_V      => Lane_Out_V(lane)
      );
  end generate;

  process(Clk)
    variable completed_this_cycle_v : integer;
  begin
    if rising_edge(Clk) then
      if Rst = '1' then
        Buf_R <= (others => (others => C_ZERO_SAMPLE));
        Fill_Sel_R <= 0;
        Drain_Sel_R <= 0;
        Input_Count_R <= 0;
        Pending_Swap_R <= '0';
        Drain_Busy_R <= '0';
        Submit_Count_R <= 0;
        Submitting_R <= '0';
        Completed_Count_R <= 0;
        Streaming_R <= '0';
        Output_Count_R <= 0;
        Sample_Out_R <= C_ZERO_SAMPLE;
        Sample_Out_V_R <= '0';
      else
        Sample_Out_V_R <= '0';

        -- Accept incoming samples into the fill buffer, one per cycle.
        if Sample_In_V = '1' and Pending_Swap_R = '0' then
          Buf_R(Fill_Sel_R)(Input_Count_R) <= Sample_In;
          if Input_Count_R = G_FFT_SIZE - 1 then
            Input_Count_R <= 0;
            if Drain_Busy_R = '0' then
              Drain_Sel_R <= Fill_Sel_R;
              Fill_Sel_R <= 1 - Fill_Sel_R;
              Drain_Busy_R <= '1';
              Submit_Count_R <= 0;
              Completed_Count_R <= 0;
              Submitting_R <= '1';
              Streaming_R <= '0';
              Output_Count_R <= 0;
            else
              Pending_Swap_R <= '1';
            end if;
          else
            Input_Count_R <= Input_Count_R + 1;
          end if;
        end if;

        -- Once the previous drain buffer frees up, swap in the frame that
        -- finished filling while we were still draining.
        if Drain_Busy_R = '0' and Pending_Swap_R = '1' then
          Drain_Sel_R <= Fill_Sel_R;
          Fill_Sel_R <= 1 - Fill_Sel_R;
          Pending_Swap_R <= '0';
          Drain_Busy_R <= '1';
          Submit_Count_R <= 0;
          Completed_Count_R <= 0;
          Submitting_R <= '1';
          Streaming_R <= '0';
          Output_Count_R <= 0;
        end if;

        -- Advance the submit pointer for the currently draining frame.
        if Submitting_R = '1' then
          if Submit_Count_R + C_LANES >= C_BUTTERFLIES_PER_STAGE then
            Submit_Count_R <= C_BUTTERFLIES_PER_STAGE;
            Submitting_R <= '0';
          else
            Submit_Count_R <= Submit_Count_R + C_LANES;
          end if;
        end if;

        -- Capture butterfly results as they emerge from the pipelined
        -- lanes (results are written back in place; index tags guarantee
        -- correct placement regardless of pipeline latency). All lanes
        -- that complete on the same cycle are accumulated together via a
        -- variable so the completed-count doesn't race across lanes.
        completed_this_cycle_v := 0;
        for lane in 0 to C_LANES - 1 loop
          if Lane_Out_V(lane) = '1' then
            Buf_R(Drain_Sel_R)(Lane_Out_Idx_A(lane)).re <= Lane_Sum_Re(lane);
            Buf_R(Drain_Sel_R)(Lane_Out_Idx_A(lane)).im <= Lane_Sum_Im(lane);
            Buf_R(Drain_Sel_R)(Lane_Out_Idx_B(lane)).re <= Lane_Diff_Re(lane);
            Buf_R(Drain_Sel_R)(Lane_Out_Idx_B(lane)).im <= Lane_Diff_Im(lane);
            completed_this_cycle_v := completed_this_cycle_v + 1;
          end if;
        end loop;
        if completed_this_cycle_v > 0 then
          if Completed_Count_R + completed_this_cycle_v >= C_BUTTERFLIES_PER_STAGE then
            Streaming_R <= '1';
            Output_Count_R <= 0;
          end if;
          Completed_Count_R <= Completed_Count_R + completed_this_cycle_v;
        end if;

        -- Stream the completed frame out one sample per cycle.
        if Streaming_R = '1' and Sample_Out_Ready = '1' then
          Sample_Out_R <= Buf_R(Drain_Sel_R)(Output_Count_R);
          Sample_Out_V_R <= '1';
          if Output_Count_R = G_FFT_SIZE - 1 then
            Streaming_R <= '0';
            Drain_Busy_R <= '0';
            Output_Count_R <= 0;
          else
            Output_Count_R <= Output_Count_R + 1;
          end if;
        end if;
      end if;
    end if;
  end process;
end architecture rtl;
