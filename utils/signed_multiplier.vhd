library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

entity signed_multiplier is
  generic (
    G_A_WIDTH            : positive;
    G_B_WIDTH            : positive;
    G_OUTPUT_WIDTH       : positive := 32;
    G_OUTPUT_SHIFT_RIGHT : natural := 0;
    G_ROUND_OUTPUT       : boolean := false;
    G_SATURATE_OUTPUT    : boolean := false;
    -- Number of registered stages after the multiply register.
    G_OUTPUT_PIPE_STAGES : natural := 1
  );
  port (
    Clk       : in  std_logic;
    Rst       : in  std_logic;
    Ce        : in  std_logic := '1';
    Data_A    : in  signed(G_A_WIDTH - 1 downto 0);
    Data_B    : in  signed(G_B_WIDTH - 1 downto 0);
    Data_V    : in  std_logic;
    Product   : out signed(G_OUTPUT_WIDTH - 1 downto 0);
    Product_V : out std_logic
  );
end entity signed_multiplier;

architecture rtl of signed_multiplier is

  function max_int(Lhs : natural; Rhs : natural) return natural is
  begin
    if Lhs > Rhs then
      return Lhs;
    end if;

    return Rhs;
  end function;

  constant C_PRODUCT_WIDTH : positive := G_A_WIDTH + G_B_WIDTH;
  constant C_FULL_WIDTH    : positive := max_int(C_PRODUCT_WIDTH, G_OUTPUT_WIDTH);
  constant C_PIPE_LENGTH   : positive := G_OUTPUT_PIPE_STAGES + 1;

  function saturate_max return signed is
    variable Result : signed(G_OUTPUT_WIDTH - 1 downto 0) := (others => '1');
  begin
    Result(Result'left) := '0';
    return Result;
  end function;

  function saturate_min return signed is
    variable Result : signed(G_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
  begin
    Result(Result'left) := '1';
    return Result;
  end function;

  function format_product(Value : signed) return signed is
    variable Adjusted : signed(C_FULL_WIDTH - 1 downto 0) := resize(Value, C_FULL_WIDTH);
    variable Shifted  : signed(C_FULL_WIDTH - 1 downto 0);
    variable Result   : signed(G_OUTPUT_WIDTH - 1 downto 0);
  begin
    -- Optional fixed-point rounding before the arithmetic right shift.
    if G_ROUND_OUTPUT and (G_OUTPUT_SHIFT_RIGHT > 0) then
      Adjusted := Adjusted + shift_left(to_signed(1, C_FULL_WIDTH), G_OUTPUT_SHIFT_RIGHT - 1);
    end if;

    if G_OUTPUT_SHIFT_RIGHT > 0 then
      Shifted := shift_right(Adjusted, G_OUTPUT_SHIFT_RIGHT);
    else
      Shifted := Adjusted;
    end if;

    Result := resize(Shifted, G_OUTPUT_WIDTH);

    -- Saturation is applied only when the formatted width truncates sign-extension bits.
    if G_SATURATE_OUTPUT and (G_OUTPUT_WIDTH < C_FULL_WIDTH) then
      for Idx in G_OUTPUT_WIDTH to C_FULL_WIDTH - 1 loop
        if Shifted(Idx) /= Result(Result'left) then
          if Shifted(Shifted'left) = '0' then
            return saturate_max;
          end if;

          return saturate_min;
        end if;
      end loop;
    end if;

    return Result;
  end function;
  subtype t_a is signed(G_A_WIDTH - 1 downto 0);
  subtype t_b is signed(G_B_WIDTH - 1 downto 0);
  subtype t_product is signed(G_OUTPUT_WIDTH - 1 downto 0);
  subtype t_full_product is signed(C_PRODUCT_WIDTH - 1 downto 0);

  type t_product_array is array (natural range <>) of t_product;

  signal Data_A_R      : t_a := (others => '0');
  signal Data_B_R      : t_b := (others => '0');
  signal Full_Product_R : t_full_product := (others => '0');
  signal Product_Pipe   : t_product_array(0 to C_PIPE_LENGTH - 1) := (others => (others => '0'));
  signal Valid_Pipe     : std_logic_vector(0 to C_PIPE_LENGTH + 1) := (others => '0');

begin

  Product   <= Product_Pipe(C_PIPE_LENGTH - 1);
  Product_V <= Valid_Pipe(C_PIPE_LENGTH + 1);

  process (Clk)
  begin
    if rising_edge(Clk) then
      if Rst = '1' then
        Data_A_R       <= (others => '0');
        Data_B_R       <= (others => '0');
        Full_Product_R <= (others => '0');
        Product_Pipe   <= (others => (others => '0'));
        Valid_Pipe     <= (others => '0');
      elsif Ce = '1' then
        -- Input register stage.
        Data_A_R        <= Data_A;
        Data_B_R        <= Data_B;
        -- Dedicated multiply register stage for Fmax.
        Full_Product_R  <= Data_A_R * Data_B_R;
        -- Output formatting stage (shift/round/saturate) then optional pipeline stages.
        Product_Pipe(0) <= format_product(Full_Product_R);
        Valid_Pipe(0)   <= Data_V;
        Valid_Pipe(1)   <= Valid_Pipe(0);
        Valid_Pipe(2)   <= Valid_Pipe(1);

        for Idx in 1 to C_PIPE_LENGTH - 1 loop
          Product_Pipe(Idx)   <= Product_Pipe(Idx - 1);
          Valid_Pipe(Idx + 2) <= Valid_Pipe(Idx + 1);
        end loop;
      end if;
    end if;
  end process;

end architecture rtl;