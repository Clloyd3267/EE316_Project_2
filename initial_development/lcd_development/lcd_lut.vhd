--------------------------------------------------------------------------------
-- Filename     : lcd_lut.vhd
-- Author(s)    : Chris Lloyd
-- Class        : EE316 (Project 2)
-- Due Date     : 2021-02-23
-- Target Board : Altera DE2 Devkit
-- Entity       : lcd_lut
-- Description  : A lookup table (lut) to decide what data (O_LCD_DATA) gets
--                displayed to the LCD module depending on the current mode
--                (I_MODE).
--------------------------------------------------------------------------------

-----------------
--  Libraries  --
-----------------
library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.lcd_screen_util.all;

--------------
--  Entity  --
--------------
entity lcd_lut is
generic
(
  C_CLK_FREQ_MHZ : integer := 50  -- System clock frequency in MHz
);
port
(
  I_CLK          : in std_logic;  -- System clk frequency of (C_CLK_FREQ_MHZ)
  I_RESET_N      : in std_logic;  -- System reset (active low)

  -- Mode of operation
  I_MODE         : in std_logic_vector(1 downto 0);

  -- Output Frequency
  I_PWM_FREQ     : in std_logic_vector(1 downto 0);

  -- 16-bit Data (4 hex nibbles)
  I_DATA         : in std_logic_vector(15 downto 0);

  -- 8-bit Address (2 hex nibbles)
  I_ADDRESS      : in std_logic_vector(7 downto 0);

  -- Output LCD "screen" array type (see lcd_display_driver.vhd:19 for type def)
  O_LCD_DATA     : out t_lcd_display_data
);
end entity lcd_lut;

--------------------------------
--  Architecture Declaration  --
--------------------------------
architecture behavioral of lcd_lut is

  ---------------
  -- Constants --
  ---------------

  -- Mode of Operation
  -- 00: Initialization
  -- 01: Test
  -- 10: Pause
  -- 11: PWM Generation
  constant C_MODE_INIT  : std_logic_vector(1 downto 0) := "00";
  constant C_MODE_TEST  : std_logic_vector(1 downto 0) := "01";
  constant C_MODE_PAUSE : std_logic_vector(1 downto 0) := "10";
  constant C_MODE_PWM   : std_logic_vector(1 downto 0) := "11";

  -- Output Frequency
  -- 00: 60 Hz
  -- 01: 120 Hz
  -- 10: 1000 Hz (1KHz)
  -- 11: Undefined (pick one)
  constant C_60_HZ  : std_logic_vector(1 downto 0) := "00";
  constant C_120_HZ : std_logic_vector(1 downto 0) := "01";
  constant C_1_KHZ  : std_logic_vector(1 downto 0) := "10";

  -- Ascii constants for writing "Strings"
  -- Upper case alphabet
  constant UA  : std_logic_vector(7 downto 0) := x"41";  -- A
  constant UB  : std_logic_vector(7 downto 0) := x"42";  -- B
  constant UC  : std_logic_vector(7 downto 0) := x"43";  -- C
  constant UD  : std_logic_vector(7 downto 0) := x"44";  -- D
  constant UE  : std_logic_vector(7 downto 0) := x"45";  -- E
  constant UF  : std_logic_vector(7 downto 0) := x"46";  -- F
  constant UG  : std_logic_vector(7 downto 0) := x"47";  -- G
  constant UH  : std_logic_vector(7 downto 0) := x"48";  -- H
  constant UI  : std_logic_vector(7 downto 0) := x"49";  -- I
  constant UJ  : std_logic_vector(7 downto 0) := x"4A";  -- J
  constant UK  : std_logic_vector(7 downto 0) := x"4B";  -- K
  constant UL  : std_logic_vector(7 downto 0) := x"4C";  -- L
  constant UM  : std_logic_vector(7 downto 0) := x"4D";  -- M
  constant UN  : std_logic_vector(7 downto 0) := x"4E";  -- N
  constant UO  : std_logic_vector(7 downto 0) := x"4F";  -- O
  constant UP  : std_logic_vector(7 downto 0) := x"50";  -- P
  constant UQ  : std_logic_vector(7 downto 0) := x"51";  -- Q
  constant UR  : std_logic_vector(7 downto 0) := x"52";  -- R
  constant US  : std_logic_vector(7 downto 0) := x"53";  -- S
  constant UT  : std_logic_vector(7 downto 0) := x"54";  -- T
  constant UU  : std_logic_vector(7 downto 0) := x"55";  -- U
  constant UV  : std_logic_vector(7 downto 0) := x"56";  -- V
  constant UW  : std_logic_vector(7 downto 0) := x"57";  -- W
  constant UX  : std_logic_vector(7 downto 0) := x"58";  -- X
  constant UY  : std_logic_vector(7 downto 0) := x"59";  -- Y
  constant UZ  : std_logic_vector(7 downto 0) := x"5A";  -- Z

  -- Lower case alphabet
  constant LA  : std_logic_vector(7 downto 0) := x"61";  -- a
  constant LB  : std_logic_vector(7 downto 0) := x"62";  -- b
  constant LC  : std_logic_vector(7 downto 0) := x"63";  -- c
  constant LD  : std_logic_vector(7 downto 0) := x"64";  -- d
  constant LE  : std_logic_vector(7 downto 0) := x"65";  -- e
  constant LF  : std_logic_vector(7 downto 0) := x"66";  -- f
  constant LG  : std_logic_vector(7 downto 0) := x"67";  -- g
  constant LH  : std_logic_vector(7 downto 0) := x"68";  -- h
  constant LI  : std_logic_vector(7 downto 0) := x"69";  -- i
  constant LJ  : std_logic_vector(7 downto 0) := x"6A";  -- j
  constant LK  : std_logic_vector(7 downto 0) := x"6B";  -- k
  constant LL  : std_logic_vector(7 downto 0) := x"6C";  -- l
  constant LM  : std_logic_vector(7 downto 0) := x"6D";  -- m
  constant LN  : std_logic_vector(7 downto 0) := x"6E";  -- n
  constant LO  : std_logic_vector(7 downto 0) := x"6F";  -- o
  constant LP  : std_logic_vector(7 downto 0) := x"70";  -- p
  constant LQ  : std_logic_vector(7 downto 0) := x"71";  -- q
  constant LR  : std_logic_vector(7 downto 0) := x"72";  -- r
  constant LS  : std_logic_vector(7 downto 0) := x"73";  -- s
  constant LT  : std_logic_vector(7 downto 0) := x"74";  -- t
  constant LU  : std_logic_vector(7 downto 0) := x"75";  -- u
  constant LV  : std_logic_vector(7 downto 0) := x"76";  -- v
  constant LW  : std_logic_vector(7 downto 0) := x"77";  -- w
  constant LX  : std_logic_vector(7 downto 0) := x"78";  -- x
  constant LY  : std_logic_vector(7 downto 0) := x"79";  -- y
  constant LZ  : std_logic_vector(7 downto 0) := x"7A";  -- z

  -- Numeric 0-9
  constant N0    : std_logic_vector(7 downto 0) := x"30";  -- 0
  constant N1    : std_logic_vector(7 downto 0) := x"31";  -- 1
  constant N2    : std_logic_vector(7 downto 0) := x"32";  -- 2
  constant N3    : std_logic_vector(7 downto 0) := x"33";  -- 3
  constant N4    : std_logic_vector(7 downto 0) := x"34";  -- 4
  constant N5    : std_logic_vector(7 downto 0) := x"35";  -- 5
  constant N6    : std_logic_vector(7 downto 0) := x"36";  -- 6
  constant N7    : std_logic_vector(7 downto 0) := x"37";  -- 7
  constant N8    : std_logic_vector(7 downto 0) := x"38";  -- 8
  constant N9    : std_logic_vector(7 downto 0) := x"39";  -- 9

  -- Other useful constants
  constant SP : std_logic_vector(7 downto 0) := x"20";  -- Space
  constant EX : std_logic_vector(7 downto 0) := x"21";  -- !
  constant CL : std_logic_vector(7 downto 0) := x"3A";  -- :

  -------------
  -- SIGNALS --
  -------------
  type t_lcd_addr_ascii is array (1 downto 0) of std_logic_vector(7 downto 0);
  type t_lcd_data_ascii is array (3 downto 0) of std_logic_vector(7 downto 0);
  type t_lcd_freq_ascii is array (6 downto 0) of std_logic_vector(7 downto 0);

  signal s_addr_ascii : t_lcd_addr_ascii := (others=>(others=>('0')));
  signal s_data_ascii : t_lcd_data_ascii := (others=>(others=>('0')));
  signal s_freq_ascii : t_lcd_freq_ascii := (others=>(others=>('0')));

begin

  ------------------------------------------------------------------------------
  -- Process Name     : LCD_LUT_DATA_LATCH
  -- Sensitivity List : I_CLK               : System clock
  --                    I_RESET_N           : System reset (active low logic)
  -- Useful Outputs   :
  -- Description      :
  ------------------------------------------------------------------------------
  LCD_LUT_DATA_LATCH: process (I_CLK, I_RESET_N)
  begin
    if (I_RESET_N = '0') then
      O_LCD_DATA     <= (others=>(others=>('0')));

    elsif (rising_edge(I_CLK)) then
      case(I_MODE) is -- CDL=> Fix Index/explain
        when C_MODE_INIT  =>
          -- [..Initializing..]
          -- [................]
          O_LCD_DATA <=
          (
            SP, SP, UI, LN, LI, LT, LI, LA, LL, LI, LZ, LI, LN, LG, SP, SP,
            SP, SP, SP, SP, SP, SP, SP, SP, SP, SP, SP, SP, SP, SP, SP, SP
          );

        when C_MODE_TEST  =>
          -- [...Test.Mode....]
          -- [..0xFF.:.0xFFFF.]
          O_LCD_DATA <=
          (
            SP, SP, SP, UT, LE, LS, LT, SP, UM, LO, LD, LE, SP, SP, SP, SP,
            SP, N0, LX, s_addr_ascii(1), s_addr_ascii(0), SP, CL, SP, N0, LX,
            s_data_ascii(3), s_data_ascii(2), s_data_ascii(1), s_data_ascii(0),
            SP, SP
          );

         when C_MODE_PAUSE =>
          -- [...Pause.Mode...]
          -- [................]
          O_LCD_DATA <=
          (
            SP, SP, SP, UP, LA, LU, LS, LE, SP, UM, LO, LD, LE, SP, SP, SP,
            SP, N0, LX, s_addr_ascii(1), s_addr_ascii(0), SP, CL, SP, N0, LX,
            s_data_ascii(3), s_data_ascii(2), s_data_ascii(1), s_data_ascii(0),
            SP, SP
          );

        when C_MODE_PWM   =>
          -- [.PWM.Generation.]
          -- [..Freq.:.NNNN.Hz]
          O_LCD_DATA <=
          (
            SP, UP, UW, UM, SP, UG, LE, LN, LE, LR, LA, LT, LI, LO, LN, SP,
            SP, UF, LR, LE, LQ, SP, CL, SP, s_freq_ascii(6), s_freq_ascii(5),
            s_freq_ascii(4), s_freq_ascii(3), s_freq_ascii(2), s_freq_ascii(1),
            s_freq_ascii(0), SP
          );

        when others =>
          O_LCD_DATA <= (others=>(others=>('0')));
      end case;
    end if;
  end process LCD_LUT_DATA_LATCH;
  ------------------------------------------------------------------------------

  -- First bit of address
  s_addr_ascii(0) <= x"3" & I_ADDRESS(3 downto 0) when I_ADDRESS(3 downto 0) < x"A"   -- 0-9
                else x"41"                        when I_ADDRESS(3 downto 0) = x"A"   -- A
                else x"42"                        when I_ADDRESS(3 downto 0) = x"B"   -- B
                else x"43"                        when I_ADDRESS(3 downto 0) = x"C"   -- C
                else x"44"                        when I_ADDRESS(3 downto 0) = x"D"   -- D
                else x"45"                        when I_ADDRESS(3 downto 0) = x"E"   -- E
                else x"46"                        when I_ADDRESS(3 downto 0) = x"F";  -- F

  -- Second bit of address
  s_addr_ascii(1) <= x"3" & I_ADDRESS(7 downto 4) when I_ADDRESS(7 downto 4) < x"A"   -- 0-9
                else x"41"                        when I_ADDRESS(7 downto 4) = x"A"   -- A
                else x"42"                        when I_ADDRESS(7 downto 4) = x"B"   -- B
                else x"43"                        when I_ADDRESS(7 downto 4) = x"C"   -- C
                else x"44"                        when I_ADDRESS(7 downto 4) = x"D"   -- D
                else x"45"                        when I_ADDRESS(7 downto 4) = x"E"   -- E
                else x"46"                        when I_ADDRESS(7 downto 4) = x"F";  -- F

  -- First bit of data
  s_data_ascii(0) <= x"3" & I_DATA(3 downto 0) when I_DATA(3 downto 0) < x"A"   -- 0-9
                else x"41"                     when I_DATA(3 downto 0) = x"A"   -- A
                else x"42"                     when I_DATA(3 downto 0) = x"B"   -- B
                else x"43"                     when I_DATA(3 downto 0) = x"C"   -- C
                else x"44"                     when I_DATA(3 downto 0) = x"D"   -- D
                else x"45"                     when I_DATA(3 downto 0) = x"E"   -- E
                else x"46"                     when I_DATA(3 downto 0) = x"F";  -- F

  -- Second bit of data
  s_data_ascii(1) <= x"3" & I_DATA(7 downto 4) when I_DATA(7 downto 4) < x"A"   -- 0-9
                else x"41"                     when I_DATA(7 downto 4) = x"A"   -- A
                else x"42"                     when I_DATA(7 downto 4) = x"B"   -- B
                else x"43"                     when I_DATA(7 downto 4) = x"C"   -- C
                else x"44"                     when I_DATA(7 downto 4) = x"D"   -- D
                else x"45"                     when I_DATA(7 downto 4) = x"E"   -- E
                else x"46"                     when I_DATA(7 downto 4) = x"F";  -- F

  -- Third bit of data
  s_data_ascii(2) <= x"3" & I_DATA(11 downto 8) when I_DATA(11 downto 8) < x"A"   -- 0-9
                else x"41"                      when I_DATA(11 downto 8) = x"A"   -- A
                else x"42"                      when I_DATA(11 downto 8) = x"B"   -- B
                else x"43"                      when I_DATA(11 downto 8) = x"C"   -- C
                else x"44"                      when I_DATA(11 downto 8) = x"D"   -- D
                else x"45"                      when I_DATA(11 downto 8) = x"E"   -- E
                else x"46"                      when I_DATA(11 downto 8) = x"F";  -- F

  -- Fourth bit of data
  s_data_ascii(3) <= x"3" & I_DATA(15 downto 12) when I_DATA(15 downto 12) < x"A"   -- 0-9
                else x"41"                       when I_DATA(15 downto 12) = x"A"   -- A
                else x"42"                       when I_DATA(15 downto 12) = x"B"   -- B
                else x"43"                       when I_DATA(15 downto 12) = x"C"   -- C
                else x"44"                       when I_DATA(15 downto 12) = x"D"   -- D
                else x"45"                       when I_DATA(15 downto 12) = x"E"   -- E
                else x"46"                       when I_DATA(15 downto 12) = x"F";  -- F

  -- Frequency ascii "string"
  s_freq_ascii <= (N6, N0, SP, UH, LZ, SP, SP) when I_PWM_FREQ = C_60_HZ
             else (N1, N2, N0, SP, UH, LZ, SP) when I_PWM_FREQ = C_120_HZ
             else (N1, N0, N0, N0, SP, UH, LZ) when I_PWM_FREQ = C_1_KHZ;

end architecture behavioral;
