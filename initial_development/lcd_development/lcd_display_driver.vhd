--------------------------------------------------------------------------------
-- Filename     : lcd_display_driver.vhd
-- Author(s)    : Chris Lloyd
-- Class        : EE316 (Project 2)
-- Due Date     : 2021-02-23
-- Target Board : Altera DE2 Devkit
-- Entity       : lcd_display_driver
-- Description  : Driver for a 16x2 LCD module. This driver takes in a custom
--                LCD data signal representing a "screen" (16x2) of data.
--                Uses the following low level controller:
--                https://www.digikey.com/eewiki/pages/viewpage.action?pageId=4096079
--------------------------------------------------------------------------------

----------------
--  Packages  --
----------------
library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
-- Custom package for LCD data representing a "screen" (16x2) of data.
package lcd_screen_util is
  type t_lcd_display_data is array (31 downto 0) of std_logic_vector(7 downto 0);
end package lcd_screen_util;

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
entity lcd_display_driver is
generic
(
  C_CLK_FREQ_MHZ   : integer := 50;      -- System clock frequency in MHz

  -- LCD Specific Settings
  C_NUM_DISP_LINES : std_logic := '1';       -- Number of lines to display (1-line mode: '0', 2-line mode: '1')
  C_CHAR_FONT      : std_logic := '1';       -- Character font (5x8 dot: '0', 5x10 dots: '1')
  C_DISP_EN        : std_logic := '1';       -- Display enable (display off: '0', display on: '1')
  C_CURSOR_EN      : std_logic := '1';       -- Cursor enable (cursor off: '0', cursor on: '1')
  C_BLINK_EN       : std_logic := '1';       -- Cursor blink enable (blink off: '0', blink on: '1')
  C_INC_DEC        : std_logic := '1';       -- Increment/decrement (decrement: '0', increment: '1')
  C_SHIFT_EN       : std_logic := '1'        -- Shift enable (shift off: '0', shift on: '1')
);
port
(
  I_CLK            : in std_logic;           -- System clk frequency of (C_CLK_FREQ_MHZ)
  I_RESET_N        : in std_logic;           -- System reset (active low)

  -- User signals
  I_LCD_DATA       : in t_lcd_display_data;  -- Incoming "screen" data
  O_LCD_BUSY       : out std_logic;          -- Output busy signal for user logic

  -- Pass through external signals
  O_LCD_DATA       : out std_logic_vector(7 downto 0);
  O_LCD_ENABLE     : out std_logic;
  O_LCD_RS         : out std_logic;
  O_LCD_RW         : out std_logic;
  O_LCD_ON         : out std_logic;

  -- CDL=> Not needed for DE2 since backlight hardware is not setup, but included anyways
  O_LCD_BLON       : out std_logic
);
end entity lcd_display_driver;

--------------------------------
--  Architecture Declaration  --
--------------------------------
architecture behavioral of lcd_display_driver is

  ----------------
  -- Components --
  ----------------
  component lcd_controller IS
  generic
  (
    clk_freq       :  integer    := 50;                  -- System clock frequency in MHz
    display_lines  :  std_logic  := '1';                 -- Number of display lines (0 = 1-line mode, 1 = 2-line mode)
    character_font :  std_logic  := '0';                 -- Font (0 = 5x8 dots, 1 = 5x10 dots)
    display_on_off :  std_logic  := '1';                 -- Display on/off (0 = off, 1 = on)
    cursor         :  std_logic  := '0';                 -- Cursor on/off (0 = off, 1 = on)
    blink          :  std_logic  := '0';                 -- Blink on/off (0 = off, 1 = on)
    inc_dec        :  std_logic  := '1';                 -- Increment/decrement (0 = decrement, 1 = increment)
    shift          :  std_logic  := '0'                  -- Shift on/off (0 = off, 1 = on)
  );
  port
  (
    clk            : in   std_logic;                     -- System clock
    reset_n        : in   std_logic;                     -- Active low reinitializes lcd
    lcd_enable     : in   std_logic;                     -- Latches data into lcd controller
    lcd_bus        : in   std_logic_vector(9 downto 0);  -- Data and control signals
    busy           : out  std_logic := '1';              -- Lcd controller busy/idle feedback
    rw, rs, e      : out  std_logic;                     -- Read/write, setup/data, and enable for lcd
    lcd_data       : out  std_logic_vector(7 downto 0)   -- Data signals for lcd
  );
  end component lcd_controller;

  ---------------
  -- Constants --
  ---------------
  constant C_LCD_INDEX_CNTR_MAX : integer := 33;
  constant C_LCD_ADDRESS_0X00   : std_logic_vector(6 downto 0) :="0000000";  -- Address 0 (start of line 1)
  constant C_LCD_ADDRESS_0X40   : std_logic_vector(6 downto 0) :="1000000";  -- Address 40 (start of line 2)

  -------------
  -- SIGNALS --
  -------------

  -- Data latching signals
  signal s_lcd_data_curr    : t_lcd_display_data;
  signal s_lcd_data_prev    : t_lcd_display_data;
  signal s_lcd_data_latched : t_lcd_display_data;
  signal s_lcd_trigger      : std_logic;

  -- State machine related signals
  type T_LCD_STATE is (READY_STATE, WRITE_STATE, WAIT_STATE, NEXT_STATE);
  signal s_lcd_curr_state       : T_LCD_STATE := READY_STATE;

  signal s_lcd_write_byte   : std_logic_vector(7 downto 0);
  signal s_lcd_rs           : std_logic;
  signal s_lcd_enable       : std_logic;
  signal s_lcd_busy         : std_logic;

  signal s_lcd_index_cntr   : integer := 0;  -- Counter for LCD index (0-33)
  signal s_lcd_write_mode   : std_logic;     -- Data: '0', Address: '1'

  signal s_lcd_rw           : std_logic;

begin

  -- LCD controller for 16x2 LCD display
  LCD_CONTROLLER_INST:lcd_controller
  generic map
  (
    clk_freq       => C_CLK_FREQ_MHZ,
    display_lines  => C_NUM_DISP_LINES,
    character_font => C_CHAR_FONT,
    display_on_off => C_DISP_EN,
    cursor         => C_CURSOR_EN,
    blink          => C_BLINK_EN,
    inc_dec        => C_INC_DEC,
    shift          => C_SHIFT_EN
  )
  port map
  (
    clk            => I_CLK,
    reset_n        => I_RESET_N,
    lcd_enable     => s_lcd_enable,
    lcd_bus        => s_lcd_rs & s_lcd_rw & s_lcd_write_byte,
    busy           => s_lcd_busy,
    rw             => O_LCD_RW,
    rs             => O_LCD_RS,
    e              => O_LCD_ENABLE,
    lcd_data       => O_LCD_DATA
  );

  ------------------------------------------------------------------------------
  -- Process Name     : LCD_DATA_LATCH
  -- Sensitivity List : I_CLK               : System clock
  --                    I_RESET_N           : System reset (active low logic)
  -- Useful Outputs   : s_lcd_data_latched  : New data to write to display.
  --                    s_lcd_trigger       : Trigger to start a new data write.
  -- Description      : Process to detect change in input data, latch the
  --                    incoming data, and trigger a write process.
  ------------------------------------------------------------------------------
  LCD_DATA_LATCH: process (I_CLK, I_RESET_N)
  begin
    if (I_RESET_N = '0') then
      s_lcd_data_curr      <= (others=>(others=>('0')));
      s_lcd_data_prev      <= (others=>(others=>('0')));
      s_lcd_data_latched   <= (others=>(others=>('0')));
      s_lcd_trigger        <= '0';

    elsif (rising_edge(I_CLK)) then
      -- Register input and previous inputs
      s_lcd_data_curr      <= I_LCD_DATA;
      s_lcd_data_prev      <= s_lcd_data_curr;

      -- Detect change in input (Not busy and data has changed)
      if ((s_lcd_curr_state <= READY_STATE) and
          (s_lcd_data_curr /= s_lcd_data_prev)) then
        s_lcd_data_latched <= s_lcd_data_curr;
        s_lcd_trigger      <= '1';
      else
        s_lcd_data_latched <= s_lcd_data_latched;
        s_lcd_trigger      <= '0';
      end if;
    end if;
  end process LCD_DATA_LATCH;
  ------------------------------------------------------------------------------

  ------------------------------------------------------------------------------
  -- Process Name     : LCD_STATE_MACHINE
  -- Sensitivity List : I_CLK            : System clock
  --                    I_RESET_N        : System reset (active low logic)
  -- Useful Outputs   : s_lcd_curr_state : Current state of LCD write process.
  -- Description      : State machine to control different states for LCD module
  ------------------------------------------------------------------------------
  LCD_STATE_MACHINE: process (I_CLK, I_RESET_N)
  begin
    if (I_RESET_N = '0') then
      s_lcd_curr_state             <= READY_STATE;

    elsif (rising_edge(I_CLK)) then
        -- I2C 7SD state machine logic
        case s_lcd_curr_state is
          when READY_STATE =>
            if (s_lcd_trigger = '1') then
              s_lcd_curr_state     <= WRITE_STATE;
            else
              s_lcd_curr_state     <= s_lcd_curr_state;
            end if;

          when WRITE_STATE =>
            s_lcd_curr_state       <= WAIT_STATE;

          when WAIT_STATE =>
            if (s_lcd_busy = '1') then
              s_lcd_curr_state     <= NEXT_STATE;
            else
              s_lcd_curr_state     <= s_lcd_curr_state;
            end if;

            when NEXT_STATE =>
              if (s_lcd_busy = '0') then
                if (s_lcd_index_cntr /= C_LCD_INDEX_CNTR_MAX) then
                  s_lcd_curr_state <= WRITE_STATE;
                else
                  s_lcd_curr_state <= READY_STATE;
                end if;
              else
                s_lcd_curr_state   <= s_lcd_curr_state;
              end if;

          -- Error condition, should never occur
          when others =>
            s_lcd_curr_state       <= READY_STATE;
        end case;
    end if;
  end process LCD_STATE_MACHINE;
  ------------------------------------------------------------------------------

  ------------------------------------------------------------------------------
  -- Process Name     : DATA_FLOW_CTRL
  -- Sensitivity List : I_CLK            : System clock
  --                    I_RESET_N        : System reset (active low logic)
  -- Useful Outputs   : O_LCD_ON         : Output LCD enabled/disabled signal.
  --                    s_lcd_enable     : Enable signal for LCD controller.
  --                    s_lcd_index_cntr : Current index of data counter.
  --                    s_lcd_write_byte : Current byte to write to the LCD.
  --                    s_lcd_rs         : Register select to LCD.
  --                    O_LCD_BUSY       : Output busy signal.
  -- Description        This process is responsible for controlling data to and
  --                    from the LCD module based on the current state:
  --                    (s_lcd_curr_state).
  ------------------------------------------------------------------------------
  DATA_FLOW_CTRL: process (I_CLK, I_RESET_N)
  begin
    if (I_RESET_N = '0') then
      O_LCD_ON             <= '0';
      s_lcd_enable         <= '0';
      s_lcd_index_cntr     <=  0;
      s_lcd_write_byte     <= (others=>'0');
      s_lcd_rs             <= '0';
      O_LCD_BUSY           <= '1';

    elsif (rising_edge(I_CLK)) then

      -- LCD Enable/Disable logic
      O_LCD_ON             <= '1';

      -- Enable signal logic
      if (s_lcd_curr_state = WRITE_STATE) then
        s_lcd_enable       <= '1';
      elsif (s_lcd_curr_state = WAIT_STATE) and
            (s_lcd_busy = '1') then
        s_lcd_enable       <= '0';
      else
        s_lcd_enable       <= s_lcd_enable;
      end if;

      -- Data or Address selection logic
      if (s_lcd_curr_state = NEXT_STATE) and (s_lcd_busy = '0') then
        if ((s_lcd_index_cntr = 17) and (s_lcd_write_mode = '0')) then
          s_lcd_write_mode <= '1';
        else
          s_lcd_write_mode <= '0';
        end if;
      else
        s_lcd_write_mode   <= s_lcd_write_mode;
      end if;

      -- Data Byte Index logic
      if (s_lcd_curr_state = NEXT_STATE) and (s_lcd_busy = '0') then
        if ((s_lcd_index_cntr = 17) and (s_lcd_write_mode = '0')) then
          s_lcd_index_cntr <= s_lcd_index_cntr;
        elsif (s_lcd_index_cntr /= C_LCD_INDEX_CNTR_MAX) then
          s_lcd_index_cntr <= s_lcd_index_cntr + 1;
        else
          s_lcd_index_cntr <= 0;
        end if;
      else
        s_lcd_index_cntr   <= s_lcd_index_cntr;
      end if;

      -- Data byte logic
      if (s_lcd_index_cntr = 0) then
			  s_lcd_write_byte   <= "1" & C_LCD_ADDRESS_0X00; -- Go to first line
			  s_lcd_rs           <= '0';
		  elsif (s_lcd_write_mode = '0') then  -- Data
        s_lcd_write_byte   <= s_lcd_data_latched(C_LCD_INDEX_CNTR_MAX - s_lcd_index_cntr);
        s_lcd_rs           <= '1';
      else  -- Address
        s_lcd_write_byte   <= "1" & C_LCD_ADDRESS_0X40; -- Go to next line
        s_lcd_rs           <= '0';
      end if;

      -- Output Busy logic
      if (s_lcd_curr_state = READY_STATE) then
        O_LCD_BUSY         <= '0';
      else
        O_LCD_BUSY         <= '1';
      end if;
    end if;
  end process DATA_FLOW_CTRL;
  ------------------------------------------------------------------------------

  s_lcd_rw   <= '0';  -- Always writing
  O_LCD_BLON <= '1';  -- Backlight always on

end architecture behavioral;
