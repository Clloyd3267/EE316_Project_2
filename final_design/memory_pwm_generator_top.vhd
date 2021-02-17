--------------------------------------------------------------------------------
-- Filename     : memory_pwm_generator_top.vhd
-- Author(s)    : Chris Lloyd, Camilla Ketola, Josiah Schmidt
-- Class        : EE316 (Project 2)
-- Due Date     : 2021-02-23
-- Target Board : Altera DE2 Devkit
-- Entity       : memory_pwm_generator_top
-- Description  : Multi Mode memory control system which loads default data from
--                ROM and stores it in SRAM. This data is then used to generate
--                a Pulse Width Modulation (PWM) sine wave at a certain frequency.
--------------------------------------------------------------------------------

-----------------
--  Libraries  --  -- Label purpose of utils
-----------------
library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.lcd_screen_util.all;

library work;
  use work.edge_detector_utilities.all;

--------------
--  Entity  --
--------------
entity memory_pwm_generator_top is
port
(
  I_CLK_50_MHZ : in std_logic;     -- System clk (50 MHZ)
  I_RESET_N    : in std_logic;     -- System reset (active low)

  I_KEY_1_N    : in std_logic;     -- Key inputs used to control the state machine
  I_KEY_2_N    : in std_logic;     -- Key inputs used to control the state machine
  I_KEY_3_N    : in std_logic;     -- Key inputs used to control the state machine

  -- I2C passthrough signals
  IO_I2C_SDA   : inout std_logic;  -- Serial data of i2c bus
  IO_I2C_SCL   : inout std_logic;  -- Serial clock of i2c bus

  -- LCD passthrough signals
  O_LCD_DATA   : out std_logic_vector(7 downto 0);
  O_LCD_ENABLE : out std_logic;
  O_LCD_RS     : out std_logic;
  O_LCD_RW     : out std_logic;
  O_LCD_ON     : out std_logic;
  O_LCD_BLON   : out std_logic;

  -- SRAM passthrough signals
  IO_SRAM_DATA : inout std_logic_vector(15 downto 0);
  O_SRAM_ADDR  : out std_logic_vector(17 downto 0);
  O_SRAM_WE_N  : out std_logic;
  O_SRAM_OE_N  : out std_logic;
  O_SRAM_UB_N  : out std_logic;
  O_SRAM_LB_N  : out std_logic;
  O_SRAM_CE_N  : out std_logic
);
end entity memory_pwm_generator_top;

--------------------------------
--  Architecture Declaration  --
--------------------------------
architecture behavioral of memory_pwm_generator_top is

  ----------------
  -- Components --
  ----------------
  component i2c_7sd_driver is
  generic
  (
    C_CLK_FREQ_MHZ : integer := 50                      -- System clock frequency in MHz
  );
  port
  (
    I_CLK          : in std_logic;                      -- System clk frequency of (C_CLK_FREQ_MHZ)
    I_RESET_N      : in std_logic;                      -- System reset (active low)
    I_DISPLAY_DATA : in std_logic_vector(15 downto 0);  -- Data to be displayed
    O_BUSY         : out std_logic;                     -- Busy signal from I2C master
    IO_I2C_SDA     : inout std_logic;                   -- Serial data of i2c bus
    IO_I2C_SCL     : inout std_logic                    -- Serial clock of i2c bus
  );
  end component i2c_7sd_driver;

  component lcd_display_driver is
  generic
  (
    C_CLK_FREQ_MHZ   : integer := 50;      -- System clock frequency in MHz

    -- LCD Specific Settings
    C_NUM_DISP_LINES : std_logic := '1';   -- Number of lines to display (1-line mode: '0', 2-line mode: '1')
    C_CHAR_FONT      : std_logic := '1';   -- Character font (5x8 dot: '0', 5x10 dots: '1')
    C_DISP_EN        : std_logic := '1';   -- Display enable (display off: '0', display on: '1')
    C_CURSOR_EN      : std_logic := '1';   -- Cursor enable (cursor off: '0', cursor on: '1')
    C_BLINK_EN       : std_logic := '1';   -- Cursor blink enable (blink off: '0', blink on: '1')
    C_INC_DEC        : std_logic := '1';   -- Increment/decrement (decrement: '0', increment: '1')
    C_SHIFT_EN       : std_logic := '1'    -- Shift enable (shift off: '0', shift on: '1')
  );
  port
  (
    I_CLK            : in std_logic;                      -- System clk frequency of (C_CLK_FREQ_MHZ)
    I_RESET_N        : in std_logic;                      -- System reset (active low)

    -- User signals
    I_LCD_DATA       : in t_lcd_display_data;
    O_LCD_BUSY       : out std_logic;

    O_LCD_ON         : out std_logic;
    O_LCD_BLON       : out std_logic;

    -- Pass through external signals
    O_LCD_DATA       : out std_logic_vector(7 downto 0);
    O_LCD_ENABLE     : out std_logic;
    O_LCD_RS         : out std_logic;
    O_LCD_RW         : out std_logic
  );
  end component lcd_display_driver;

  component lcd_lut is
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
  end component lcd_lut;

  component debounce_button is
  generic
  (
    C_CLK_FREQ_MHZ   : integer;       -- System clock frequency in MHz
    C_STABLE_TIME_MS : integer        -- Time required for button to remain stable in ms
  );
  port
  (
    I_CLK            : in std_logic;  -- System clk frequency of (C_CLK_FREQ_MHZ)
    I_RESET_N        : in std_logic;  -- System reset (active low)
    I_BUTTON         : in std_logic;  -- Button data to be debounced
    O_BUTTON         : out std_logic  -- Debounced button data
  );
  end component debounce_button;

  component edge_detector is
  generic
  (
    C_CLK_FREQ_MHZ   : integer;       -- System clock frequency in MHz
    C_TRIGGER_EDGE   : T_EDGE_TYPE    -- Edge to trigger on
  );
  port
  (
    I_CLK            : in std_logic;  -- System clk frequency of (C_CLK_FREQ_MHZ)
    I_RESET_N        : in std_logic;  -- System reset (active low)
    I_SIGNAL         : in std_logic;  -- Input signal to pass through edge detector
    O_EDGE_SIGNAL    : out std_logic  -- Output pulse on (C_TRIGGER_EDGE) edge
  );
  end component edge_detector;

  -- CDL=> Add SRAM and ROM

  ---------------
  -- Constants --
  ---------------

  constant C_CLK_FREQ_MHZ   : integer := 50;  -- System clock frequency in MHz

  -- LCD Specific Settings
  constant C_NUM_DISP_LINES : std_logic := '1';   -- Number of lines to display (1-line mode: '0', 2-line mode: '1')
  constant C_CHAR_FONT      : std_logic := '0';   -- Character font (5x8 dot: '0', 5x10 dots: '1')
  constant C_DISP_EN        : std_logic := '1';   -- Display enable (display off: '0', display on: '1')
  constant C_CURSOR_EN      : std_logic := '0';   -- Cursor enable (cursor off: '0', cursor on: '1')
  constant C_BLINK_EN       : std_logic := '0';   -- Cursor blink enable (blink off: '0', blink on: '1')
  constant C_INC_DEC        : std_logic := '1';   -- Increment/decrement (decrement: '0', increment: '1')
  constant C_SHIFT_EN       : std_logic := '0';   -- Shift enable (shift off: '0', shift on: '1')

  -- Mode of Operation
  -- 00: Initialization
  -- 01: Test
  -- 10: Pause
  -- 11: PWM Generation
  constant C_INIT_MODE  : std_logic_vector(1 downto 0) := "00";
  constant C_TEST_MODE  : std_logic_vector(1 downto 0) := "01";
  constant C_PAUSE_MODE : std_logic_vector(1 downto 0) := "10";
  constant C_PWM_MODE   : std_logic_vector(1 downto 0) := "11";

  -- Output Frequency
  -- 00: 60 Hz
  -- 01: 120 Hz
  -- 10: 1000 Hz (1KHz)
  -- 11: Undefined (pick one)
  constant C_60_HZ  : std_logic_vector(1 downto 0) := "00";
  constant C_120_HZ : std_logic_vector(1 downto 0) := "01";
  constant C_1_KHZ  : std_logic_vector(1 downto 0) := "10";

  -- Inital System reset time in ms
  constant C_RESET_TIME_MS : integer := 25;

  -- Max address of SRAM and ROM (255)
  constant C_MAX_ADDRESS   : unsigned(7 downto 0) := to_unsigned(255, 8);

  constant C_STABLE_TIME_MS : integer     := 10;      -- Time required for button to remain stable in ms
  constant C_TRIGGER_EDGE   : T_EDGE_TYPE := RISING;  -- Edge to trigger on

  -------------
  -- SIGNALS --
  -------------

  signal s_lcd_data        : t_lcd_display_data;             -- Data to be displayed on LCD
  signal s_lcd_lut_data    : t_lcd_display_data;             -- Data from LCD LUT
  signal s_lcd_busy        : std_logic;                      -- Busy signal from LCD

  signal s_i2c_busy        : std_logic;                      -- Busy signal from I2C

  signal s_curr_mode       : std_logic_vector(1 downto 0);
  signal s_prev_mode       : std_logic_vector(1 downto 0);
  signal s_curr_pwm_freq   : std_logic_vector(1 downto 0);

  signal s_curr_address    : std_logic_vector(7 downto 0)  := x"FF";
  signal s_curr_data       : std_logic_vector(15 downto 0) := x"FFFF";

  -- System reset control signal
  signal s_reset_n              : std_logic := '0';

  -- System reset control signal from counter
  signal s_cntr_reset_n         : std_logic := '0';

  -- Signal to toggle (increment/decrement) address
  signal s_address_toggle       : std_logic := '0';

  -- Current counter address
  signal s_current_address      : unsigned(17 downto 0) := (others=>'0');

  signal s_key_0_debounced : std_logic;
  signal s_key_1_debounced : std_logic;
  signal s_key_2_debounced : std_logic;
  signal s_key_3_debounced : std_logic;

  signal s_key_1           : std_logic;
  signal s_key_2           : std_logic;
  signal s_key_3           : std_logic;

begin

  -- User logic display driver for LCD
  LCD_DISPLAY_DRIVER_INST: lcd_display_driver
  generic map
  (
    C_CLK_FREQ_MHZ   => C_CLK_FREQ_MHZ,
    C_NUM_DISP_LINES => C_NUM_DISP_LINES,
    C_CHAR_FONT      => C_CHAR_FONT,
    C_DISP_EN        => C_DISP_EN,
    C_CURSOR_EN      => C_CURSOR_EN,
    C_BLINK_EN       => C_BLINK_EN,
    C_INC_DEC        => C_INC_DEC,
    C_SHIFT_EN       => C_SHIFT_EN
  )
  port map
  (
    I_CLK            => I_CLK_50_MHZ,
    I_RESET_N        => s_reset_n,
    I_LCD_DATA       => s_lcd_data,
    O_LCD_BUSY       => s_lcd_busy,
    O_LCD_ON         => O_LCD_ON,
    O_LCD_BLON       => O_LCD_BLON,
    O_LCD_DATA       => O_LCD_DATA,
    O_LCD_ENABLE     => O_LCD_ENABLE,
    O_LCD_RS         => O_LCD_RS,
    O_LCD_RW         => O_LCD_RW
  );

  -- User logic lookup table (LUT) for LCD
  LCD_LUT_INST: lcd_lut
  generic map
  (
    C_CLK_FREQ_MHZ => C_CLK_FREQ_MHZ
  )
  port map
  (
    I_CLK          => I_CLK_50_MHZ,
    I_RESET_N      => s_reset_n,
    I_MODE         => s_curr_mode, -- s_curr_mode / C_PAUSE_MODE
    I_PWM_FREQ     => s_curr_pwm_freq,
    I_DATA         => s_curr_data,
    I_ADDRESS      => s_curr_address,
    O_LCD_DATA     => s_lcd_lut_data
  );

  -- Device driver for 7SD
  I2C_DISPLAY_DRIVER_INST: i2c_7sd_driver
  generic map
  (
    C_CLK_FREQ_MHZ => C_CLK_FREQ_MHZ
  )
  port map
  (
    I_CLK          => I_CLK_50_MHZ,
    I_RESET_N      => s_reset_n,

    I_DISPLAY_DATA => s_curr_data,
    O_BUSY         => s_i2c_busy,
    IO_I2C_SDA     => IO_I2C_SDA,
    IO_I2C_SCL     => IO_I2C_SCL
  );

  KEY0_DEBOUNCE_INST: debounce_button -------------------------------------
  generic map
  (
    C_CLK_FREQ_MHZ   => C_CLK_FREQ_MHZ,
    C_STABLE_TIME_MS => C_STABLE_TIME_MS
  )
  port map
  (
    I_CLK            => I_CLK_50_MHZ,
    I_RESET_N        => s_reset_n,
    I_BUTTON         => not I_RESET_N,
    O_BUTTON         => s_key_0_debounced
  );

  KEY1_DEBOUNCE_INST: debounce_button
  generic map
  (
    C_CLK_FREQ_MHZ   => C_CLK_FREQ_MHZ,
    C_STABLE_TIME_MS => C_STABLE_TIME_MS
  )
  port map
  (
    I_CLK            => I_CLK_50_MHZ,
    I_RESET_N        => s_reset_n,
    I_BUTTON         => not I_KEY_1_N,
    O_BUTTON         => s_key_1_debounced
  );

  KEY2_DEBOUNCE_INST: debounce_button
  generic map
  (
    C_CLK_FREQ_MHZ   => C_CLK_FREQ_MHZ,
    C_STABLE_TIME_MS => C_STABLE_TIME_MS
  )
  port map
  (
    I_CLK            => I_CLK_50_MHZ,
    I_RESET_N        => s_reset_n,
    I_BUTTON         => not I_KEY_2_N,
    O_BUTTON         => s_key_2_debounced
  );

  KEY3_DEBOUNCE_INST: debounce_button
  generic map
  (
    C_CLK_FREQ_MHZ   => C_CLK_FREQ_MHZ,
    C_STABLE_TIME_MS => C_STABLE_TIME_MS
  )
  port map
  (
    I_CLK            => I_CLK_50_MHZ,
    I_RESET_N        => s_reset_n,
    I_BUTTON         => not I_KEY_3_N,
    O_BUTTON         => s_key_3_debounced
  );

  KEY1_EDGE_DETEC_INST: edge_detector
  generic map
  (
    C_CLK_FREQ_MHZ => C_CLK_FREQ_MHZ,
    C_TRIGGER_EDGE => C_TRIGGER_EDGE
  )
  port map
  (
    I_CLK          => I_CLK_50_MHZ,
    I_RESET_N      => s_reset_n,
    I_SIGNAL       => s_key_1_debounced,
    O_EDGE_SIGNAL  => s_key_1
  );

  KEY2_EDGE_DETEC_INST: edge_detector
  generic map
  (
    C_CLK_FREQ_MHZ => C_CLK_FREQ_MHZ,
    C_TRIGGER_EDGE => C_TRIGGER_EDGE
  )
  port map
  (
    I_CLK          => I_CLK_50_MHZ,
    I_RESET_N      => s_reset_n,
    I_SIGNAL       => s_key_2_debounced,
    O_EDGE_SIGNAL  => s_key_2
  );

  KEY3_EDGE_DETEC_INST: edge_detector
  generic map
  (
    C_CLK_FREQ_MHZ => C_CLK_FREQ_MHZ,
    C_TRIGGER_EDGE => C_TRIGGER_EDGE
  )
  port map
  (
    I_CLK          => I_CLK_50_MHZ,
    I_RESET_N      => s_reset_n,
    I_SIGNAL       => s_key_3_debounced,
    O_EDGE_SIGNAL  => s_key_3
  );
  ---------------
  -- Processes --
  ---------------

  ------------------------------------------------------------------------------
  -- Process Name     : SYSTEM_RST_OUTPUT
  -- Sensitivity List : I_CLK_50_MHZ   : System clock
  -- Useful Outputs   : s_cntr_reset_n : Reset signal from counter (active low)
  -- Description      : System FW Reset Output logic (active low reset logic).
  --                    Holding design in reset for (C_RESET_TIME_MS) ms
  ------------------------------------------------------------------------------
  SYSTEM_RST_OUTPUT: process (I_CLK_50_MHZ)
    variable C_RST_MS_DURATION : integer := C_CLK_FREQ_MHZ * C_RESET_TIME_MS * 1000;
    variable v_reset_cntr      : integer range 0 TO C_RST_MS_DURATION := 0;
  begin
    if (rising_edge(I_CLK_50_MHZ)) then
      if (v_reset_cntr = C_RST_MS_DURATION) then
        v_reset_cntr    := v_reset_cntr;
        s_cntr_reset_n  <= '1';
      else
        v_reset_cntr    := v_reset_cntr + 1;
        s_cntr_reset_n  <= '0';
      end if;
    end if;
  end process SYSTEM_RST_OUTPUT;
  ------------------------------------------------------------------------------

  ------------------------------------------------------------------------------
  -- Process Name     : ADDRESS_TOGGLE_COUNTER
  -- Sensitivity List : I_CLK_50_MHZ     : System clock
  --                    s_reset_n        : System reset (active low logic)
  -- Useful Outputs   : s_address_toggle : Pulsed signal to toggle address
  -- Description      : Counter to delay changing address at a rate of either
  --                    255Hz (INIT_STATE) or 1Hz (OP_STATE). -- CDL=> Add/Fix freqs
  ------------------------------------------------------------------------------
  ADDRESS_TOGGLE_COUNTER: process (I_CLK_50_MHZ, s_reset_n)
    constant C_1HZ_MAX_COUNT       : integer := C_CLK_FREQ_MHZ * 1000000;  -- 1 Hz
    constant C_255HZ_MAX_COUNT     : integer := C_CLK_FREQ_MHZ * 4000;     -- 255 Hz
    variable v_address_toggle_cntr : integer range 0 TO C_1HZ_MAX_COUNT := 0;
  begin
    if (s_reset_n = '0') then
      v_address_toggle_cntr     :=  0;
      s_address_toggle          <= '0';

    elsif (rising_edge(I_CLK_50_MHZ)) then

      -- Address index output logic (Create toggle pulse on max count)
      if (s_curr_mode = C_INIT_MODE and v_address_toggle_cntr = C_255HZ_MAX_COUNT) then
        s_address_toggle      <= '1';
      else
        s_address_toggle      <= '0';
      end if;

      -- Counter and rollover logic
      if (s_curr_mode = C_INIT_MODE and v_address_toggle_cntr = C_255HZ_MAX_COUNT) then
        v_address_toggle_cntr := 0;
      else
        v_address_toggle_cntr := v_address_toggle_cntr + 1;
      end if;
    end if;
  end process ADDRESS_TOGGLE_COUNTER;
  ------------------------------------------------------------------------------

  ------------------------------------------------------------------------------
  -- Process Name     : ADDRESS_INDEX_COUNTER
  -- Sensitivity List : I_CLK_50_MHZ      : System clock
  --                    s_reset_n         : System reset (active low logic)
  -- Useful Outputs   : s_current_address : Current address of counter
  -- Description      : A process to increment address depending on mode. -- CDL=> here
  ------------------------------------------------------------------------------
  ADDRESS_INDEX_COUNTER: process (I_CLK_50_MHZ, s_reset_n)
  begin
    if (s_reset_n = '0') then
      s_current_address   <= (others=>'1');

    elsif (rising_edge(I_CLK_50_MHZ)) then

      -- Increment address when toggle signal occurs
      if (s_address_toggle = '1') and (s_current_address(7 downto 0) = C_MAX_ADDRESS) then
        s_current_address <= (others=>'0');
      else
        s_current_address <= s_current_address + 1;
      end if;
    end if;
  end process ADDRESS_INDEX_COUNTER;
  ------------------------------------------------------------------------------

  ------------------------------------------------------------------------------
  -- Process Name     : MODE_STATE_MACHINE
  -- Sensitivity List : I_CLK_50_MHZ : System clock
  --                    s_reset_n    : System reset (active low logic)
  -- Useful Outputs   : s_curr_mode  : Current mode of the system
  --                    s_prev_mode  : Mode of system last clock edge
  -- Description      : State machine to control different modes for
  --                    initialization, and operation of system. -- CDL=> Redo logic -- Explain states
  ------------------------------------------------------------------------------
  MODE_STATE_MACHINE: process (I_CLK_50_MHZ, s_reset_n)
  begin
    if (s_reset_n = '0') then
      s_curr_mode <= C_INIT_MODE;
      s_prev_mode <= C_INIT_MODE;

    elsif (rising_edge(I_CLK_50_MHZ)) then

      -- Initialization mode
      if (s_curr_mode = C_INIT_MODE) then
        -- Wait for ROM data to be loaded into SRAM
--        if (s_current_address = C_MAX_ADDRESS) and
--           (s_address_toggle = '1') then
--          s_curr_mode <= C_TEST_MODE;
--        else
--          s_curr_mode <= s_curr_mode;
--        end if;
		  s_curr_mode <= C_TEST_MODE;

      -- Test mode
      elsif (s_curr_mode = C_TEST_MODE) then
        if (s_key_1 = '1') then
          s_curr_mode <= C_PAUSE_MODE;
        elsif (s_key_2 = '1') then
          s_curr_mode <= C_PWM_MODE;
        else
          s_curr_mode <= s_curr_mode;
        end if;

      -- Pause mode
      elsif (s_curr_mode = C_PAUSE_MODE) then
        if (s_key_1 = '1') then
          s_curr_mode <= C_TEST_MODE;
        elsif (s_key_2 = '1') then
          s_curr_mode <= C_PWM_MODE;
        else
          s_curr_mode <= s_curr_mode;
        end if;

      -- PWM Generation mode
      elsif (s_curr_mode = C_PWM_MODE) then
        if (s_key_2 = '1') then
          s_curr_mode <= C_TEST_MODE;
        else
          s_curr_mode <= s_curr_mode;
        end if;

      -- Error condition, should never occur
      else
        s_curr_mode   <= C_INIT_MODE;
      end if;

      -- Store previous mode for use in detecting mode "changes"
      s_prev_mode    <= s_curr_mode;
    end if;
  end process MODE_STATE_MACHINE;
  ------------------------------------------------------------------------------

  ------------------------------------------------------------------------------
  -- Process Name     : FREQ_STATE_MACHINE
  -- Sensitivity List : I_CLK_50_MHZ : System clock
  --                    s_reset_n    : System reset (active low logic)
  -- Useful Outputs   : s_curr_mode  : Current freq of the system
  -- Description      : State machine to control different freqencies of system.
  ------------------------------------------------------------------------------
  FREQ_STATE_MACHINE: process (I_CLK_50_MHZ, s_reset_n)
  begin
    if (s_reset_n = '0') then
      s_curr_pwm_freq         <= C_60_HZ;

    elsif (rising_edge(I_CLK_50_MHZ)) then
      if (s_curr_mode = C_PWM_MODE) then
        if (s_key_3 = '1') then
          case (s_curr_pwm_freq) is
            when C_60_HZ  =>
              s_curr_pwm_freq <= C_120_HZ;
            when C_120_HZ =>
              s_curr_pwm_freq <= C_1_KHZ;
            when C_1_KHZ  =>
              s_curr_pwm_freq <= C_60_HZ;
            when others   =>
              s_curr_pwm_freq <= C_60_HZ;
          end case;
        else
          s_curr_pwm_freq <= s_curr_pwm_freq;
        end if;
      else
        s_curr_pwm_freq <= C_60_HZ;
      end if;
    end if;
  end process FREQ_STATE_MACHINE;
  ------------------------------------------------------------------------------
  
  process (I_CLK_50_MHZ, s_reset_n) -- CDl=> Here
  begin
    if (s_reset_n = '0') then
      s_lcd_data <= (others=>(others=>'0'));

    elsif (rising_edge(I_CLK_50_MHZ)) then
		if (s_lcd_busy = '0') then
			s_lcd_data <= s_lcd_lut_data;
		end if;
    end if;
  end process;
  ------------------------------------------------------------------------------  
  
  -- Reset system (low) when counter reset signal is low or system reset is low
  s_reset_n <= s_cntr_reset_n and (not s_key_0_debounced);

end architecture behavioral;
