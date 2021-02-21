--------------------------------------------------------------------------------
-- Filename     : i2c_7sd_driver.vhd
-- Author(s)    : Chris Lloyd, Camilla Ketola, Josiah Schmidt
-- Class        : EE316 (Project 2)
-- Due Date     : 2021-02-23
-- Target Board : Altera DE2 Devkit
-- Entity       : i2c_7sd_driver
-- Description  : This I2C Seven Segment Display (7SD) Driver takes in a 16 bit
--                number to display to an external SparkFun I2C Display.
--                https://www.digikey.com/eewiki/pages/viewpage.action?pageId=10125324
--------------------------------------------------------------------------------

-----------------
--  Libraries  --
-----------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--------------
--  Entity  --
--------------
entity i2c_7sd_driver is
generic
(
  C_CLK_FREQ_MHZ : integer := 50                      -- System clock frequency in MHz
);
port
(
  I_CLK          : in std_logic;                      -- System clk frequency of (C_CLK_FREQ_MHZ)
  I_RESET_N      : in std_logic;                      -- System reset (active low)

  I_DISP_ENABLE  : in std_logic;                      -- Whether the screen is on '1' or off '0'

  I_DISPLAY_DATA : in std_logic_vector(15 downto 0);  -- Data to be displayed
  O_BUSY         : out std_logic;                     -- Busy signal from I2C master

  IO_I2C_SDA     : inout std_logic;                   -- Serial data of i2c bus
  IO_I2C_SCL     : inout std_logic                    -- Serial clock of i2c bus
);
end entity i2c_7sd_driver;

--------------------------------
--  Architecture Declaration  --
--------------------------------
architecture behavioral of i2c_7sd_driver is

  ----------------
  -- Components --
  ----------------
  component i2c_master is
  generic
  (
    input_clk : integer := 50_000_000;               -- Input clock speed from user logic in Hz
    bus_clk   : integer := 400_000                   -- Speed the i2c bus (scl) will run at in Hz
  );
  port
  (
    clk       : in     std_logic;                    -- System clock
    reset_n   : in     std_logic;                    -- Active low reset
    ena       : in     std_logic;                    -- Latch in command
    addr      : in     std_logic_vector(6 downto 0); -- Address of target slave
    rw        : in     std_logic;                    -- '0' is write, '1' is read
    data_wr   : in     std_logic_vector(7 downto 0); -- Data to write to slave
    busy      : out    std_logic;                    -- Indicates transaction in progress
    data_rd   : out    std_logic_vector(7 downto 0); -- Data read from slave
    ack_error : buffer std_logic;                    -- Flag if improper acknowledge from slave
    sda       : inout  std_logic;                    -- Serial data output of i2c bus
    scl       : inout  std_logic                     -- Serial clock output of i2c bus
  );
  end component i2c_master;

  ---------------
  -- Constants --
  ---------------
  constant C_CLK_FREQ_HZ         : integer := C_CLK_FREQ_MHZ * 1_000_000;
  constant C_I2C_BUS_CLK_FREQ_HZ : integer := 100_000;
  constant C_I2C_7SD_ADDR        : std_logic_vector(6 downto 0) := "1110001";  -- Default address is 0x71
  constant C_WR_BYTE_INDEX_MAX   : integer := 12;
  constant C_WR_BYTE_READY_INDEX : integer := 7;

  constant C_7SD_CLEAR_DISP_CMD  : std_logic_vector(7 downto 0) := x"76";
  constant C_7SD_BRIGHTNESS_CMD  : std_logic_vector(7 downto 0) := x"7A";
  constant C_7SD_DEC_CTRL_CMD    : std_logic_vector(7 downto 0) := x"77";
  constant C_7SD_CURS_CTRL_CMD   : std_logic_vector(7 downto 0) := x"79";

  -------------
  -- SIGNALS --
  -------------

  -- State machine related signals
  type T_7SD_STATE is (READY_STATE, WRITE_STATE, WAIT_STATE, NEXT_STATE);
  signal s_7sd_curr_state       : T_7SD_STATE := READY_STATE;

  signal s_display_data_latched : std_logic_vector(15 downto 0);
  signal s_7sd_enable           : std_logic;
  signal s_wr_data_byte_index   : integer;

  signal s_wr_data_byte         : std_logic_vector(7 downto 0);

  signal s_7sd_wr               : std_logic;  --'0' is write, '1' is read
  signal s_7sd_address          : std_logic_vector(6 downto 0) := C_I2C_7SD_ADDR;
  signal s_7sd_busy             : std_logic;

begin

  -- I2C controller for 4 digit Seven Segment Display
  I2C_MASTER_INST:i2c_master
  generic map
  (
    input_clk => C_CLK_FREQ_HZ,
    bus_clk   => C_I2C_BUS_CLK_FREQ_HZ
  )
  port map
  (
    clk       => I_CLK,
    reset_n   => I_RESET_N,
    ena       => s_7sd_enable,
    addr      => s_7sd_address,
    rw        => s_7sd_wr,
    data_wr   => s_wr_data_byte,
    busy      => s_7sd_busy,
    data_rd   => open,
    ack_error => open,
    sda       => IO_I2C_SDA,
    scl       => IO_I2C_SCL
  );

  ---------------
  -- Processes --
  ---------------

  ------------------------------------------------------------------------------
  -- Process Name     : I2C_STATE_MACHINE
  -- Sensitivity List : I_CLK            : System clock
  --                    I_RESET_N        : System reset (active low logic)
  -- Useful Outputs   : s_7sd_curr_state : Current state of S7D write process.
  -- Description      : State machine to control different states for I2C
  --                    Seven Segment Display (7SD) module.
  ------------------------------------------------------------------------------
  I2C_STATE_MACHINE: process (I_CLK, I_RESET_N)
  begin
    if (I_RESET_N = '0') then
      s_7sd_curr_state             <= READY_STATE;

    elsif (rising_edge(I_CLK)) then
        -- I2C 7SD state machine logic
        case s_7sd_curr_state is
          when READY_STATE =>
            if (s_display_data_latched /= I_DISPLAY_DATA) then
              s_7sd_curr_state     <= WRITE_STATE;
            else
              s_7sd_curr_state     <= s_7sd_curr_state;
            end if;

          when WRITE_STATE =>
            s_7sd_curr_state       <= WAIT_STATE;

          when WAIT_STATE =>
            if (s_7sd_busy = '1') then
              s_7sd_curr_state     <= NEXT_STATE;
            else
              s_7sd_curr_state     <= s_7sd_curr_state;
            end if;

            when NEXT_STATE =>
              if (s_7sd_busy = '0') then
                if (s_wr_data_byte_index /= C_WR_BYTE_INDEX_MAX) then
                  s_7sd_curr_state <= WRITE_STATE;
                else
                  s_7sd_curr_state <= READY_STATE;
                end if;
              else
                s_7sd_curr_state   <= s_7sd_curr_state;
              end if;

          -- Error condition, should never occur
          when others =>
            s_7sd_curr_state       <= READY_STATE;
        end case;
    end if;
  end process I2C_STATE_MACHINE;
  ------------------------------------------------------------------------------

  ------------------------------------------------------------------------------
  -- Process Name     : DATA_FLOW_CTRL
  -- Sensitivity List : I_CLK                  : System clock
  --                    I_RESET_N              : System reset (active low logic)
  -- Useful Outputs   : s_display_data_latched : Latched data to remain stable.
  --                    s_7sd_enable           : Enable signal for I2C master.
  --                    s_wr_data_byte_index   : Current data byte index.
  --                    O_BUSY                 : Output busy signal.
  -- Description      : Process to increment index counter representing current
  --                    data byte to send. LUT table in (WRITE_DATA_LUT). Also
  --                    controls enable signal for I2C master, and output busy
  --                    signal.
  ------------------------------------------------------------------------------
  DATA_FLOW_CTRL: process (I_CLK, I_RESET_N)
  begin
    if (I_RESET_N = '0') then
      s_display_data_latched     <= (others=>'1');
      s_7sd_enable               <= '0';
      s_wr_data_byte_index       <=  0;
      O_BUSY                     <= '1';

    elsif (rising_edge(I_CLK)) then
      -- Latch data so it does not change during write
      if (s_7sd_curr_state = READY_STATE) then
         s_display_data_latched  <= I_DISPLAY_DATA;
      else
         s_display_data_latched  <= s_display_data_latched;
      end if;

      -- Enable signal logic
      if (s_7sd_curr_state = WRITE_STATE) then
        s_7sd_enable             <= '1';
      elsif (s_7sd_curr_state = WAIT_STATE) and
            (s_7sd_busy = '1') then
        s_7sd_enable             <= '0';
      else
        s_7sd_enable             <= s_7sd_enable;
      end if;

      -- Data Byte Index logic
      if (I_DISP_ENABLE = '0') then
        s_wr_data_byte_index <= 0;
      elsif (s_7sd_curr_state = NEXT_STATE) and (s_7sd_busy = '0') then
          if (s_wr_data_byte_index /= C_WR_BYTE_INDEX_MAX) then
            s_wr_data_byte_index <= s_wr_data_byte_index + 1;
          else
            s_wr_data_byte_index <= C_WR_BYTE_READY_INDEX;
          end if;
      else
        s_wr_data_byte_index     <= s_wr_data_byte_index;
      end if;

      -- Output Busy logic
      if (s_7sd_curr_state = READY_STATE) then
        O_BUSY                   <= '0';
      else
        O_BUSY                   <= '1';
      end if;
    end if;
  end process DATA_FLOW_CTRL;
  ------------------------------------------------------------------------------

  ------------------------------------------------------------------------------
  -- Process Name     : WRITE_DATA_LUT
  -- Sensitivity List : I_CLK          : System clock
  --                    I_RESET_N      : System reset (active low logic)
  -- Useful Outputs   : s_wr_data_byte : Data byte to send to Display
  -- Description      : Process to act as Look Up Table (LUT) to decide what
  --                    byte to send to display based on (s_wr_data_byte_index).
  ------------------------------------------------------------------------------
  WRITE_DATA_LUT: process (I_CLK, I_RESET_N)
  begin
    if (I_RESET_N = '0') then
      s_wr_data_byte                  <= C_7SD_CLEAR_DISP_CMD;
    elsif (rising_edge(I_CLK)) then
      case s_wr_data_byte_index is
        when 0      => s_wr_data_byte <= C_7SD_CLEAR_DISP_CMD;
        when 1      => s_wr_data_byte <= C_7SD_CLEAR_DISP_CMD;
        when 2      => s_wr_data_byte <= C_7SD_CLEAR_DISP_CMD;
        when 3      => s_wr_data_byte <= C_7SD_BRIGHTNESS_CMD;
        when 4      => s_wr_data_byte <= x"64";
        when 5      => s_wr_data_byte <= C_7SD_DEC_CTRL_CMD;
        when 6      => s_wr_data_byte <= x"00";  -- No decimal points
        when 7      => s_wr_data_byte <= C_7SD_CURS_CTRL_CMD;
        when 8      => s_wr_data_byte <= x"00";  -- Reset cursor to digit 0
        when 9      => s_wr_data_byte <= x"0" & s_display_data_latched(15 downto 12);
        when 10     => s_wr_data_byte <= x"0" & s_display_data_latched(11 downto 8);
        when 11     => s_wr_data_byte <= x"0" & s_display_data_latched(7 downto 4);
        when 12     => s_wr_data_byte <= x"0" & s_display_data_latched(3 downto 0);
        when others => s_wr_data_byte <= C_7SD_CLEAR_DISP_CMD;
      end case;
    end if;
  end process WRITE_DATA_LUT;
  ------------------------------------------------------------------------------

  s_7sd_wr <= '0'; -- Always writing

end architecture behavioral;
