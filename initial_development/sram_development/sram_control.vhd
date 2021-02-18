-----------------------------------------------------------------------------
-- Author		: Camilla Ketola
-- Project		: PWM Generation
-- Description	: Creates the SRAM entity and state machine for PWM generation
-----------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- SRAM Entity 

entity sram_control is
	port(clk					: in std_logic;	--clock 
		  rst					: in std_logic;	--reset button
		  en					: in std_logic;	--enable
		  rw					: in std_logic; 	--the read/write signal
		  ce					: out std_logic;	--chip enable
		  we					: out std_logic;	--write enable
		  oe					: out std_logic;	--output enable
		  ub					: out std_logic;	--upper bit output
		  lb					: out std_logic; 	--lower but output
		  datain				: in std_logic_vector(15 downto 0);		-- sram input data
		  dataout			: out std_logic_vector(15 downto 0);	--sram output data
		  dataio				: inout std_logic_vector(15 downto 0);	--sram i/o
		  addr_in			: in std_logic_vector(17 downto 0);		--sram address input
		  addr_out			: out std_logic_vector(17 downto 0)
	);
	
end sram_control;

--architecture

architecture behaviour of sram_control is

type FSM_States is (INIT, STANDBY, READ0, WRITE0);

signal cstate: FSM_States; 

signal data : std_logic_vector(15 downto 0);

signal writeto : std_logic;

begin
--state interactions

	process(clk) begin
	
			if(clk'event and clk='1') then 
				case cstate is
					when INIT =>
						cstate <= STANDBY;
					when STANDBY =>
						if(rw = '0') then 
							cstate <= WRITE0;
						elsif(rw = '1') then 
							cstate <= READ0;
						end if;
					when READ0 =>
						cstate <= STANDBY;
					when WRITE0 =>
						cstate <= STANDBY;
				end case;
				
			end if;
	end process;
	
--state machine 

	process(clk, rw) begin
	
			if(clk'event and clk='1') then
				case cstate is
					
					when INIT =>
						we <= '1';
						oe <= '1';
					when STANDBY =>
						writeto <= '0';
						we <= '1';
						oe <= '1';
					when READ0 =>
						data <= dataio;
						we <= '0';
						oe <= '1';
					when WRITE0 =>
						writeto <= '1';
						we <= '0';
						oe <= '1';
				end case;
			end if;
	end process;
	
	ce <= '0';
	ub <= '0';
	lb <= '0';
	dataio <= datain when writeto = '0';
	dataout <= data;
	addr_out <= addr_in;

end architecture;


	