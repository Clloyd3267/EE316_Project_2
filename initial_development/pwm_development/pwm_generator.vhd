--pwm generator DE2
library ieee;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;
	use ieee.std_logic_unsigned.all;

entity pwm_generator is
	generic(
		  clk_freq	: integer := 50; --the 50MHz input clock
		  data_res	: integer := 6	  --the data resolution
	);

	port(
		  clk				: in std_logic;							--pwm clock
		  rst				: in std_logic;							--reset (active low)
		  en				: in std_logic;							--pwm enable
		  pwm_datain	: in std_logic_vector(5 downto 0);	--input data for pwm generator
		  pwm_out		: out std_logic 							--the output signal
		  );

end entity pwm_generator;

architecture rtl of pwm_generator is

--signals
	signal count : integer:= 0;

begin

--processes
	process(clk, rst) begin

	if(en = '1') then
		if(rst = '0') then
			pwm_out <= '0';
			count <= 0;

		elsif(clk'event and clk = '1') then

			if (count < (2 ** data_res)) then	--when count is less than counter
				count <= count + 1;		--increments counter
			else
				count <= 0;				--resets counter
			end if;

			if(count < to_integer(unsigned(pwm_datain))) then	--stays on for duty cycle
				pwm_out <= '1';
			else
				pwm_out <= '0';
			end if;
		end if;
	else
		pwm_out <= '0';
	end if;

	end process;

end rtl;
