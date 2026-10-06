-- Created by IP Generator (Version 2025.2 build 211867)
-- Instantiation Template
--
-- Insert the following codes into your VHDL file.
--   * Change the_instance_name to your own instance name.
--   * Change the net names in the port map.


COMPONENT pll
  PORT (
    clkout0 : OUT STD_LOGIC;  -- 37.12500000MHz
    clkout1 : OUT STD_LOGIC;  -- 9.99519231MHz
    clkout2 : OUT STD_LOGIC;  -- 24.75000000MHz
    lock : OUT STD_LOGIC;
    clkin1 : IN STD_LOGIC  -- 27.00000000MHz
  );
END COMPONENT;


the_instance_name : pll
  PORT MAP (
    clkout0 => clkout0,
    clkout1 => clkout1,
    clkout2 => clkout2,
    lock => lock,
    clkin1 => clkin1
  );
