###################################################################

# Created by write_sdc on Fri Apr 30 09:52:21 2021

###################################################################
set sdc_version 2.1

set_units -time ns -resistance MOhm -capacitance fF -voltage V -current uA
set_wire_load_model -name 16000 -library saed32rvt_tt0p85v25c
set_max_transition 0.15 [current_design]
set_driving_cell -lib_cell NAND2X2_RVT -library saed32rvt_tt0p85v25c           \
[get_ports RST_n]
set_driving_cell -lib_cell NAND2X2_RVT -library saed32rvt_tt0p85v25c           \
[get_ports MISO]
set_driving_cell -lib_cell NAND2X2_RVT -library saed32rvt_tt0p85v25c           \
[get_ports INT]
set_driving_cell -lib_cell NAND2X2_RVT -library saed32rvt_tt0p85v25c           \
[get_ports RX]
set_load -pin_load 0.1 [get_ports SS_n]
set_load -pin_load 0.1 [get_ports SCLK]
set_load -pin_load 0.1 [get_ports MOSI]
set_load -pin_load 0.1 [get_ports TX]
set_load -pin_load 0.1 [get_ports FRNT]
set_load -pin_load 0.1 [get_ports BCK]
set_load -pin_load 0.1 [get_ports LFT]
set_load -pin_load 0.1 [get_ports RGHT]
create_clock [get_ports clk]  -period 3.75  -waveform {0 1.875}
set_clock_uncertainty 0.2  [get_clocks clk]
set_input_delay -clock clk  0.25  [get_ports RST_n]
set_input_delay -clock clk  0.25  [get_ports MISO]
set_input_delay -clock clk  0.25  [get_ports INT]
set_input_delay -clock clk  0.25  [get_ports RX]
set_output_delay -clock clk  0.5  [get_ports SS_n]
set_output_delay -clock clk  0.5  [get_ports SCLK]
set_output_delay -clock clk  0.5  [get_ports MOSI]
set_output_delay -clock clk  0.5  [get_ports TX]
set_output_delay -clock clk  0.5  [get_ports FRNT]
set_output_delay -clock clk  0.5  [get_ports BCK]
set_output_delay -clock clk  0.5  [get_ports LFT]
set_output_delay -clock clk  0.5  [get_ports RGHT]
