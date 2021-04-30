module ESCs(clk, rst_n, frnt_spd, bck_spd, lft_spd, rght_spd, wrt, motors_off, frnt,
            bck, lft, rght);

  input clk, rst_n;
  // speeds of each of the motors - we need to separate thsee for each of the ESC controllers
  input [10:0] frnt_spd, bck_spd, lft_spd, rght_spd;
  input wrt, motors_off; // interconnect signals that provide functional enable / reset

  // PWM signals
  output frnt, bck, lft, rght;

  // we need extra wires to assign speed ins based off of motors off
  wire [10:0] frnt_spd_in, bck_spd_in, rght_spd_in, lft_spd_in;

  // assignments to speed based on motors off
  assign frnt_spd_in = motors_off ? 11'd0 : frnt_spd;
  assign bck_spd_in = motors_off ? 11'd0 : bck_spd;
  assign rght_spd_in = motors_off ? 11'd0 : rght_spd;
  assign lft_spd_in = motors_off ? 11'd0 : lft_spd;

  // block of ESC_Interfaces that drive each motor, respectively
  ESC_interface ifrnt(.clk(clk), .rst_n(rst_n), .wrt(wrt), .SPEED(frnt_spd_in), .PWM(frnt));
  ESC_interface ibck(.clk(clk), .rst_n(rst_n), .wrt(wrt), .SPEED(bck_spd_in), .PWM(bck));
  ESC_interface ilft(.clk(clk), .rst_n(rst_n), .wrt(wrt), .SPEED(lft_spd_in), .PWM(lft));
  ESC_interface irght(.clk(clk), .rst_n(rst_n), .wrt(wrt), .SPEED(rght_spd_in), .PWM(rght));

endmodule