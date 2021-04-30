module ESC_interface(clk, rst_n, wrt, SPEED, PWM);
				
  input clk, rst_n, wrt;
  input [10:0] SPEED;
  output reg PWM; // PWM signal that actually drives the motors

  wire all_zeros; // when the timer reaches 0, indicate that we want the PWM low
  reg [10:0] flop_speed; // pipeline flop for speed, solving max delay
  wire [11:0] double_speed; // used to use a shift and add rather than a multiply
  wire [12:0] triple_speed; // we have a width of 2047 possible values on SPEED, max PWM on would be 6250 clk
                            // cycles - meaning that 2047 * 3 ~= 6250. This is why we need triple_speed here
  reg [13:0] hold_high_tmr; // timer that will reflect the PWM being high when it is actively counting

  localparam MIN_DTY_CYCL = 6250; // minimum duty cycle to output for the PWM signal
  
  // pipeline flop used to bring down max delay timing
  always @(posedge clk, negedge rst_n) begin
    if (!rst_n)
      flop_speed <= 11'h000;
    else
      flop_speed <= SPEED;
    end

  assign double_speed = {flop_speed[10:0], 1'b0}; // shift by 1 instead of using mult, easier to visualize synthesis
  assign triple_speed = double_speed + flop_speed; // add 2*speed + speed = 3*speed

  // COUNT DOWN FF
  always @(posedge clk, negedge rst_n) begin
    if (!rst_n)
      hold_high_tmr <= 14'h0000;
    else if (wrt) // write to the flop
      hold_high_tmr <= MIN_DTY_CYCL + triple_speed;
    else // decrement since we want to check all zeroes when we're done counting
      hold_high_tmr <= hold_high_tmr - 1;
  end
  
  // when the timer goes to all zeroes - we want to lower the PWM signal
  assign all_zeros = ~(|hold_high_tmr);

  // SR FF implementation
  always @(posedge clk, negedge rst_n) begin
    if (!rst_n)
      PWM <= 1'b0;
    else if (wrt) // wrt = SET
      PWM <= 1'b1;
    else if (all_zeros) // all_zeros = RESET
      PWM <= 1'b0;
  end

endmodule 
