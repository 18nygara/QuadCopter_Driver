module PD_math(clk, rst_n, vld, desired, actual, pterm, dterm);

  input clk, rst_n, vld;
  input [15:0] desired, actual;
  output [9:0] pterm;
  output reg [11:0] dterm;

  localparam D_QUEUE_DEPTH = 12; // number of flops used for derivation
  localparam derivative_term = 5'b00111;

  wire [16:0] err;
  wire [9:0] err_sat;
  reg [9:0] err_sat_piped; // used for pipelining - solves max delay issues
  wire [16:0] ext_desired, ext_actual; // sign extended versions of desired and actual
  wire [9:0] D_diff; // derivative from error
  wire signed [6:0] D_diff_sat;
  wire [9:0] half_err, eigth_err; // used for calculating 5/8 multiplier for pterm
  reg [9:0] prev_err [0:D_QUEUE_DEPTH-1]; // memory of flops used to cache previous err_sat

  reg [4:0] i; // loop variable used to assign all flops for derivative

  wire [11:0] dterm_raw; // input to the pipeline flop - solves max delay issues

  // pipeline dterm - solve max delay issues
  always @(posedge clk, negedge rst_n) begin
    if (!rst_n) 
      dterm <= 12'h000;
    else 
      dterm <= dterm_raw;
  end

  // pipeline err_sat - solve max delay issues
  always @(posedge clk, negedge rst_n) begin
    if (!rst_n)
      err_sat_piped <= 10'h000;
    else 
      err_sat_piped <= err_sat;
  end

  // sign extend block
  assign ext_desired = {desired[15], desired[15:0]};
  assign ext_actual = {actual[15], actual[15:0]};

  // subtract block
  assign err = ext_actual - ext_desired;

  // saturate 10 bits block
  assign err_sat = (err[16]) ?
    ((&err[15:9]) ? err[9:0] : 10'h200) :
    ((|err[15:9]) ? 10'h1ff : err[9:0]);
  
  // ff block
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      for (i = 0; i < D_QUEUE_DEPTH; i = i + 1) // duplicated logic - fixed length for loop
        prev_err[i] <= 10'h000;
    else if (vld) begin
      // shift data over
      for (i = D_QUEUE_DEPTH - 1; i > 0; i = i - 1) // shift down the flops
        prev_err[i] <= prev_err[i-1];
      prev_err[0] <= err_sat_piped; // new data arrives at LSR
    end

  // 2nd subtract block
  assign D_diff = err_sat_piped - prev_err[D_QUEUE_DEPTH-1];

  // saturate 7 bits block
  assign D_diff_sat = (D_diff[9]) ? 
    ((&D_diff[8:6]) ? D_diff[6:0] : 7'h40) :
    ((|D_diff[8:6]) ? 7'h3f : D_diff[6:0]);

  // multiply block
  assign dterm_raw = $signed(derivative_term) * D_diff_sat;

  // 5/8 logic
  assign half_err = {err_sat_piped[9], err_sat_piped[9:1]};
  assign eigth_err = {{3{err_sat_piped[9]}},err_sat_piped[9:3]};
  assign pterm = half_err + eigth_err;

endmodule