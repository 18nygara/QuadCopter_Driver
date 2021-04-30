module flght_cntrl(clk,rst_n,vld,inertial_cal,d_ptch,d_roll,d_yaw,ptch,
					roll,yaw,thrst,frnt_spd,bck_spd,lft_spd,rght_spd);
				
input clk,rst_n;
input vld;									// tells when a new valid inertial reading ready
											// only update D_QUEUE on vld readings
input inertial_cal;							// need to run motors at CAL_SPEED during inertial calibration
input signed [15:0] d_ptch,d_roll,d_yaw;	// desired pitch roll and yaw (from cmd_cfg)
input signed [15:0] ptch,roll,yaw;			// actual pitch roll and yaw (from inertial interface)
input [8:0] thrst;							// thrust level from slider
output [10:0] frnt_spd;						// 11-bit unsigned speed at which to run front motor
output [10:0] bck_spd;						// 11-bit unsigned speed at which to back front motor
output [10:0] lft_spd;						// 11-bit unsigned speed at which to left front motor
output [10:0] rght_spd;						// 11-bit unsigned speed at which to right front motor


//////////////////////////////////////////////////////
// You will need a bunch of interal wires declared //
// for intermediate math results...do that here   //
///////////////////////////////////////////////////
wire [9:0] ptch_pterm, roll_pterm, yaw_pterm;
wire [11:0] ptch_dterm, roll_dterm, yaw_dterm;
wire [12:0] thrst_ext, ptch_pterm_ext, ptch_dterm_ext, roll_pterm_ext, roll_dterm_ext, yaw_pterm_ext, yaw_dterm_ext;
wire [12:0] sum_front, sum_back, sum_right, sum_left; // actual speed values for the copter
wire [10:0] sum_front_sat, sum_back_sat, sum_right_sat, sum_left_sat; // saturated speed values - needed for ESC speed width

///////////////////////////////////////////////////////////////
// some Parameters to keep things more generic and flexible //
/////////////////////////////////////////////////////////////
localparam CAL_SPEED = 11'h290;		// speed to run motors at during inertial calibration
localparam MIN_RUN_SPEED = 13'h2C0;	// minimum speed while running  
localparam D_COEFF = 5'b00111;		// D coefficient in PID control = +7

//////////////////////////////////////
// Instantiate 3 copies of PD_math //
////////////////////////////////////
PD_math iPTCH(.clk(clk),.rst_n(rst_n),.vld(vld),.desired(d_ptch),.actual(ptch),.pterm(ptch_pterm),.dterm(ptch_dterm));
PD_math iROLL(.clk(clk),.rst_n(rst_n),.vld(vld),.desired(d_roll),.actual(roll),.pterm(roll_pterm),.dterm(roll_dterm));
PD_math iYAW(.clk(clk),.rst_n(rst_n),.vld(vld),.desired(d_yaw),.actual(yaw),.pterm(yaw_pterm),.dterm(yaw_dterm));  

// get the sign extended terms
assign thrst_ext = {{4{1'b0}}, thrst[8:0]};
assign ptch_pterm_ext = {{3{ptch_pterm[9]}}, ptch_pterm[9:0]};
assign roll_pterm_ext = {{3{roll_pterm[9]}}, roll_pterm[9:0]};
assign yaw_pterm_ext = {{3{yaw_pterm[9]}}, yaw_pterm[9:0]};
assign ptch_dterm_ext = {ptch_dterm[11], ptch_dterm[11:0]};
assign roll_dterm_ext = {roll_dterm[11], roll_dterm[11:0]};
assign yaw_dterm_ext = {yaw_dterm[11], yaw_dterm[11:0]};

// front speed calculation
assign sum_front = MIN_RUN_SPEED + thrst_ext - ptch_pterm_ext - ptch_dterm_ext - yaw_pterm_ext - yaw_dterm_ext;
assign sum_front_sat = (|sum_front[12:11]) ? 11'h7ff : sum_front[10:0]; // saturate to 11 bits - this is the width ESC uses

// back speed calculation
assign sum_back = MIN_RUN_SPEED + thrst_ext + ptch_pterm_ext + ptch_dterm_ext - yaw_pterm_ext - yaw_dterm_ext;
assign sum_back_sat =(|sum_back[12:11]) ? 11'h7ff : sum_back[10:0]; // saturate to 11 bits - this is the width ESC uses

// left speed calculation
assign sum_left = MIN_RUN_SPEED + thrst_ext - roll_pterm_ext - roll_dterm_ext + yaw_pterm_ext + yaw_dterm_ext;
assign sum_left_sat = (|sum_left[12:11]) ? 11'h7ff : sum_left[10:0]; // saturate to 11 bits - this is the width ESC uses
      
// right speed calculation
assign sum_right = MIN_RUN_SPEED + thrst_ext + roll_pterm_ext + roll_dterm_ext + yaw_pterm_ext + yaw_dterm_ext;
assign sum_right_sat = (|sum_right[12:11]) ? 11'h7ff : sum_right[10:0]; // saturate to 11 bits - this is the width ESC uses

// calibrate or assign to the saturated value
assign frnt_spd = inertial_cal ? CAL_SPEED : sum_front_sat;
assign bck_spd = inertial_cal ? CAL_SPEED : sum_back_sat;
assign lft_spd = inertial_cal ? CAL_SPEED : sum_left_sat;
assign rght_spd = inertial_cal ? CAL_SPEED : sum_right_sat;

endmodule 
