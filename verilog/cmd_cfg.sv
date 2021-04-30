module cmd_cfg(clk, rst_n, cmd_rdy, cmd, data, clr_cmd_rdy, resp, send_resp, d_ptch, d_roll,
	d_yaw, thrst, strt_cal, inertial_cal, cal_done, motors_off);

	// Inputs
	input logic clk,rst_n;						// clock and active low reset
	input logic cmd_rdy;						  // New command valid from UART_wrapper
	input logic [7:0] cmd;						// Command opcode
	input logic [15:0] data;					// Data to accompany command
	input logic cal_done;						  // From inertial_integrator. Indicates calibration is complete

	// outputs
	output logic clr_cmd_rdy;					// Knocks down cmd_rdy after cmd_cfg has "digested" command
	output logic [7:0] resp;					// Response back to remote. Typically pos_ack (0xA5)
	output logic send_resp;						// Indicates UART_comm should send response byte
	output logic [15:0] d_ptch, d_roll, d_yaw; 	// Desired pitch, roll, yaw as 16-bit signed numbers. cmd_cfg will have
												// registers that store the desired pitch, roll, and yaw and output them
												// to flght_cntrl unit.
	output logic [8:0] thrst;					// 9-bit unsigned thrust level. Goes to flght_cntrl.
	output logic strt_cal;						// Indicates to intertial_integrator to start calibration procedure. Only 1
												// clock wide pulse at end of 1.32 sec motor spinup period
	output logic inertial_cal;					// To flght_cntrl unit. Held high during duration fo calibration (including
												// motor spin up). Keeps motor speed at CAL_SPEED.
	output logic motors_off;					// Goes to ESCs (see project spec), shuts off motors. (user back off throttle and
												// gestures down to shut off copter after landing, or more likely after crashing).
	
	// other internal signals
	parameter FAST_SIM = 1;        		// can be set at testbench level
	logic tmr_full; 					        // output from mtr_ramp_tmr
	logic [25:0] mtr_ramp_tmr;       	// timer that waits after inertial_cal is asserted, allowing motors to get up to speed
	logic wptch, wroll, wyaw, wthrst;	// selector signals for flops
	logic emergency_landing; 					// true after EMER_LAND cmd
	logic clr_tmr;										// SM signal tells mtr_ramp_tmr to reset
	logic mtrs_off;										// SM signal to set motors_off
	
	// enumeration for cmds
	typedef enum reg [3:0] {SET_PITCH=4'h2, SET_ROLL=4'h3,
						   SET_YAW=4'h4, SET_THRUST=4'h5,
						   CALIBRATE=4'h6, EMER_LAND=4'h7, 
						   MOTORS_OFF=4'h8} opcode_t;
	
	// flop to generate counter behavior in mtr_ramp_tmr
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n) 
			mtr_ramp_tmr <= 26'd0;
		else if (clr_tmr) // when in IDLE - hold at 0
			mtr_ramp_tmr <= 26'd0; 
		/* since FAST_SIM has two different counting modes, we need to manually reset instead of
		   relying on overflow to reset our counter */
		else if (tmr_full)
			mtr_ramp_tmr <= 26'd0;
		else
			mtr_ramp_tmr <= mtr_ramp_tmr + 1;
	end

	/* tmr_full is high if 9 bits are ones in tmr, given that FAST_SIM is set, otherwise 
		 tmr_full is high if tmr's 26 bits are ones */
	generate if (FAST_SIM)
		assign tmr_full = (&mtr_ramp_tmr[8:0]) ? 1'b1 : 1'b0;
	else
		assign tmr_full = (&mtr_ramp_tmr) ? 1'b1 : 1'b0;
	endgenerate   

	// d_ptch signal flop
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			d_ptch <= 16'h0000;
		else if (emergency_landing)
			d_ptch <= 16'h0000;
		else if (wptch)
			d_ptch <= data;
	end

	// d_roll signal flop
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			d_roll <= 16'h0000;
		else if (emergency_landing)
			d_roll <= 16'h0000;
		else if (wroll)
			d_roll <= data;
	end

	// d_yaw signal flop
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			d_yaw <= 16'h0000;
		else if (emergency_landing)
			d_yaw <= 16'h0000;
		else if (wyaw)
			d_yaw <= data;
	end
	
	// thrst signal flop
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n)
			thrst <= 9'h000;
		else if (emergency_landing)
			thrst <= 9'h000;
		else if (wthrst)
			thrst <= data[8:0];
	end
	
	// motors_off set/ reset flop
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n) // want motors_off active on a reset
			motors_off <= 1'b1;
		else if (mtrs_off)
			motors_off <= 1'b1;
		else if (inertial_cal)
			motors_off <= 1'b0;
	end
	
	////////////////////
	// State Machine //
	//////////////////
	
	// state machine enum variables
	typedef enum reg[1:0] {IDLE, READY, CAL, PACK} state_t;	
	state_t state, next_state;
	
	// state flop
	always_ff@(posedge clk, negedge rst_n) begin
		if(!rst_n)
			state <= IDLE;
		else
			state <= next_state;
	end // end always_ff
	
	// Update transition block
	always_comb begin
		// default outputs //
		wptch = 1'b0;
		wroll = 1'b0;
		wyaw = 1'b0;
		wthrst = 1'b0;
		mtrs_off = 1'b0;
		inertial_cal = 1'b0; 
		strt_cal = 1'b0;
		clr_cmd_rdy = 1'b0;
		resp = 8'h00;
		send_resp = 1'b0;
		clr_tmr = 1'b0;
		emergency_landing = 1'b0;
		next_state = state;
		case(state)
			READY : // takes an arbitrary command - now we have to parse it
				case(cmd)
					SET_PITCH : begin
						next_state = PACK; // now we want to acknowledge the set_ptch command
						wptch = 1'b1;
					end
					SET_ROLL : begin
						next_state = PACK; // now we want to acknowledge the set_roll command
						wroll = 1'b1;
					end
					SET_YAW : begin
						next_state = PACK; // now we want to acknowledge the set_yaw command
						wyaw = 1'b1;
					end
					SET_THRUST : begin
						next_state = PACK; // now we want to acknowledge the set_thrst command
						wthrst = 1'b1;
					end
					CALIBRATE : begin
						inertial_cal = 1'b1; // assert and hold for timer
						if(tmr_full) begin
							next_state = CAL; // now we should wait for the calibration to complete
							strt_cal = 1'b1; // pulse strt_cal for this clock cycle
						end
					end
					EMER_LAND : begin
						next_state = PACK; // must acknowledge that we've handled the command
						emergency_landing = 1'b1;
					end
					MOTORS_OFF : begin
						next_state = PACK; // must acknowledge that we've handled the command
						mtrs_off = 1'b1;
					end
					default : // invalid command
						next_state = IDLE;
				endcase
			CAL : begin
				inertial_cal = 1'b1; // assert and hold til cal_done	
				if(cal_done) begin // gotten from the integrator - shows that we have fully calibrated
					next_state = PACK;
				end			
			end
			PACK : begin
				resp = 8'hA5; // typical acknowledgement byte
				send_resp = 1'b1;
				clr_cmd_rdy = 1'b1;
				next_state = IDLE;
			end
			default : begin // IDLE
				clr_tmr = 1'b1;
				if (cmd_rdy) begin
					next_state = READY;
				end
			end
		endcase
	end // end always_comb

endmodule
