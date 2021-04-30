module inert_intf(clk,rst_n,ptch,roll,yaw,strt_cal,cal_done,vld,SS_n,SCLK,
                  MOSI,MISO,INT);
				  
  parameter FAST_SIM = 1;		// used to accelerate simulation
 
  input clk, rst_n;
  input MISO;					// SPI input from inertial sensor
  input INT;					// goes high when measurement ready
  input strt_cal;				// from comand config.  Indicates we should start calibration
  
  output signed [15:0] ptch,roll,yaw;	// fusion corrected angles
  output cal_done;						// indicates calibration is done
  output reg vld;						// goes high for 1 clock when new outputs available
  output SS_n,SCLK,MOSI;				// SPI outputs

  ////////////////////////////////////////////
  // Declare any needed internal registers //
  //////////////////////////////////////////
  reg [7:0] pitchL, pitchH; // two bytes comprising pitch[15:0]
  reg [7:0] rollL, rollH; // two bytes comprising roll[15:0]
  reg [7:0] yawL, yawH; // two bytes comprising yaw[15:0]
  reg [7:0] AXL, AXH; // two bytes comprising AX[15:0]
  reg [7:0] AYL, AYH; // two bytes comprising AY[15:0]
  reg [16:0] timer;
  reg INT_ff1, INT_ff2; // metastablity FF outputs

  //////////////////////////////////////
  // Outputs of SM are of type logic //
  ////////////////////////////////////
  logic wrt; // asserted to start the intertial_integrater and SPI

  //////////////////////////////////////////////////////////////
  // Declare any needed internal signals that connect blocks //
  ////////////////////////////////////////////////////////////
  wire signed [15:0] ptch_rt,roll_rt,yaw_rt;	// feeds inertial_integrator
  wire signed [15:0] ax,ay;						// accel data to inertial_integrator
  reg [15:0] cmd; // cmd to be sent via the spi
  // these signals will tell the holding registers that they are ready to be written
  reg pitchL_vld, pitchH_vld, rollL_vld, rollH_vld, yawL_vld, yawH_vld, AXL_vld, AXH_vld,
    AYL_vld, AYH_vld;
  wire [15:0] inert_data; // data read from the SPI
  wire done; // done signal from the SPI
  reg clr_tmr; // signal used to clear the timer after INIT1 is done
  
  ///////////////////////////////////////
  // Create enumerated type for state //
  /////////////////////////////////////
  typedef enum reg [3:0] {IDLE, RDY, INIT1, INIT2, INIT3, INIT4, RDPTCHL, RDPTCHH, RDRLLL,
    RDRLLH, RDYWL, RDYWH, RDAXL, RDAXH, RDAYL, RDAYH} state_t;
  state_t nxt_state, state; // state regs

  ////////////////////////////////////////////////////////////
  // Instantiate SPI monarch for Inertial Sensor interface //
  //////////////////////////////////////////////////////////
  SPI_mnrch iSPI(.clk(clk),.rst_n(rst_n),.SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),.MOSI(MOSI),
                 .wrt(wrt),.done(done),.rd_data(inert_data),.wt_data(cmd));
				  
  ////////////////////////////////////////////////////////////////////
  // Instantiate Angle Engine that takes in angular rate readings  //
  // and acceleration info and produces ptch,roll, & yaw readings //
  /////////////////////////////////////////////////////////////////
  inertial_integrator #(FAST_SIM) iINT(.clk(clk), .rst_n(rst_n), .strt_cal(strt_cal), .cal_done(cal_done),
                                       .vld(vld), .ptch_rt(ptch_rt), .roll_rt(roll_rt), .yaw_rt(yaw_rt), .ax(ax),
						                           .ay(ay), .ptch(ptch), .roll(roll), .yaw(yaw));

  // Timer for the initialization setup
  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      timer <= 16'h0000;
    else if (clr_tmr) // better to have a clear than to let the timer needlessly run
      timer <= 16'h0000;
    else
      timer <= timer + 1;
  
  // double flopped INT input for metastability issues
  // placed in the same always block for clarity of the double chain
  always @(posedge clk, negedge rst_n)
    if (!rst_n) begin
      INT_ff1 <= 0;
      INT_ff2 <= 0;
    end else begin
      INT_ff1 <= INT;
      INT_ff2 <= INT_ff1;
    end

  // concatenate the high and low bytes of the holding regs into 
  assign ptch_rt = {pitchH, pitchL};
  assign roll_rt = {rollH, rollL};
  assign yaw_rt = {yawH, yawL};
  assign ax = {AXH, AXL};
  assign ay = {AYH, AYL};

  // state machine transition block
  always_comb begin
    vld = 0;
    wrt = 0;
    pitchL_vld = 0;
    pitchH_vld = 0;
    rollL_vld = 0;
    rollH_vld = 0;
    yawL_vld = 0;
    yawH_vld = 0;
    AXL_vld = 0;
    AXH_vld = 0;
    AYL_vld = 0;
    AYH_vld = 0;
    clr_tmr = 1; // normally want the timer off
    cmd = 16'h0000;
    nxt_state = state;
    case (state)
      INIT2: begin
        cmd = 16'h1062; // set up acceleration for 416Hz data rate
        if (done) begin
          wrt = 1;
          nxt_state = INIT3;
        end
      end
      INIT3: begin
        cmd = 16'h1162; // set up gyro for 416 Hz data rate
        if (done) begin
          wrt = 1;
          nxt_state = INIT4;
        end
      end
      INIT4: begin
        cmd = 16'h1460; // turn rounding on for both acceleration and gyro
        if (done) begin
          wrt = 1;
          nxt_state = IDLE;
        end
      end
      RDPTCHL: begin // no reads are currently being performed, so we can immediately read
        cmd = 16'hA200; // read lower pitch byte
        wrt = 1;
        nxt_state = RDPTCHH;
      end
      RDPTCHH: begin
        cmd = 16'hA300; // read higher pitch byte
        if (done) begin // we need to wait until the previous transaction is done before proceeding
          pitchL_vld = 1; // state the previous byte transaction is complete for the flip flop
          wrt = 1;
          nxt_state = RDRLLL;
        end
      end
      RDRLLL: begin
        cmd = 16'hA400; // read lower roll byte
        if (done) begin // same as above and all remaining byte transactions
          pitchH_vld = 1;
          wrt = 1;
          nxt_state = RDRLLH;
        end
      end
      RDRLLH: begin
        cmd = 16'hA500; // read higher roll byte
        if (done) begin
          rollL_vld = 1;
          wrt = 1;
          nxt_state = RDYWL;
        end
      end
      RDYWL: begin
        cmd = 16'hA600; // read lower yaw byte
        if (done) begin
          rollH_vld = 1;
          wrt = 1;
          nxt_state = RDYWH;
        end
      end
      RDYWH: begin
        cmd = 16'hA700; // read higher yaw byte
        if (done) begin
          yawL_vld = 1;
          wrt = 1;
          nxt_state = RDAXL;
        end
      end
      RDAXL: begin
        cmd = 16'hA800; // read lower AX byte
        if (done) begin
          yawH_vld = 1;
          wrt = 1;
          nxt_state = RDAXH;
        end
      end
      RDAXH: begin
        cmd = 16'hA900; // read higher AX byte
        if (done) begin
          AXL_vld = 1;
          wrt = 1;
          nxt_state = RDAYL;
        end
      end
      RDAYL: begin
        cmd = 16'hAA00; // read lower AY byte
        if (done) begin
          AXH_vld = 1;
          wrt = 1;
          nxt_state = RDAYH;
        end
      end
      RDAYH: begin
        cmd = 16'hAB00; // read higher AY byte
        if (done) begin
          AYL_vld = 1;
          wrt = 1;
          nxt_state = RDY;
        end
      end
      IDLE: begin // waiting for INT to go high
        if (INT_ff2)
          nxt_state = RDPTCHL;
      end
      RDY: begin // indicate we are done
        if (done) begin
          AYH_vld = 1;
          vld = 1;
          nxt_state = IDLE;
        end
      end
      default: begin // INIT1
        clr_tmr = 1'b0;
        cmd = 16'h0D02; // enable interrupt upon data ready
        if (&timer) begin
          wrt = 1;
          nxt_state = INIT2;
        end
      end
    endcase
  end

  // curr state block
  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      state <= INIT1;
    else
      state <= nxt_state;

  /// BLOCK OF HOLDING REGISTERS ///
  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      pitchL <= 16'h0000;
    else if (pitchL_vld)
      pitchL <= inert_data[7:0];

  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      pitchH <= 16'h0000;
    else if (pitchH_vld)
      pitchH <= inert_data[7:0];

  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      rollL <= 16'h0000;
    else if (rollL_vld)
      rollL <= inert_data[7:0];

  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      rollH <= 16'h0000;
    else if (rollH_vld)
      rollH <= inert_data[7:0];

  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      yawL <= 16'h0000;
    else if (yawL_vld)
      yawL <= inert_data[7:0];

  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      yawH <= 16'h0000;
    else if (yawH_vld)
      yawH <= inert_data[7:0];

  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      AXL <= 16'h0000;
    else if (AXL_vld)
      AXL <= inert_data[7:0];

  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      AXH <= 16'h0000;
    else if (AXH_vld)
      AXH <= inert_data[7:0];

  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      AYL <= 16'h0000;
    else if (AYL_vld)
      AYL <= inert_data[7:0];

  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      AYH <= 16'h0000;
    else if (AYH_vld)
      AYH <= inert_data[7:0];

endmodule
	  