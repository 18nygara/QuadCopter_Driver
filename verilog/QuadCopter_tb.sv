module QuadCopter_tb();
			
  //// Interconnects to DUT/support defined as type wire ////
  wire SS_n, SCLK, MOSI, MISO, INT;
  wire RX, TX;
  wire [7:0] resp;				// response from DUT
  wire cmd_sent, resp_rdy;
  wire frnt_ESC, back_ESC, left_ESC, rght_ESC;

  ////// Stimulus is declared as type reg ///////
  reg clk, RST_n;
  reg [7:0] host_cmd;				// command host is sending to DUT
  reg [15:0] data;				// data associated with command
  reg send_cmd;					// asserted to initiate sending of command
  reg clr_resp_rdy;				// asserted to knock down resp_rdy

  integer i, j, k; // for loop variables
  reg [7:0] cmd2send; // task input that allows us to manually pick a command

  //// localparams for command encoding ////
  localparam RANDOM = 4'hf;
  localparam SET_PITCH = 4'h2;
  localparam SET_ROLL = 4'h3;
  localparam SET_YAW = 4'h4;
  localparam SET_THRUST = 4'h5;
  localparam CALIBRATE = 4'h6;
  localparam EMER_LAND = 4'h7;
  localparam MOTORS_OFF = 4'h8;

  ////////////////////////////////////////////////////////////////
  // Instantiate Physical Model of Copter with Inertial sensor //
  //////////////////////////////////////////////////////////////	

  // THIS IS NOT INCLUDED IN THE REPO - THIS WAS A TESTING MODULE THAT I DID NOT WRITE //
  CycloneIV iQuad(.clk(clk),.RST_n(RST_n),.SS_n(SS_n),.SCLK(SCLK),.MISO(MISO),
                  .MOSI(MOSI),.INT(INT),.frnt_ESC(frnt_ESC),.back_ESC(back_ESC),
                  .left_ESC(left_ESC),.rght_ESC(rght_ESC));				  			
    
    
  ////// Instantiate DUT ////////
  QuadCopter iDUT(.clk(clk),.RST_n(RST_n),.SS_n(SS_n),.SCLK(SCLK),.MOSI(MOSI),.MISO(MISO),
                  .INT(INT),.RX(RX),.TX(TX), .FRNT(frnt_ESC),.BCK(back_ESC),
                  .LFT(left_ESC),.RGHT(rght_ESC));


  //// Instantiate Master UART (mimics host commands) //////
  RemoteComm iREMOTE(.clk(clk), .rst_n(RST_n), .RX(TX), .TX(RX),
                     .cmd(host_cmd), .data(data), .send_cmd(send_cmd),
                     .cmd_sent(cmd_sent), .resp_rdy(resp_rdy),
                     .resp(resp), .clr_resp_rdy(clr_resp_rdy));

  always
      #5 clk = ~clk;

  initial begin
    // four different testing suites - run one after the other and stop if we error out.
	  control_workflow(); // this will test the basic control workflow calibrate - set p/r/y - land
    simple_cmds(); // sends one ptch roll or yaw command at a time and checks if the value is correct
    complex_cmds(); // this will send multiple ptch, roll, and yaw requests such that they overlap each other
    emer_landing_sequence(); // tests an emergency landing sequence
    $display("All tests passed!\n");
    $stop();
  end

  // TASK BLOCK // (Rupel said to place the tasks in here instead of a separate file)
  task initialize; // simple task to initialize the outputs of the DUT

    clk = 0;
    RST_n = 0;

    clr_resp_rdy = 1;
    send_cmd = 0;
    host_cmd = 8'h00;
    data = 16'h0000;

    @(posedge clk); // assert rst through a positive edge
    @(negedge clk); 
    RST_n = 1;
    clr_resp_rdy = 0;

  endtask

  task get_airbourne; // get off the ground so that the cyclone can test setting pitch, roll, and yaw

    clr_resp_rdy = 1; // clear the previous response
    host_cmd = SET_THRUST;
    data = 16'h00FF; // set to 0xFF - get off the ground
    send_cmd = 1;
    @(posedge clk); // send the command from the RemoteComm
    @(negedge clk);
    clr_resp_rdy = 0;
    send_cmd = 0;

    rcv_ack(); // wait for the thrust to be acknowledged
  endtask

  task calibrate; // task that will calibrate the quadcopter so that we can run other commands similar to a test flight

    clr_resp_rdy = 1; // clear the previous response
    host_cmd = CALIBRATE;
    data = 16'h0000;
    send_cmd = 1;
    @(posedge clk); // send the command from the RemoteComm
    @(negedge clk);
    clr_resp_rdy = 0;
    send_cmd = 0;

    rcv_ack(); // wait to be acknowledged
  endtask

  task create_cmd; // set up RemoteComm to send a new comand and then send it

    clr_resp_rdy = 1; // clear the previous response
    if (cmd2send == RANDOM)
      host_cmd = ($urandom() % 8'h03) + 2; // randomize selection of pitch roll, or yaw - NOT ANY OTHER COMMANDS, WE WANT TO PRESERVE CONTROL FLOW
    else
      host_cmd = cmd2send;
    // in practice - we only want the quadcopter to be within 15 degrees of upright, so we don't want to send data too large
    data = ($urandom() % 16'h0201) - 16'h0100; // get a value between positive and negative 256
    send_cmd = 1;
    @(posedge clk); // send the command from the RemoteComm
    @(negedge clk);
    clr_resp_rdy = 0;
    send_cmd = 0;

  endtask

  task rcv_ack; // simple task to wait for a response to be received by remote comm
  
    fork
      begin
        // test to see if the response made it out of the quadcopter and into the RemoteComm
        @(posedge resp_rdy);
        disable timeout1;
      end
      
      begin : timeout1
        repeat (2000000) @(posedge clk); // timeout loop waiting for the positive ack
        $error("Timed out waiting for positive acknowledgement to be generated from cmd_cfg.\n");
        $stop();
      end
    join
    
  endtask
  
  task control_workflow; // this will test the basic control workflow calibrate - set p/r/y - land
    
    $display("Starting control workflow tests...\n");

    initialize();
  
    for (i = 0; i < 4; i = i + 1) begin // cycle through 4 control flow tests
      $display("~~~ Cycle%d - control workflow test ~~~\n", i);

      calibrate(); // calibrate before launch
      get_airbourne(); // launch

      // set a few of the signals to test maneuvering
      cmd2send = SET_PITCH;
      create_cmd();
      rcv_ack();
      repeat (2000000) @(posedge clk); // wait 2000000 clk cycles for the cyclone to read the pwm
      
      fork
        begin : ctrl_flow_ptch
          repeat (10000000) @(posedge clk); // timeout loop waiting for the positive ack
          $error("Timed out waiting for the command %x to set the correct value.\n", host_cmd);
          $stop();
        end

        begin
          while (!(iDUT.ptch >= data - 16'h0005 && iDUT.ptch <= data + 16'h0005)) begin
            // allow a range of 10 around data to be accepted - busy wait while the signal changes
          end
          disable ctrl_flow_ptch;
        end
      join
      
      cmd2send = SET_ROLL;
      create_cmd();
      rcv_ack();
      repeat (2000000) @(posedge clk); // wait 2000000 clk cycles for the cyclone to read the pwm
      
      fork
        begin : ctrl_flow_roll
          repeat (10000000) @(posedge clk); // timeout loop waiting for the positive ack
          $error("Timed out waiting for the command %x to set the correct value.\n", host_cmd);
          $stop();
        end

        begin
          while (!(iDUT.roll >= data - 16'h0005 && iDUT.roll <= data + 16'h0005)) begin
            // allow a range of 10 around data to be accepted - busy wait while the signal changes
          end
          disable ctrl_flow_roll;
        end
      join
      
      cmd2send = SET_YAW;
      create_cmd();
      rcv_ack();
      repeat (2000000) @(posedge clk); // wait 2000000 clk cycles for the cyclone to read the pwm
      
      fork
        begin : ctrl_flow_yaw
          repeat (10000000) @(posedge clk); // timeout loop waiting for the positive ack
          $error("Timed out waiting for the command %x to set the correct value.\n", host_cmd);
          $stop();
        end

        begin
          while (!(iDUT.yaw >= data - 16'h0005 && iDUT.yaw <= data + 16'h0005)) begin
            // allow a range of 10 around data to be accepted - busy wait while the signal changes
          end
          disable ctrl_flow_yaw;
        end
      join

      // Basic flight should be good now - time to land
      cmd2send = MOTORS_OFF;
      create_cmd();
      rcv_ack();
      repeat (2000000) @(posedge clk); // wait 2000000 clk cycles for the cyclone to read the pwm
      
      fork
        begin : ctrl_flow_motors_off
          repeat (200000) @(posedge clk); // the turn off should happen relatively fast
          $error("Error waiting for the motors to turn off after motors_off cmd is asserted.\n");
          $stop();
        end

        begin
          // busy wait on these values while we wait for motors to turn off
          while (iDUT.frnt_spd != 0) begin
          end
          while (iDUT.bck_spd != 0) begin
          end
          while (iDUT.lft_spd != 0) begin
          end
          while (iDUT.rght_spd != 0) begin
          end
          disable ctrl_flow_motors_off;
        end

      join

      $display("Cycle%d control flow passed!\n", i);
    end

    $display("Control Workflow Tests Passed!\n");
  endtask

  task simple_cmds; // sends one ptch roll or yaw command at a time and checks if the value is correct

    $display("Starting simple command tests...\n");

    cmd2send = RANDOM;
    initialize();
    calibrate();
    get_airbourne();
    for (i = 0; i < 16; i = i + 1) begin // send 16 random commands that set pitch, roll, and yaw
      $display("~~~ Cycle%d - simple test ~~~\n", i);
      create_cmd(); // randomize a p/r/y command and send it via the remote
      rcv_ack();
      repeat (2000000) @(posedge clk); // wait 2000000 clk cycles for the cyclone to read the pwm
      
      fork
        begin : wait_for_value
          repeat (10000000) @(posedge clk);
          $error("Timed out waiting for the command %x to set the correct value.\n", host_cmd);
          $stop();
        end

        begin
          casex (host_cmd) // the command was 'randomized' - we need a case to check which parameter we changed
            SET_PITCH: begin // PITCH CHECK
              while (!(iDUT.ptch >= data - 16'h0005 && iDUT.ptch <= data + 16'h0005)) begin
                // allow a range of 10 around data to be accepted - busy wait while the signal changes
              end
            end
            SET_ROLL: // ROLL CHECK
              while (!(iDUT.roll >= data - 16'h0005 && iDUT.roll <= data + 16'h0005)) begin
                // allow a range of 10 around data to be accepted - busy wait while the signal changes
              end
            default: // YAW CHECK
              while (!(iDUT.yaw >= data - 16'h0005 && iDUT.yaw <= data + 16'h0005)) begin
                // allow a range of 10 around data to be accepted - busy wait while the signal changes
              end
          endcase

          $display("Cycle%d passed!\n", i);
          disable wait_for_value;
        end
      join

    end

    $display("Simple tests passed!\n");
  endtask

  task complex_cmds; // this will send multiple ptch, roll, and yaw requests such that they overlap each other

    $display("Starting Complex pitch/roll/yaw tests...\n");

    initialize();
    calibrate();
    get_airbourne();

    fork // we want ptch roll and yaw to send simultaneously

      begin // PTCH SEND
        for (i = 0; i < 5; i = i + 1) begin // send 5 random pitch commands
          cmd2send = SET_PITCH;
          create_cmd();
          rcv_ack();
          repeat (2000000) @(posedge clk); // wait 2000000 clk cycles for the cyclone to read the pwm
          
          fork

            begin : wait_for_value_ptch
              repeat (10000000) @(posedge clk);
              $error("Timed out waiting for the command %x to set the correct value.\n", host_cmd);
              $stop();
            end

            begin
              while (!(iDUT.ptch >= data - 16'h0005 && iDUT.ptch <= data + 16'h0005)) begin
                // allow a range of 10 around data to be accepted - busy wait while the signal changes
              end
              $display("ptch correct... onto the next cycle\n");
              disable wait_for_value_ptch;
            end

          join

        end
      end

      begin // ROLL SEND
        repeat (666666) @(posedge clk); // offset the sending of roll from ptch by 1/3 of 2 million
        for (j = 0; j < 5; j = j + 1) begin // send 5 random roll commands
          cmd2send = SET_ROLL;
          create_cmd();
          rcv_ack();
          repeat (2000000) @(posedge clk); // wait 2000000 clk cycles for the cyclone to read the pwm
          
          fork

            begin : wait_for_value_roll
              repeat (10000000) @(posedge clk);
              $error("Timed out waiting for the command %x to set the correct value.\n", host_cmd);
              $stop();
            end

            begin
              while (!(iDUT.roll >= data - 16'h0005 && iDUT.roll <= data + 16'h0005)) begin
                // allow a range of 10 around data to be accepted - busy wait while the signal changes
              end
              $display("roll correct... onto the next cycle\n");
              disable wait_for_value_roll;
            end
            
          join

        end
      end

      begin // YAW SEND
        repeat (1300000) @(posedge clk); // offset the sending of yaw from ptch by ~2/3 of 2 million
        for (k = 0; k < 5; k = k + 1) begin // send 5 random yaw commands
          cmd2send = SET_YAW;
          create_cmd();
          rcv_ack();
          repeat (2000000) @(posedge clk); // wait 2000000 clk cycles for the cyclone to read the pwm
          
          fork

            begin : wait_for_value_yaw
              repeat (10000000) @(posedge clk);
              $error("Timed out waiting for the command %x to set the correct value.\n", host_cmd);
              $stop();
            end

            begin
              while (!(iDUT.yaw >= data - 16'h0005 && iDUT.yaw <= data + 16'h0005)) begin
                // allow a range of 10 around data to be accepted - busy wait while the signal changes
              end
              $display("yaw correct... onto the next cycle\n");
              disable wait_for_value_yaw;
            end
            
          join

        end
      end

    join

    $display("Complex pitch/roll/yaw tests passed!\n");
  endtask

  task emer_landing_sequence; // tests an emergency landing sequence
    
    $display("Starting Landing Sequence Testing...\n");
    
    cmd2send = EMER_LAND; // EMERGENCY LAND command
    create_cmd();
    rcv_ack();
    repeat (2000000) @(posedge clk);
    fork

      begin : timeout_emergency_land
        repeat (10000000) @(posedge clk);
        $error("Error waiting for the ptch roll and yaw to become 0 after emergency land is asserted.\n");
        $stop();
      end

      begin
        while (!(iDUT.ptch >= -10 && iDUT.ptch <= 10)) begin
          // busy wait while the signal changes
        end
        while (!(iDUT.roll >= -10 && iDUT.roll <= 10)) begin
          // busy wait while the signal changes
        end
        while (!(iDUT.yaw >= -10 && iDUT.yaw <= 10)) begin
          // busy wait while the signal changes
        end
        while (iDUT.thrst != 0) begin
          // busy wait for the thrust - should already be set though...
        end
        disable timeout_emergency_land;
      end

    join

    cmd2send = MOTORS_OFF; // MOTORS_OFF command
    create_cmd();
    rcv_ack();
    // check to see if all motors are off.
    fork

      begin : timeout_motors_off
        repeat (200000) @(posedge clk);
        $error("Error waiting for the motors to turn off after motors_off cmd is asserted.\n");
        $stop();
      end

      begin
        // busy wait on these values while we wait for them to turn off
        while (iDUT.frnt_spd != 0) begin
        end
        while (iDUT.bck_spd != 0) begin
        end
        while (iDUT.lft_spd != 0) begin
        end
        while (iDUT.rght_spd != 0) begin
        end
        disable timeout_motors_off;
      end

    join

    cmd2send = CALIBRATE; // Calibrate command - calibrate the copter so we're ready for another launch
    create_cmd();
    rcv_ack();
    // test if the motor speeds are the CAL_SPEED from the flight control - we need to actually self check this value
    fork

      begin : timeout_calibrate
        repeat (200000) @(posedge clk);
        $error("Error waiting for the motors to turn off after motors_off cmd is asserted.\n");
        $stop();
      end

      begin
        // busy wait on these values while we wait for them to change to CAL_SPEED
        while (iDUT.frnt_spd != iDUT.ifly.CAL_SPEED) begin
        end
        while (iDUT.bck_spd != iDUT.ifly.CAL_SPEED) begin
        end
        while (iDUT.lft_spd != iDUT.ifly.CAL_SPEED) begin
        end
        while (iDUT.rght_spd != iDUT.ifly.CAL_SPEED) begin
        end
        disable timeout_calibrate;
      end

    join

    $display("Landing sequence passed!\n");

  endtask

endmodule