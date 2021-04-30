module RemoteComm(clk, rst_n, RX, TX, cmd, data, send_cmd, cmd_sent, resp_rdy, resp, clr_resp_rdy);

	input clk, rst_n;		// clock and active low reset
	input RX;				// serial data input
	input send_cmd;			// indicates to tranmit 24-bit command (cmd)
	input [7:0] cmd;		// 8-bit command to send
	input [15:0] data;		// 16-bit data that accompanies command
	input clr_resp_rdy;		// asserted in test bench to knock down resp_rdy

	output TX;				// serial data output
	output reg cmd_sent;		// indicates transmission of command complete
	output resp_rdy;		// indicates 8-bit response has been received
	output [7:0] resp;		// 8-bit response from DUT

	////////////////////////////////////////////////////
	// Declare any needed internal signals/registers //
	// below including state defINITions            //
	/////////////////////////////////////////////////
	typedef enum reg [1:0] {INIT, TRANSMITTING_DATA1, TRANSMITTING_DATA2, TRANSMITTING_CMD} state_t;
	state_t curr_state, nxt_state;
	wire [7:0] tx_data;
	reg [7:0] top_data_byte, bottom_data_byte;
	reg trmt;

	typedef enum reg [1:0] {SEL_CMD, SEL_UPPER_DATA, SEL_LOWER_DATA} sel_t;
	sel_t sel; // selection used for the UART

	///////////////////////////////////////////////
	// Instantiate basic 8-bit UART transceiver //
	/////////////////////////////////////////////
	UART iUART(.clk(clk), .rst_n(rst_n), .RX(RX), .TX(TX), .tx_data(tx_data), .trmt(trmt),
			   .tx_done(tx_done), .rx_data(resp), .rx_rdy(resp_rdy), .clr_rx_rdy(clr_resp_rdy));
		   
	/////////////////////////////////
	// Implement RemoteComm Below //
	///////////////////////////////

	// data storage flip flops for top and bottom byte of data
	always @(posedge clk, negedge rst_n)
		if (!rst_n)
			top_data_byte <= 8'h00;
		else if (send_cmd) 
			top_data_byte <= data[15:8];

	always @(posedge clk, negedge rst_n)
		if (!rst_n)
			bottom_data_byte <= 8'h00;
		else if (send_cmd)
			bottom_data_byte <= data[7:0];

	// mux for tx selection - we receive 3 bytes and need to store these 3 in separate locations
	assign tx_data = (sel == SEL_CMD) ? cmd :
									(sel == SEL_UPPER_DATA) ? top_data_byte :
									bottom_data_byte;

	// state transition block
	always_comb begin
		nxt_state = curr_state;
		trmt = 1'b0;
		cmd_sent = 1'b0;
		sel = SEL_CMD;
		casex (curr_state)
			TRANSMITTING_CMD:
				if (tx_done) begin // done transmitting command byte, onto the next
					sel = SEL_UPPER_DATA;
					trmt = 1'b1;
					nxt_state = TRANSMITTING_DATA1;
				end else begin
					sel = SEL_CMD;
				end
			TRANSMITTING_DATA1:
				if (tx_done) begin // done transmitting upper data byte, onto the next
					sel = SEL_LOWER_DATA;
					trmt = 1'b1;
					nxt_state = TRANSMITTING_DATA2;
				end else begin
					sel = SEL_UPPER_DATA;
				end
			TRANSMITTING_DATA2:
				if (tx_done) begin // done transmitting lower data byte, we're done
					cmd_sent = 1'b1;
					nxt_state = INIT;
				end else begin
					sel = SEL_LOWER_DATA;
				end
			default: // INIT case
				if (send_cmd) begin
					sel = SEL_CMD;
					trmt = 1'b1;
					nxt_state = TRANSMITTING_CMD;
				end
		endcase
	end

	// curr_state flip flop
	always @(posedge clk, negedge rst_n)
		if (!rst_n)
			curr_state <= INIT;
		else
			curr_state <= nxt_state;

endmodule	
