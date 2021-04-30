module UART_comm(clk, rst_n, RX, TX, resp, send_resp, resp_sent, cmd_rdy, cmd, data, clr_cmd_rdy);

	input clk, rst_n;		// clock and active low reset
	input RX;				// serial data input
	input send_resp;		// indicates to transmit 8-bit data (resp)
	input [7:0] resp;		// byte to transmit
	input clr_cmd_rdy;		// host asserts when command digested

	output TX;				// serial data output
	output resp_sent;		// indicates transmission of response complete
	output reg cmd_rdy;		// indicates 24-bit command has been received
	output reg [7:0] cmd;		// 8-bit opcode sent from host via BLE
	output reg [15:0] data;	// 16-bit parameter sent LSB first via BLE

	wire [7:0] rx_data;		// 8-bit data received from UART
	wire rx_rdy;			// indicates new 8-bit data ready from UART
	reg clr_rx_rdy;	// output of posedge detector on rx_rdy used to transition SM

	////////////////////////////////////////////////////
	// declare any needed internal signals/registers //
	// below including any state defINITions        //
	/////////////////////////////////////////////////
	typedef enum reg [1:0] {INIT, BOTTOM_BYTE, MIDDLE_BYTE} state_t;
	state_t curr_state, nxt_state;
	reg capture_middle, capture_top, clr_cmd_rdy_i, set_cmd_rdy;

	///////////////////////////////////////////////
	// Instantiate basic 8-bit UART transceiver //
	/////////////////////////////////////////////
	UART iUART(.clk(clk), .rst_n(rst_n), .RX(RX), .TX(TX), .tx_data(resp), .trmt(send_resp),
			   .tx_done(resp_sent), .rx_data(rx_data), .rx_rdy(rx_rdy), .clr_rx_rdy(clr_rx_rdy));
		
	////////////////////////////////
	// Implement UART_comm below //
	//////////////////////////////
	// command capture byte flop
	always @(posedge clk, negedge rst_n)
		if (!rst_n)
			cmd <= 8'h00;
		else if (capture_top)
			cmd <= rx_data;

	// the top data byte flop
	always @(posedge clk, negedge rst_n)
		if (!rst_n)
			data[15:8] <= 8'h00;
		else if (capture_middle)
			data[15:8] <= rx_data;

	// always capture the LSBs as rx_data, as this will be correct at the end
	assign data[7:0] = rx_data;

	// next state determination
	always_comb begin
		capture_middle = 1'b0;
		capture_top = 1'b0;
		clr_cmd_rdy_i = 1'b0;
		clr_rx_rdy = 1'b0;
		set_cmd_rdy = 1'b0;
		nxt_state = curr_state;
		casex (curr_state) 
			MIDDLE_BYTE:
				if (rx_rdy) begin // only capture when the UART is ready
					clr_rx_rdy = 1'b1;
					capture_middle = 1'b1;
					nxt_state = BOTTOM_BYTE;
				end
			BOTTOM_BYTE:
				if (rx_rdy) begin // only capture when the UART is ready
					clr_rx_rdy = 1'b1;
					nxt_state = INIT; // all done with receiving data
					set_cmd_rdy = 1'b1;
				end
			default: // INIT case (captures high byte)
				if (rx_rdy) begin
					clr_rx_rdy = 1'b1;
					capture_top = 1'b1;
					clr_cmd_rdy_i = 1'b1;
					nxt_state = MIDDLE_BYTE;
				end
		endcase
	end

	// curr state ff
	always @(posedge clk, negedge rst_n)
		if (!rst_n)
			curr_state <= INIT;
		else
			curr_state <= nxt_state;

	// rdy ff logic - RS FF
	always @(posedge clk, negedge rst_n)
		if (!rst_n)
			cmd_rdy <= 1'b0;
		else if (clr_cmd_rdy_i || clr_cmd_rdy)
			cmd_rdy <= 1'b0;
		else if (set_cmd_rdy)
			cmd_rdy <= 1'b1;

endmodule	