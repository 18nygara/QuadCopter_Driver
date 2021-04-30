module UART_tx (clk, rst_n, TX, trmt, tx_data, tx_done);

input clk, rst_n, trmt;
input [7:0] tx_data;
output reg TX, tx_done;

localparam baud_rate = 2604;

reg [11:0] baud_cnt; // timer for baud rate
reg [8:0] tx_shift_reg;
reg [3:0] bit_cnt; 

wire shift;

reg load, trnsmttng; // state machine outputs

typedef enum reg {INIT, TRANSMITTING} state_t;
state_t curr_state, nxt_state;

// FLOP FOR RESET FLOP
always @(posedge clk, negedge rst_n)
    if (!rst_n) 
        tx_shift_reg <= 9'h1FF;
    else if (load)  // INITialize tx_shift_reg with a start bit and the data
        tx_shift_reg <= {tx_data, 1'b0};
    else if (shift)
        tx_shift_reg <= {1'b1, tx_shift_reg[8:1]}; // shift in 1 so the stop bit is correct

// bit_cnt counter
always @(posedge clk, negedge rst_n) begin
    if (!rst_n)
        bit_cnt <= 4'h0;
    else if (load)
        bit_cnt <= 4'h0;
    else if (shift)
        bit_cnt <= bit_cnt + 1;
end

// baud_cnt counter
always @(posedge clk, negedge rst_n) begin
    if (!rst_n)
        baud_cnt <= 12'h000;
    else if (load || shift)
        baud_cnt <= 12'h000;
    else if (trnsmttng)
        baud_cnt <= baud_cnt + 1;
end

// signal that the next bit is ready to be transmitted
assign shift = (baud_cnt[11:0] == baud_rate) ? 1'b1 : 1'b0;
assign TX = tx_shift_reg[0];

// STATE TRANSITION LOGIC
always_comb begin
    nxt_state = INIT;
    load = 1'b0;
    trnsmttng = 1'b0;
    case(curr_state)
        TRANSMITTING: 
            if (bit_cnt == 10)
                nxt_state = INIT;
            else begin
                nxt_state = TRANSMITTING;
                trnsmttng = 1'b1;
            end
        default:
            if (trmt) begin
                nxt_state = TRANSMITTING;
                load = 1'b1;
            end
    endcase
end

// STATE MACHINE
always @(posedge clk, negedge rst_n)
    if (!rst_n)
        curr_state <= INIT;
    else
        curr_state <= nxt_state;

// TX_DONE FLOP
always @(posedge clk, negedge rst_n)
    if (!rst_n)
        tx_done <= 1'b0;
    else if (trmt)
        tx_done <= 1'b0;
    else if (bit_cnt == 10)
        tx_done <= 1'b1;

endmodule