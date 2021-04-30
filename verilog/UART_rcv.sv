module UART_rcv(clk, rst_n, RX, clr_rdy, rx_data, rdy);

input clk, rst_n, clr_rdy, RX;
output reg [7:0] rx_data;
output reg rdy;

localparam baud_rate = 2604;
localparam baud_rate_half = 1302;

reg [8:0] shift_reg; // MSB will contain stop bit at rdy
reg [11:0] baud_cnt; // baud rate timer for the UART
reg [3:0] bit_cnt; // number of bits we've received
reg rx_flop, rx_flop2; // double flops for meta stability

// outputs of SM
reg load, set_rdy, receive;

wire shift;

typedef enum reg {INIT, RECEIVING} state_t;
state_t curr_state, nxt_state;

// bit_cnt counter
always @(posedge clk, negedge rst_n) begin
    if (!rst_n)
        bit_cnt <= 4'b0000;
    else if (load)
        bit_cnt <= 4'b0000;
    else if (shift)
        bit_cnt <= bit_cnt + 1;
end

// baud_cnt counter
always @(posedge clk, negedge rst_n) begin
    if (!rst_n)
        baud_cnt <= baud_rate_half;
    else if (load)
        baud_cnt <= baud_rate_half;
    else if (shift)
        baud_cnt <= baud_rate;
    else if (receive)
        baud_cnt <= baud_cnt - 1;
end

// RX double flop - these are declared together for clarity of the chain
always @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin // reset high since UART is idle high
        rx_flop <= 1'b1;
        rx_flop2 <= 1'b1;
    end else begin
        rx_flop <= RX; // asynch in - reason for double flop
        rx_flop2 <= rx_flop;
    end
end

// rx_data shift in
always @(posedge clk, negedge rst_n)
    if (!rst_n)
        shift_reg <= 9'h000;
    else if (shift)
        shift_reg <= {rx_flop2, shift_reg[8:1]};

// shift when the baud count clock has reached a full period - onto the next bit
assign shift = (baud_cnt == 12'h000) ? 1'b1 : 1'b0;
assign rx_data = shift_reg[7:0];

// state transition block
always_comb begin
    load = 1'b0;
    set_rdy = 1'b0;
    receive = 1'b0;
    nxt_state = curr_state;
    case (curr_state)
        RECEIVING: begin
            receive = 1;
            if (bit_cnt == 4'hA) begin // move on if we received the start/stop bits and byte of data
                set_rdy = 1;
                nxt_state = INIT;
            end
        end
        default: begin // INIT state
            if (!rx_flop2) begin
                nxt_state = RECEIVING;
                load = 1'b1;
            end
        end
    endcase
end

// curr_state flop
always @(posedge clk, negedge rst_n)
    if (!rst_n)
        curr_state <= INIT;
    else
        curr_state <= nxt_state;

// rdy flop
always @(posedge clk, negedge rst_n)
    if (!rst_n)
        rdy <= 1'b0;
    else if (load || clr_rdy)
        rdy <= 1'b0;
    else if (set_rdy)
        rdy <= 1'b1;

endmodule