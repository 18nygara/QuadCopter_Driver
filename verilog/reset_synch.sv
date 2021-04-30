module reset_synch(RST_n, rst_n, clk);
  input RST_n, clk;
  output rst_n;

  reg rst_n_1, rst_n;

  // this is a double flop with the exact same conditions
  // therefore, it's put together to show the chain clearly
  // for metastabilty issues
  always @(negedge clk, negedge RST_n)
    if (!RST_n) begin
      rst_n_1 <= 1'b0;
      rst_n <= 1'b0;
    end else begin
      rst_n_1 <= 1'b1;
      rst_n <= rst_n_1;
    end

endmodule