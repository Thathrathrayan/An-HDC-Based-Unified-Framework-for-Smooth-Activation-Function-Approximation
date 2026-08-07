`timescale 1ns / 1ps

module hdc_activation_tb;

parameter D = 1760;
parameter BLOCK_SIZE = 22;
parameter CHUNK_SIZE = 110;
parameter VOCAB_SIZE = 36;
parameter INPUT_STEPS = 41;

reg clk;
reg rst;
reg signed [15:0] x_in;
reg [1:0] func_sel;
reg start;
wire done;
wire signed [15:0] y_out;

// Instantiate UUT with parameters matching python script output
hdc_activation #(
    .D(D),
    .BLOCK_SIZE(BLOCK_SIZE),
    .CHUNK_SIZE(CHUNK_SIZE),
    .VOCAB_SIZE(VOCAB_SIZE),
    .INPUT_STEPS(INPUT_STEPS)
) uut (
    .clk(clk),
    .rst(rst),
    .start(start),
    .x_in(x_in),
    .func_sel(func_sel),
    .done(done),
    .y_out(y_out)
);

always #5 clk = ~clk;

integer file_id;
integer i;
reg signed [15:0] swish_res, gelu_res, tanh_res;

initial begin
    clk = 0;
    rst = 1;
    x_in = 0;
    func_sel = 0;
    start = 0;

    #20;
    rst = 0;
    #20;

    $display("==================================================");
    $display("Starting Hardware Verification Sweep for D=%0d...", D);
    $display("==================================================");

    file_id = $fopen("verilog_results.csv", "w");
    if (file_id == 0) begin
        $display("Error: Could not open verilog_results.csv for writing!");
        $finish;
    end

    $fwrite(file_id, "x_in,swish_out,gelu_out,tanh_out\n");

    for (i = -6144; i <= 6144; i = i + 51) begin
        // --- Swish ---
        @(posedge clk);
        x_in = i; func_sel = 2'b00; start = 1;
        @(posedge clk); start = 0;
        @(posedge clk); while (!done) @(posedge clk);
        swish_res = y_out;

        // --- GELU ---
        @(posedge clk);
        x_in = i; func_sel = 2'b01; start = 1;
        @(posedge clk); start = 0;
        @(posedge clk); while (!done) @(posedge clk);
        gelu_res = y_out;

        // --- Tanhexp ---
        @(posedge clk);
        x_in = i; func_sel = 2'b10; start = 1;
        @(posedge clk); start = 0;
        @(posedge clk); while (!done) @(posedge clk);
        tanh_res = y_out;

        $fwrite(file_id, "%d,%d,%d,%d\n", i, swish_res, gelu_res, tanh_res);
    end

    $fclose(file_id);$display("==================================================");
    $display("Sweep Complete. Results saved to verilog_results.csv");
    $display("==================================================");
    $finish;
end


endmodule
