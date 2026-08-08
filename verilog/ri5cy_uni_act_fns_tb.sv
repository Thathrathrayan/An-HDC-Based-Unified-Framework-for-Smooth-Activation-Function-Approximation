`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/28/2026 12:09:54 PM
// Design Name: 
// Module Name: ri5cy_uni_act_fns_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module top_tb;

    // ---------------- DUT I/O ----------------
    reg         clk_i, rstn_i;
    reg  [31:0] irq_i;
    reg         debug_req_i, debug_we_i;
    wire        debug_gnt_o, debug_rvalid_o, debug_halted_o;
    reg  [14:0] debug_addr_i;
    reg  [31:0] debug_wdata_i;
    wire [31:0] debug_rdata_o;
    reg         fetch_enable_i;
    wire        core_busy_o, top_done;
    wire signed [15:0] y;

    top #(
        .INSTR_RDATA_WIDTH(128),
        .ADDR_WIDTH(13),          // dp_ram = 2**(13-2) = 2048 bytes
        .BOOT_ADDR('h80)
    ) uut (
        .clk_i(clk_i), .rstn_i(rstn_i), .irq_i(irq_i),
        .debug_req_i(debug_req_i), .debug_gnt_o(debug_gnt_o),
        .debug_rvalid_o(debug_rvalid_o), .debug_addr_i(debug_addr_i),
        .debug_we_i(debug_we_i), .debug_wdata_i(debug_wdata_i),
        .debug_rdata_o(debug_rdata_o), .debug_halted_o(debug_halted_o),
        .load_i(1'b0), .fetch_enable_i(fetch_enable_i),
        .core_busy_o(core_busy_o), .top_done(top_done), .y(y)
    );

    always #5 clk_i = ~clk_i;

    localparam integer DATA_BASE = 'h600;   // above the program, < 2048

    // =====================================================================
    //  TEST VECTORS  (edit these)
    // =====================================================================
    localparam N = 6;
    reg  signed [15:0] xv   [0:N-1];
    reg  [1:0]         fsv  [0:N-1];
    reg                expv [0:N-1];
    reg  signed [15:0] exp  [0:N-1];

    initial begin
        xv[0]= 16'sd1024;  fsv[0]=2'b10;  expv[0]=1;  exp[0]= 16'sd1024; // tanh  +in-range -> y=x
        xv[1]= 16'sd8192;  fsv[1]=2'b00;  expv[1]=1;  exp[1]= 16'sd8192; // swish >4096 pos -> y=x
        xv[2]=-16'sd8192;  fsv[2]=2'b01;  expv[2]=1;  exp[2]= 16'sd0;    // gelu  <-4096   -> y=0
        xv[3]= 16'sd2048;  fsv[3]=2'b10;  expv[3]=1;  exp[3]= 16'sd2048; // tanh  +in-range -> y=x
        xv[4]= 16'sd512;   fsv[4]=2'b00;  expv[4]=0;  exp[4]= 16'sd0;    // swish in-range  (print)
        xv[5]=-16'sd512;   fsv[5]=2'b01;  expv[5]=0;  exp[5]= 16'sd0;    // gelu  neg       (print)
    end

    // =====================================================================
    //  RESULT MONITOR  - catches every top_done pulse
    // =====================================================================
    integer done_count;
    reg  signed [15:0] results [0:N-1];

    initial done_count = 0;
    always @(posedge clk_i) begin
        if (rstn_i && top_done && (done_count < N)) begin
            results[done_count] = y;
            done_count = done_count + 1;
        end
    end

    // =====================================================================
    //  CAPTURE MONITOR - prints what hdc_split actually latches
    //  (relies on internal signal names in hdc_split: state==WAIT(2'd1), load)
    // =====================================================================
    integer cap_count;
    initial cap_count = 0;
    always @(posedge clk_i) begin
        if (rstn_i && (uut.hdc_split_i.state == 2'd1) && uut.hdc_split_i.load) begin
            $display("  [capture %0d @ %0t] data_path=0x%08h -> x_in=%0d func_sel=%0d",
                     cap_count, $time,
                     uut.hdc_split_i.data_path,
                     $signed(uut.hdc_split_i.data_path[15:0]),
                     uut.hdc_split_i.data_path[17:16]);
            cap_count = cap_count + 1;
        end
    end

    // =====================================================================
    //  MEMORY WRITE HELPER
    // =====================================================================
    task wr32(input integer a, input [31:0] d);
        begin
            uut.ram_i.dp_ram_i.mem[a+0] = d[7:0];
            uut.ram_i.dp_ram_i.mem[a+1] = d[15:8];
            uut.ram_i.dp_ram_i.mem[a+2] = d[23:16];
            uut.ram_i.dp_ram_i.mem[a+3] = d[31:24];
        end
    endtask

    function [31:0] rd32(input integer a);
        rd32 = {uut.ram_i.dp_ram_i.mem[a+3], uut.ram_i.dp_ram_i.mem[a+2],
                uut.ram_i.dp_ram_i.mem[a+1], uut.ram_i.dp_ram_i.mem[a+0]};
    endfunction

    // ---- VERIFIED 32-bit instruction constants (see python check) ----
    localparam [31:0] I_LI_X10_600 = 32'h60000513;   // addi x10, x0, 0x600
    localparam [31:0] I_CUSTOM     = 32'h007302EB;   // custom start pulse
    localparam [31:0] I_NOP        = 32'h00000013;   // addi x0, x0, 0
    localparam [31:0] I_JSELF      = 32'h0000006F;   // j .
    localparam        NPAD         = 40;

    // lw x11, imm(x10) : base = lw x11,0(x10)=0x00052583; imm goes in [31:20]
    function [31:0] LW_X11(input integer imm);
        LW_X11 = ((imm & 32'hFFF) << 20) | 32'h00052583;
    endfunction

    integer j, p, iaddr, daddr;

    task loadProgram;
        begin
            // ---- DATA words ----
            daddr = DATA_BASE;
            for (j = 0; j < N; j = j + 1) begin
                wr32(daddr, {14'b0, fsv[j], xv[j]});   // {func_sel, x_in}
                daddr = daddr + 4;
            end

            // ---- INSTRUCTIONS ----
            iaddr = 32'h80;
            wr32(iaddr, I_LI_X10_600); iaddr = iaddr + 4;

            for (j = 0; j < N; j = j + 1) begin
                wr32(iaddr, I_CUSTOM);        iaddr = iaddr + 4;
                wr32(iaddr, LW_X11(j*4));     iaddr = iaddr + 4;
                for (p = 0; p < NPAD; p = p + 1) begin
                    wr32(iaddr, I_NOP);       iaddr = iaddr + 4;
                end
            end
            wr32(iaddr, I_JSELF);

            // ---- read-back / layout sanity ----
            $display("=====================================================");
            $display(" LAYOUT: program 0x80..0x%0h,  data 0x%0h..0x%0h,  mem=2048B",
                     iaddr, DATA_BASE, DATA_BASE + N*4 - 1);
            $display(" instr[0x80]=0x%08h (expect 60000513 li x10,0x600)", rd32(32'h80));
            $display(" instr[0x84]=0x%08h (expect 007302EB custom)",       rd32(32'h84));
            $display(" instr[0x88]=0x%08h (expect 00052583 lw 0)",         rd32(32'h88));
            for (j = 0; j < N; j = j + 1)
                $display(" data[0x%0h]=0x%08h  (x_in=%0d func=%0d)",
                         DATA_BASE + j*4, rd32(DATA_BASE + j*4),
                         $signed(rd32(DATA_BASE + j*4) & 32'hFFFF),
                         (rd32(DATA_BASE + j*4) >> 16) & 3);
            $display("=====================================================");
        end
    endtask

    // =====================================================================
    //  MAIN
    // =====================================================================
    integer k, errors, timeout;
    reg [39:0] fname;

    initial begin
        clk_i=0; rstn_i=0; irq_i=0;
        debug_req_i=0; debug_we_i=0; debug_wdata_i=0; debug_addr_i=0;
        fetch_enable_i=0;

        #100; rstn_i=1;
        loadProgram();
        fetch_enable_i=1;

        timeout = 0;
        while (done_count < N && timeout < 200000) begin
            @(posedge clk_i); timeout = timeout + 1;
        end
        @(posedge clk_i);

        errors = 0;
        $display("=====================================================");
        $display(" HDC-in-core results (%0d jobs, done_count=%0d)", N, done_count);
        $display("-----------------------------------------------------");
        for (k = 0; k < N; k = k + 1) begin
            case (fsv[k])
                2'b00: fname = "swish";
                2'b01: fname = "gelu ";
                2'b10: fname = "tanh ";
                default: fname = "?    ";
            endcase
            if (k < done_count) begin
                if (expv[k]) begin
                    if (results[k] === exp[k])
                        $display(" [%0s] x=%6d -> y=%6d   PASS", fname, xv[k], results[k]);
                    else begin
                        $display(" [%0s] x=%6d -> y=%6d   FAIL (exp %0d)",
                                 fname, xv[k], results[k], exp[k]);
                        errors = errors + 1;
                    end
                end else
                    $display(" [%0s] x=%6d -> y=%6d   (no check)", fname, xv[k], results[k]);
            end else begin
                $display(" [%0s] x=%6d -> (NO top_done captured!)", fname, xv[k]);
                errors = errors + 1;
            end
        end
        $display("-----------------------------------------------------");
        if (done_count < N)
            $display(" WARNING: only %0d/%0d jobs completed (timeout?)", done_count, N);
        $display(" Checked vectors: %0d error(s)", errors);
        $display(" %s", (errors==0 && done_count==N) ? "ALL PASS" : "CHECK FAILURES ABOVE");
        $display("=====================================================");

        #50; $finish;
    end
endmodule
