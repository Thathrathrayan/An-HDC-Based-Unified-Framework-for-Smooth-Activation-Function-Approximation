`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: ri5cy_uni_act_fns_tb
// Description:
//   Multi-job RI5CY + HDC activation integration test.
//   Each job executes:
//      custom instruction -> lw packed operand -> NPAD NOPs
//
//   Packed operand format:
//      data_word[15:0]  = x_in signed Q6.10
//      data_word[17:16] = func_sel
//
//   func_sel:
//      2'b00 = swish
//      2'b01 = gelu
//      2'b10 = tanh
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
    wire        core_busy_o;
    wire        top_done;
    wire signed [15:0] y;

    top #(
        .INSTR_RDATA_WIDTH(128),
        .ADDR_WIDTH(14),          // dp_ram = 2**(14-2) = 4096 bytes
        .BOOT_ADDR('h80)
    ) uut (
        .clk_i(clk_i),
        .rstn_i(rstn_i),
        .irq_i(irq_i),

        .debug_req_i(debug_req_i),
        .debug_gnt_o(debug_gnt_o),
        .debug_rvalid_o(debug_rvalid_o),
        .debug_addr_i(debug_addr_i),
        .debug_we_i(debug_we_i),
        .debug_wdata_i(debug_wdata_i),
        .debug_rdata_o(debug_rdata_o),
        .debug_halted_o(debug_halted_o),

        .load_i(1'b0),
        .fetch_enable_i(fetch_enable_i),
        .core_busy_o(core_busy_o),
        .top_done(top_done),
        .y(y)
    );

    always #5 clk_i = ~clk_i;

    // With ADDR_WIDTH=14, memory range is 0x000 to 0xFFF.
    // NPAD=88 makes the program end around 0x8F8.
    // So 0xC00 is safely above the program and inside RAM.
    localparam integer DATA_BASE = 'hC00;

    // =====================================================================
    //  TEST VECTORS
    // =====================================================================
    localparam N = 6;

    reg  signed [15:0] xv   [0:N-1];
    reg  [1:0]         fsv  [0:N-1];
    reg                expv [0:N-1];
    reg  signed [15:0] exp  [0:N-1];

    initial begin
        xv[0] =  16'sd1024;  fsv[0] = 2'b10;  expv[0] = 1;  exp[0] =  16'sd1024; // tanh positive -> y=x
        xv[1] =  16'sd8192;  fsv[1] = 2'b00;  expv[1] = 1;  exp[1] =  16'sd8192; // swish out-of-range positive -> y=x
        xv[2] = -16'sd8192;  fsv[2] = 2'b01;  expv[2] = 1;  exp[2] =  16'sd0;    // gelu out-of-range negative -> y=0
        xv[3] =  16'sd2048;  fsv[3] = 2'b10;  expv[3] = 1;  exp[3] =  16'sd2048; // tanh positive -> y=x
        xv[4] =  16'sd512;   fsv[4] = 2'b00;  expv[4] = 1;  exp[4] =  16'sd317;  // swish 512
        xv[5] = -16'sd512;   fsv[5] = 2'b01;  expv[5] = 1;  exp[5] = -16'sd154;  // gelu -512
    end

    // =====================================================================
    //  RESULT MONITOR - catches every top_done pulse
    // =====================================================================
    integer done_count;
    reg signed [15:0] results [0:N-1];

    initial begin
        done_count = 0;
    end

    always @(posedge clk_i) begin
        if (rstn_i && top_done && (done_count < N)) begin
            results[done_count] = y;
            done_count = done_count + 1;
        end
    end

    // =====================================================================
    //  CAPTURE MONITOR - prints what hdc_split actually latches
    // =====================================================================
    integer cap_count;

    initial begin
        cap_count = 0;
    end

    always @(posedge clk_i) begin
        if (rstn_i && (uut.hdc_split_i.state == 2'd1) && uut.hdc_split_i.load) begin
            $display("  [capture %0d @ %0t] data_path=0x%08h -> x_in=%0d func_sel=%0d",
                     cap_count,
                     $time,
                     uut.hdc_split_i.data_path,
                     $signed(uut.hdc_split_i.data_path[15:0]),
                     uut.hdc_split_i.data_path[17:16]);
            cap_count = cap_count + 1;
        end
    end

    // =====================================================================
    //  MEMORY HELPERS
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
        begin
            rd32 = {
                uut.ram_i.dp_ram_i.mem[a+3],
                uut.ram_i.dp_ram_i.mem[a+2],
                uut.ram_i.dp_ram_i.mem[a+1],
                uut.ram_i.dp_ram_i.mem[a+0]
            };
        end
    endfunction

    function automatic signed [15:0] sx16(input [31:0] w);
        begin
            sx16 = w[15:0];
        end
    endfunction

    // =====================================================================
    //  INSTRUCTION CONSTANTS
    // =====================================================================

    // We want x10 = DATA_BASE = 0xC00.
    //
    // 0xC00 cannot be safely loaded with addi x10,x0,0xC00 because addi
    // immediate is signed 12-bit, so 0xC00 means -1024.
    //
    // Therefore:
    //   lui  x10, 0x1          -> x10 = 0x1000
    //   addi x10, x10, -1024   -> x10 = 0x0C00
    localparam [31:0] I_LUI_X10_1       = 32'h00001537; // lui  x10, 0x1
    localparam [31:0] I_ADDI_X10_NEG1K  = 32'hC0050513; // addi x10, x10, -1024

    localparam [31:0] I_CUSTOM          = 32'h007302EB; // custom start pulse
    localparam [31:0] I_NOP             = 32'h00000013; // addi x0, x0, 0
    localparam [31:0] I_JSELF           = 32'h0000006F; // j .

    localparam integer NPAD = 88;

    // lw x11, imm(x10)
    // base lw x11,0(x10) = 32'h00052583
    function [31:0] LW_X11(input integer imm);
        begin
            LW_X11 = ((imm & 32'hFFF) << 20) | 32'h00052583;
        end
    endfunction

    integer j, p, iaddr, daddr;

    task loadProgram;
        begin
            // -------------------------------------------------------------
            // DATA SECTION
            // -------------------------------------------------------------
            daddr = DATA_BASE;

            for (j = 0; j < N; j = j + 1) begin
                // packed word = {14'b0, func_sel[1:0], x_in[15:0]}
                wr32(daddr, {14'b0, fsv[j], xv[j]});
                daddr = daddr + 4;
            end

            // -------------------------------------------------------------
            // INSTRUCTION SECTION
            // -------------------------------------------------------------
            iaddr = 32'h80;

            // x10 = 0xC00
            wr32(iaddr, I_LUI_X10_1);       iaddr = iaddr + 4;
            wr32(iaddr, I_ADDI_X10_NEG1K);  iaddr = iaddr + 4;

            for (j = 0; j < N; j = j + 1) begin
                // custom instruction triggers hdc_split top_start
                wr32(iaddr, I_CUSTOM);      iaddr = iaddr + 4;

                // load packed operand from DATA_BASE + j*4
                wr32(iaddr, LW_X11(j*4));   iaddr = iaddr + 4;

                // Enough NOPs for HDC to finish and return to IDLE
                for (p = 0; p < NPAD; p = p + 1) begin
                    wr32(iaddr, I_NOP);
                    iaddr = iaddr + 4;
                end
            end

            wr32(iaddr, I_JSELF);

            // -------------------------------------------------------------
            // SANITY PRINTS
            // -------------------------------------------------------------
            $display("=====================================================");
            $display(" LAYOUT: program 0x80..0x%0h, data 0x%0h..0x%0h, mem=4096B",
                     iaddr + 3,
                     DATA_BASE,
                     DATA_BASE + N*4 - 1);

            $display(" instr[0x80]=0x%08h (expect 00001537 lui x10,0x1)",
                     rd32(32'h80));
            $display(" instr[0x84]=0x%08h (expect C0050513 addi x10,x10,-1024)",
                     rd32(32'h84));
            $display(" instr[0x88]=0x%08h (expect 007302EB custom)",
                     rd32(32'h88));
            $display(" instr[0x8c]=0x%08h (expect 00052583 lw x11,0(x10))",
                     rd32(32'h8C));

            for (j = 0; j < N; j = j + 1) begin
                $display(" data[0x%0h]=0x%08h  (x_in=%0d func=%0d)",
                         DATA_BASE + j*4,
                         rd32(DATA_BASE + j*4),
                         sx16(rd32(DATA_BASE + j*4)),
                         (rd32(DATA_BASE + j*4) >> 16) & 3);
            end

            $display("=====================================================");
        end
    endtask

    // =====================================================================
    //  MAIN
    // =====================================================================
    integer k, errors, timeout;
    reg [39:0] fname;

    initial begin
        clk_i = 0;
        rstn_i = 0;
        irq_i = 0;

        debug_req_i = 0;
        debug_we_i = 0;
        debug_wdata_i = 0;
        debug_addr_i = 0;

        fetch_enable_i = 0;

        #100;
        rstn_i = 1;

        loadProgram();

        fetch_enable_i = 1;

        timeout = 0;
        while (done_count < N && timeout < 2000000) begin
            @(posedge clk_i);
            timeout = timeout + 1;
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
                    if (results[k] === exp[k]) begin
                        $display(" [%0s] x=%6d -> y=%6d   PASS",
                                 fname, xv[k], results[k]);
                    end else begin
                        $display(" [%0s] x=%6d -> y=%6d   FAIL (exp %0d)",
                                 fname, xv[k], results[k], exp[k]);
                        errors = errors + 1;
                    end
                end else begin
                    $display(" [%0s] x=%6d -> y=%6d   (no check)",
                             fname, xv[k], results[k]);
                end
            end else begin
                $display(" [%0s] x=%6d -> (NO top_done captured!)",
                         fname, xv[k]);
                errors = errors + 1;
            end
        end

        $display("-----------------------------------------------------");

        if (done_count < N) begin
            $display(" WARNING: only %0d/%0d jobs completed (timeout?)",
                     done_count, N);
        end

        $display(" Checked vectors: %0d error(s)", errors);
        $display(" %s",
                 (errors == 0 && done_count == N) ?
                 "ALL PASS" : "CHECK FAILURES ABOVE");
        $display("=====================================================");

        #50;
        $finish;
    end

endmodule
