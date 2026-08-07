`timescale 1ns / 1ps

// ==========================================================================
// HDC ACTIVATION -- ULTRA-LOW AREA SYNTHESIS (GRANULAR CHUNKS & TIGHT BITS)
// Applies the Double-Base Sliding Window to 22-bit slices per clock cycle.
// VERSION: Old Division-Based Quantization (0.1 steps, [-4.0, 0.0] domain)
// ==========================================================================

module hdc_activation #(
parameter D = 1760,
parameter BLOCK_SIZE = 22,
parameter VOCAB_SIZE = 36,
parameter INPUT_STEPS = 41,
parameter CHUNK_SIZE = 22,             // GRANULAR CHUNKING: 5x smaller adders!
parameter NUM_CHUNKS = D / CHUNK_SIZE  // Takes 80 clock cycles to complete
)(
input wire clk,
input wire rst,
input wire start,
input wire signed [15:0] x_in,
input wire [1:0] func_sel,
output reg signed [15:0] y_out,
output reg done
);

// ==========================================================================
// TRADITIONAL DIVISION QUANTIZATION (0.1 Step Size over [-4.0, 0.0] Domain)
// 4.0 in Q6.10 = 4096. 0.1 step = 102.
// ==========================================================================
wire is_positive = (x_in >= 0);
wire signed [15:0] abs_x = is_positive ? x_in : -x_in;
wire out_of_bounds = (abs_x > 16'd4096);

// Nearest Rounding Division via Hardware Divider
wire [15:0] rounded_index = (abs_x + 16'd51) / 16'd102; 
wire [5:0] raw_index = (rounded_index >= 16'd40) ? 6'd40 : rounded_index[5:0];
wire [5:0] input_index = 6'd40 - raw_index;

wire [6:0] active_func_idx = (func_sel == 2'b00) ? 7'd41 :
                             (func_sel == 2'b01) ? 7'd42 :
                             (func_sel == 2'b10) ? 7'd43 : 7'd41;

// ==========================================================================
// THE "DOUBLE BASE" SLIDING WINDOW
// ==========================================================================
localparam [D-1:0] BASE_VECTOR = {
    22'b1010110111110000001010, // LUT 0
    22'b0101011000000111110111,
    22'b1001001101101011000000,
    22'b1110100000001100010001,
    22'b0101100101111110111011,
    22'b0000101000011000100110,
    22'b0011111110101101101101,
    22'b0100110011101011011100,
    22'b1011000011010111101100,
    22'b1100010110101000100111,
    22'b0000110110010110110000,
    22'b1110111001110101110100,
    22'b0111011101001100001010,
    22'b0101111010010000001001,
    22'b0011001111011001010111,
    22'b1001011101010010111101,
    22'b1100101111000110001111,
    22'b1011001010100000110010,
    22'b0010001100110111001011,
    22'b1011010000101010001110,
    22'b1111010001110110000001,
    22'b0110001010111010000101,
    22'b0100000111100101101001,
    22'b0001000100110100010101,
    22'b1001110110100011111011,
    22'b1110100100010011111110,
    22'b1010011100001001111000,
    22'b0101010001011001100000,
    22'b0010111101000011000110,
    22'b1110111001011010101011,
    22'b0010001010101101011110,
    22'b1101100111001101010100,
    22'b0111100110011111000001,
    22'b1111100011011010110000,
    22'b1100000001000010101000,
    22'b0000111101100100100011,
    22'b0001101011101100010001,
    22'b1010010110100101010001,
    22'b1100011111011101110001,
    22'b1000110011010000010101,
    22'b0000000000001110001101,
    22'b1111101110000101101010,
    22'b1000100011001011000011,
    22'b1110001111111110111110,
    22'b0111100101000000100101,
    22'b1000110000111111011010,
    22'b1100001010011000011010,
    22'b0111111100110001011111,
    22'b0011101100110010101100,
    22'b1100001001101001001111,
    22'b1010100101101000111111,
    22'b1011011000010101000110,
    22'b0101010111010010000110,
    22'b1101111101010111010010,
    22'b1000001001100110010110,
    22'b1111100000111001000100,
    22'b0110110011110111101111,
    22'b1001111100100100011000,
    22'b0010101001010011110001,
    22'b0110000011010101010000,
    22'b0101100000100011110000,
    22'b1111011110110011100000,
    22'b0001011011110101111010,
    22'b1000001010110001111101,
    22'b0011110001011101011001,
    22'b0110010100101110100000,
    22'b1011010011001100110111,
    22'b0111000011101011100110,
    22'b0011110101100101110110,
    22'b1110101010110101000011,
    22'b0110010110000100111110,
    22'b0111001001010000111110,
    22'b0101011110101001010010,
    22'b0011111010111110000010,
    22'b1101011110011110000101,
    22'b0100001010011111101001,
    22'b0011011000000000100001,
    22'b1101111101001000011111,
    22'b1011100010100110011101,
    22'b0010101100101110111010  // LUT 79
};

localparam [2*D-1:0] DOUBLE_BASE = {BASE_VECTOR, BASE_VECTOR};

localparam [D-1:0] SUPER_VECTOR = 1760'b01100011111111011101011010001010010000000010100001100100110100110001010001100110000001000011011000011001110001100000111101001001000010000001111010110111101110101000110100001100111011100111001101100011101100011011000111001011001110100101110111110000011100001010111101101111110100111110000010000001101111011010111001011111110000101110100101111101001011100100000111000100100010101010101000101110010000010011001110101100110011011110000101000001110111110011001110100110101100011011111101110001110111010110100000000001010111010010010001010100111000100101111010000111001011101101111001011010111001001001101100011111000111000101100101000000111101100100000100010011100100000001001111011010011110111100010110110111101101101000110010000011010101111111010001010000010110110111111001000110111000011111001100110011100100101010111110111100100111011100101100110110001011010111110010011110100010011100010000011111001000001100110011111100001010000001000000011111110100000110100010111011101011110100011100110011100111100000011000111001001011111100010001000101010111011110001110001111100100000000000011111110000110011010101100011100000001010001011010101011110101110111111110001000010001100000001111110000001010011111001001011100100001111011110010010100010001110111110101000111101010011100100111010110101000100000001000000100001001101001110011010100110110111011011011010110110011000010000100011111100001010110001000000000100001110011000000010101001000110010111101100100100000111100010100001001000110100110010011110101101001011101010110101100010111001000111111000010110010001110000100111010100101011110101011111101001110100011100010111101011111001001110101100101100011001111001100011100001111110000011101001111101000011000001101111000100111111110000100000011010001101100011100010101;

// ==========================================================================
// STREAMING CHUNK GENERATION (22 BITS AT A TIME)
// Parameterized to scale beautifully. 80-cycle latency, but ultra-low area.
// ==========================================================================
reg [6:0] chunk_counter; // 7-bit counter safely holds up to 80

// Constant bit-slice offset calculation
wire [11:0] input_offset = 12'd1760 - (input_index * 12'd22) + (chunk_counter * CHUNK_SIZE);
wire [11:0] func_offset  = 12'd1760 - (active_func_idx * 12'd22) + (chunk_counter * CHUNK_SIZE);

wire [CHUNK_SIZE-1:0] input_chunk = DOUBLE_BASE[input_offset +: CHUNK_SIZE];
wire [CHUNK_SIZE-1:0] func_chunk  = DOUBLE_BASE[func_offset +: CHUNK_SIZE];
wire [CHUNK_SIZE-1:0] super_chunk = SUPER_VECTOR[chunk_counter * CHUNK_SIZE +: CHUNK_SIZE];

// Compute exact CHUNK_SIZE bits of the XOR datapath per clock cycle
wire [CHUNK_SIZE-1:0] active_noisy_chunk = super_chunk ^ (input_chunk ^ func_chunk);

// Constant offset dictionary generation
wire [CHUNK_SIZE-1:0] output_dict_chunks [0:35];
genvar g;
generate
    for (g = 0; g < 36; g = g + 1) begin : OUT_DICT_GEN
        // Synthesizer calculates this subtraction entirely at compile time
        localparam [11:0] DICT_START_OFFSET = 1760 - ((g + 44) * 22);
        wire [11:0] dict_offset = DICT_START_OFFSET + (chunk_counter * CHUNK_SIZE);
        
        // This maps to 36 tiny 22-bit multiplexers!
        assign output_dict_chunks[g] = DOUBLE_BASE[dict_offset +: CHUNK_SIZE];
    end
endgenerate

// ==========================================================================
// PURE COMBINATIONAL OUTPUT MULTIPLEXER
// ==========================================================================
function [15:0] get_output_value;
    input [5:0] idx;
    begin
        case (idx)
            6'd0: get_output_value = 16'b1111111010011010;
            6'd1: get_output_value = 16'b1111111010100100;
            6'd2: get_output_value = 16'b1111111010101110;
            6'd3: get_output_value = 16'b1111111010111000;
            6'd4: get_output_value = 16'b1111111011000011;
            6'd5: get_output_value = 16'b1111111011001101;
            6'd6: get_output_value = 16'b1111111011010111;
            6'd7: get_output_value = 16'b1111111011100001;
            6'd8: get_output_value = 16'b1111111011101100;
            6'd9: get_output_value = 16'b1111111011110110;
            6'd10: get_output_value = 16'b1111111100000000;
            6'd11: get_output_value = 16'b1111111100001010;
            6'd12: get_output_value = 16'b1111111100010100;
            6'd13: get_output_value = 16'b1111111100011111;
            6'd14: get_output_value = 16'b1111111100101001;
            6'd15: get_output_value = 16'b1111111100110011;
            6'd16: get_output_value = 16'b1111111100111101;
            6'd17: get_output_value = 16'b1111111101001000;
            6'd18: get_output_value = 16'b1111111101010010;
            6'd19: get_output_value = 16'b1111111101011100;
            6'd20: get_output_value = 16'b1111111101100110;
            6'd21: get_output_value = 16'b1111111101110001;
            6'd22: get_output_value = 16'b1111111101111011;
            6'd23: get_output_value = 16'b1111111110000101;
            6'd24: get_output_value = 16'b1111111110001111;
            6'd25: get_output_value = 16'b1111111110011010;
            6'd26: get_output_value = 16'b1111111110100100;
            6'd27: get_output_value = 16'b1111111110101110;
            6'd28: get_output_value = 16'b1111111110111000;
            6'd29: get_output_value = 16'b1111111111000011;
            6'd30: get_output_value = 16'b1111111111001101;
            6'd31: get_output_value = 16'b1111111111010111;
            6'd32: get_output_value = 16'b1111111111100001;
            6'd33: get_output_value = 16'b1111111111101100;
            6'd34: get_output_value = 16'b1111111111110110;
            6'd35: get_output_value = 16'b0000000000000000;
            default: get_output_value = 16'd0;
        endcase
    end
endfunction

// ==========================================================================
// POPCOUNT ACCUMULATION TREE (TIGHTENED BIT-WIDTHS)
// ==========================================================================

// TIGHTENING: 7-bit return safely captures any 22-bit popcount up to 127
function [6:0] popcount_chunk;
    input [CHUNK_SIZE-1:0] vec;
    integer k;
    begin
        popcount_chunk = 0;
        for (k = 0; k < CHUNK_SIZE; k = k + 1) begin
            popcount_chunk = popcount_chunk + vec[k];
        end
    end
endfunction

localparam STATE_IDLE   = 2'd0;
localparam STATE_CALC   = 2'd1;
localparam STATE_ARGMIN = 2'd2;
localparam STATE_DONE   = 2'd3;

reg [1:0] state;

// TIGHTENING: Max distance is 1760. 11 bits can hold 2047! (Saved 108 Flip Flops)
reg [10:0] hamming_distances [0:35];

integer i;
reg [10:0] current_min;
reg [5:0] calculated_best_idx;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state <= STATE_IDLE;
        done <= 0;
        y_out <= 0;
        chunk_counter <= 0;
    end else begin
        case (state)
            STATE_IDLE: begin
                done <= 0;
                chunk_counter <= 0;
                if (start) begin
                    for (i = 0; i < VOCAB_SIZE; i = i + 1) begin
                        hamming_distances[i] <= 0;
                    end
                    state <= STATE_CALC;
                end
            end

            STATE_CALC: begin
                for (i = 0; i < VOCAB_SIZE; i = i + 1) begin
                    hamming_distances[i] <= hamming_distances[i] + 
                        popcount_chunk(active_noisy_chunk ^ output_dict_chunks[i]);
                end
                
                if (chunk_counter == NUM_CHUNKS - 1) begin
                    state <= STATE_ARGMIN;
                end else begin
                    chunk_counter <= chunk_counter + 1;
                end
            end

            STATE_ARGMIN: begin
                current_min = 11'h7FF; // 2047 (Acts as infinity for 1760 max)
                calculated_best_idx = 0;
                for (i = 0; i < VOCAB_SIZE; i = i + 1) begin
                    if (hamming_distances[i] < current_min) begin
                        current_min = hamming_distances[i];
                        calculated_best_idx = i[5:0];
                    end
                end

                if (func_sel == 2'b10) begin
                    if (is_positive) y_out <= x_in;
                    else if (out_of_bounds) y_out <= 0;
                    else y_out <= get_output_value(calculated_best_idx);
                end else begin
                    if (out_of_bounds) y_out <= is_positive ? x_in : 16'd0;
                    else if (is_positive) y_out <= x_in + get_output_value(calculated_best_idx);
                    else y_out <= get_output_value(calculated_best_idx);
                end
                state <= STATE_DONE;
            end

            STATE_DONE: begin
                done <= 1;
                state <= STATE_IDLE;
            end
        endcase
    end
end


endmodule
