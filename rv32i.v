module rv32i(
    input clk,
    input reset,
    output [31:0] pc_out,
    output [31:0] instr_out,
    output [31:0] alu_result_out,
    output [31:0] reg_data_out
);

// Program Counter
reg [31:0] pc;
wire [31:0] instr;
wire [31:0] pc_next;

// Instruction Memory: simple ROM (64 words)
reg [31:0] instr_mem [0:63];

// Register File signals
wire [4:0] rs1, rs2, rd;
wire [31:0] rs1_data, rs2_data, rd_data;
reg [31:0] regfile [0:31];

// Control signals
wire [6:0] opcode;
wire [2:0] funct3;
wire [6:0] funct7;
wire [31:0] imm;
wire alu_src;   // Select ALU source (imm or rs2)
wire mem_write;
wire mem_read;
wire reg_write;
wire mem_to_reg; // Writeback from mem or alu_result
wire [3:0] alu_control;

// ALU wires
wire [31:0] alu_input2;
wire [31:0] alu_result;
wire alu_zero;

// Data Memory (simple RAM)
reg [31:0] data_mem [0:63];
wire [31:0] mem_read_data;

// For loops require declaration of integer i at module level
integer i;

// Instruction Fetch logic
assign pc_out = pc;
assign instr_out = instr;
assign alu_result_out = alu_result;
assign reg_data_out = regfile[1];  // for debug: output register x1 data

// Initialize instruction memory (for demo, could load from file)
initial begin
    instr_mem[0] = 32'h00000513; // addi x10, x0, 0
    instr_mem[1] = 32'h00100593; // addi x11, x0, 1
    instr_mem[2] = 32'h00b50633; // add x12, x10, x11
    instr_mem[3] = 32'h00000000; // nop (could be used for halt)
    for (i = 4; i < 64; i = i + 1) begin
        instr_mem[i] = 32'b0;
    end
end

// Instruction fetch: update PC
always @(posedge clk or posedge reset) begin
    if (reset) 
        pc <= 32'b0;
    else 
        pc <= pc_next;
end

assign instr = instr_mem[pc[7:2]]; // 4-byte aligned (word addressing)

// Instruction decode
assign opcode = instr[6:0];
assign rd = instr[11:7];
assign funct3 = instr[14:12];
assign rs1 = instr[19:15];
assign rs2 = instr[24:20];
assign funct7 = instr[31:25];

// Immediate generator (I-type, S-type, B-type)
function [31:0] imm_gen;
    input [31:0] inst;
    reg [31:0] imm_val;
    begin
        case(inst[6:0])
            7'b0010011, // I-type ALU
            7'b0000011: // I-type load
                imm_val = {{20{inst[31]}}, inst[31:20]};
            7'b0100011: // S-type store
                imm_val = {{20{inst[31]}}, inst[31:25], inst[11:7]};
            7'b1100011: // B-type branch
                imm_val = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
            default:
                imm_val = 32'b0;
        endcase
        imm_gen = imm_val;
    end
endfunction

assign imm = imm_gen(instr);

// Register file read
assign rs1_data = regfile[rs1];
assign rs2_data = regfile[rs2];

// ALU source mux: choose between immediate or second register operand
assign alu_src = (opcode == 7'b0010011) || (opcode == 7'b0000011); // I-type ALU or load use immediate
assign alu_input2 = alu_src ? imm : rs2_data;

// ALU control unit (simplified)
function [3:0] alu_ctrl_gen;
    input [6:0] op;
    input [2:0] f3;
    input [6:0] f7;
    begin
        case(op)
            7'b0110011: // R-Type
                case({f7, f3})
                    10'b0000000000: alu_ctrl_gen = 4'b0000; // add
                    10'b0100000000: alu_ctrl_gen = 4'b0001; // sub
                    10'b0000000111: alu_ctrl_gen = 4'b0010; // and
                    10'b0000000110: alu_ctrl_gen = 4'b0011; // or
                    10'b0000000100: alu_ctrl_gen = 4'b0100; // xor
                    10'b0000000001: alu_ctrl_gen = 4'b0101; // sll
                    10'b0000000101: alu_ctrl_gen = 4'b0110; // srl
                    10'b0100000101: alu_ctrl_gen = 4'b0111; // sra
                    default: alu_ctrl_gen = 4'bxxxx;
                endcase
            7'b0010011: // I-Type ALU
                case(f3)
                    3'b000: alu_ctrl_gen = 4'b0000; // addi
                    3'b111: alu_ctrl_gen = 4'b0010; // andi
                    3'b110: alu_ctrl_gen = 4'b0011; // ori
                    3'b100: alu_ctrl_gen = 4'b0100; // xori
                    3'b001: alu_ctrl_gen = 4'b0101; // slli
                    3'b101: alu_ctrl_gen = (f7 == 7'b0000000) ? 4'b0110 : 4'b0111; // srli or srai
                    default: alu_ctrl_gen = 4'bxxxx;
                endcase
            7'b0000011: alu_ctrl_gen = 4'b0000; // load = add for address calculation
            7'b0100011: alu_ctrl_gen = 4'b0000; // store = add for address calculation
            7'b1100011: alu_ctrl_gen = 4'b0001; // branch = sub for comparison
            default: alu_ctrl_gen = 4'bxxxx;
        endcase
    end
endfunction

assign alu_control = alu_ctrl_gen(opcode, funct3, funct7);

// ALU implementation
function [31:0] alu_func;
    input [3:0] ctrl;
    input [31:0] a;
    input [31:0] b;
    reg [31:0] res;
    begin
        case(ctrl)
            4'b0000: res = a + b; // add
            4'b0001: res = a - b; // sub
            4'b0010: res = a & b; // and
            4'b0011: res = a | b; // or
            4'b0100: res = a ^ b; // xor
            4'b0101: res = a << b[4:0]; // sll
            4'b0110: res = a >> b[4:0]; // srl
            4'b0111: res = $signed(a) >>> b[4:0]; // sra
            default: res = 32'b0;
        endcase
        alu_func = res;
    end
endfunction

assign alu_result = alu_func(alu_control, rs1_data, alu_input2);

// Branch taken calculation
wire branch_taken;
assign branch_taken = ((opcode == 7'b1100011) && (
    (funct3 == 3'b000 && alu_result == 0) ||    // beq
    (funct3 == 3'b001 && alu_result != 0)       // bne
));

// Next PC logic (branch or sequential)
assign pc_next = (branch_taken) ? pc + imm : pc + 4;

// Memory control signals
assign mem_write = (opcode == 7'b0100011) ? 1'b1 : 1'b0; // sw
assign mem_read = (opcode == 7'b0000011) ? 1'b1 : 1'b0;  // lw
assign reg_write = ((opcode == 7'b0110011) || (opcode == 7'b0010011) || (opcode == 7'b0000011)) ? 1'b1 : 1'b0;
assign mem_to_reg = (opcode == 7'b0000011) ? 1'b1 : 1'b0;

// Data memory read/write (word aligned)
always @(posedge clk) begin
    if (mem_write)
        data_mem[alu_result[7:2]] <= rs2_data;
end
assign mem_read_data = data_mem[alu_result[7:2]];

// Register file write
integer j;
always @(posedge clk or posedge reset) begin
    if (reset) begin
        for (j = 0; j < 32; j = j + 1)
            regfile[j] <= 32'b0;
    end else if (reg_write && rd != 0) begin
        regfile[rd] <= mem_to_reg ? mem_read_data : alu_result;
    end
end

endmodule
