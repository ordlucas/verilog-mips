`timescale 1ns / 1ps
module top(clk, reset, writed);
    input clk, reset;
    output [31:0] writed;
    
    wire [31:0] outa, outb, mr;
    
    wire wreg, m2reg, wmem, aluimm, regrt, ewreg, em2reg, ewmem, ealuimm, mwreg, mm2reg, mwmem, wwreg, wm2reg;
    wire [1:0] fwda, fwdb;
    wire [3:0] aluc, ealuc;
    wire [4:0] rn, rs, rd, rt, ern, mrn, wrn;
    wire [5:0] op, func;
    wire [8:0] pc_in, pc_out;
    wire [15:0] imm;
    wire [31:0] imm_ext, qa, qb, do, eqa, eqb, eimm_ext, eb, er, mqb, mdo, wr, wdo;


    pc pc_inst(pc_in, pc_out, reset, clk);
    pc_adder pc_adder_inst(pc_out, pc_in);
    i_mem i_mem_inst(pc_out, do);
    if_id if_id_inst(do, op, func, rs, rd, rt, imm, clk);
    sign_ext sign_ext_inst(imm, imm_ext);
    control_unit control_unit_inst(op, func, rs, rt, wreg, m2reg, wmem, aluc, aluimm, regrt, mrn, mm2reg, mwreg, ern, em2reg, ewreg, fwda, fwdb);
    mux_5 wmux_inst(regrt, rd, rt, rn);
    reg_file reg_file_inst(rs, rt, wrn, writed, qa, qb, wwreg);
    id_exe id_exe_inst(wreg, m2reg, wmem, aluc, aluimm, rn, outa, outb, imm_ext, ewreg, em2reg, ewmem, ealuc, ealuimm, ern, eqa, eqb, eimm_ext, clk);
    mux_32 alu_mux_inst(ealuimm, eqb, eimm_ext, eb);
    alu alu_inst(ealuc, eqa, eb, er);
    // module exe_mem(ewreg, em2reg, ewmem, ern, er, eqb, mwreg, mm2reg, mwmem, mrn, mr, mqb, clk);
    exe_mem exe_mem_inst(ewreg, em2reg, ewmem, ern, er, eqb, mwreg, mm2reg, mwmem, mrn, mr, mqb, clk);
    data_mem data_mem_inst(mr, mqb, mwmem, mdo);
    mem_wb mem_wb_inst(mwreg, mm2reg, mrn, mr, mdo, wwreg, wm2reg, wrn, wr, wdo, clk);
    mux_32 wb_mux_inst(wm2reg, wr, wdo, writed);
    mux_fwd fwda_inst(fwda, qa, er, mr, mdo, outa);
    mux_fwd fwdb_inst(fwdb, qb, er, mr, mdo, outb);

endmodule

// Program counter
module pc(in, out, reset, clk);
    input [8:0] in;
    input clk, reset;
    output reg [8:0] out;

    always @ (posedge clk) begin
        if (reset) begin
            out <= 0;
        end
        else begin
            out <= in;
        end
    end   
endmodule

// PC incrementer
module pc_adder(in, out);
    input [8:0] in;
    output [8:0] out;
    
    assign out = in + 4;
endmodule

// Instruction memory
module i_mem(a, do);
    input [8:0] a; // address of instruction
    output [31:0] do; // instruction
    
    wire [31:0] memory [0:16]; // instruction memory array
    
    assign memory[0] = 32'b00000000001000100001100000100000; // add $3, $1, $2
    assign memory[4] = 32'b00000001001000110010000000100010; // sub $4, $9, $3
    assign memory[8] = 32'b00000000011010010010100000100101; // or $5, $3, $9
    assign memory[12] = 32'b00000000011010010011000000100110; // xor $6, $3, $9
    assign memory[16] = 32'b00000000011010010011100000100100; // and $7, $3, $9

    assign do = memory[a];
endmodule

// IF/ID register
module if_id(do, op, func, rs, rd, rt, imm, clk);
    input [31:0] do;
    input clk;
    output reg [5:0] op, func;
    output reg [4:0] rs, rd, rt;
    output reg [15:0] imm;
    
    always @ (posedge clk) begin
        op <= do[31:26];
        rs <= do[25:21];
        rt <= do[20:16];
        
        if (do[31:26] == 6'b000000) begin // if r-format
            func <= do[5:0]; // func is last 6 bits
            rd <= do[15:11];
            imm <= 16'bXXXXXXXXXXXXXXXX;
        end
        else begin // if i-format
            imm <= do[15:0];
            func <= 6'bXXXXXX;
            rd <= 5'bXXXXX;
        end
    end
endmodule

// Sign extender
module sign_ext(in, out);
    input [15:0] in;
    output [31:0] out;
    
    assign out = {{16{in[15]}}, in};
endmodule

// ALU control unit
module control_unit(op, func, rs, rt, wreg, m2reg, wmem, aluc, aluimm, regrt, mrn, mm2reg, mwreg, ern, em2reg, ewreg, fwda, fwdb);
    input [5:0] op, func;
    input [4:0] rs, rt, mrn, ern;
    input mm2reg, mwreg, em2reg, ewreg;
    output reg wreg, m2reg, wmem, aluimm, regrt;
    output reg [1:0] fwda, fwdb;
    output reg [3:0] aluc;
    
    always @ (*) begin
        if(op == 6'b000000) begin // r-format
            wreg <= 1;
            wmem <= 0;
            m2reg <= 0;
            regrt <= 0;
            aluimm <= 0;
            
            if (func == 6'b100000) begin // add
                aluc <= 4'b0010;
            end
            else if (func == 6'b100010) begin // subtract
                aluc <= 4'b0110;
            end
            else if (func == 6'b100100) begin // AND
                aluc <= 4'b0000;
            end
            else if (func == 6'b100101) begin // OR
                aluc <= 4'b0001;
            end
            else if (func == 6'b101010) begin // slt
                aluc <= 4'b0111;
            end
            else if (func == 6'b100110) begin // XOR
                aluc <= 4'b1101;
            end
            else begin
                aluc <= 4'bXXXX;
            end
        end
        else if (op == 6'b100011) begin // lw
            wreg <= 1;
            wmem <= 0;
            m2reg <= 1;
            regrt <= 1;
            aluimm <= 1;
            aluc <= 4'b0010;
        end
        else if (op == 6'b101011) begin // sw
            wreg <= 0;
            wmem <= 1;
            m2reg <= 1'bX;
            regrt <= 1'bX;
            aluimm <= 1;
            aluc <= 4'b0010;
        end
        else begin
            wreg <= 1'bX;
            wmem <= 1'bX;
            m2reg <= 1'bX;
            regrt <= 1'bX;
            aluimm <= 1'bX;
            aluc <= 4'bXXXX;
        end
        
        // default fwd selector bits
        fwda <= 2'b00;
        fwdb <= 2'b00;
        
        // Rs = ern
        if(ewreg && rs == ern && ern != 0) begin
            if(em2reg) begin // check for mem to register transfer
                fwda <= 2'b11;
            end
            else begin
                fwda <= 2'b01;
            end
        end
        // Rs = mrn
        else if(mwreg && rs == mrn && mrn != 0) begin
           if(mm2reg) begin
                fwda <= 2'b11;
            end
            else begin
                fwda <= 2'b10;
            end
        end
        
        // Rt = ern
        if(ewreg && rt == ern && ern != 0) begin
            if(em2reg) begin
                fwdb <= 2'b11;
            end
            else begin
                fwdb <= 2'b01;
            end
        end
        // Rt = mrn
        else if(mwreg && rt == mrn && mrn != 0) begin
            if(mm2reg) begin
                fwdb <= 2'b11;
            end
            else begin
                fwdb <= 2'b10 ;
            end
        end
    end
endmodule

// 5-bit multiplexor
module mux_5(sel, rd, rt, out);
    input sel;
    input [4:0] rd, rt;
    output reg [4:0] out;
    
    always @ (*) begin
        if (sel) begin
            out <= rt;
        end
        else begin
            out <= rd;
        end
    end
endmodule

// Forwarding mux
module mux_fwd(sel, q, r, mr, do, out);
    input [1:0] sel;
    input [31:0] q, r, mr, do;
    output reg [31:0] out;
    
    always @(*) begin
        case(sel)
            2'b00: out <= q;
            2'b01: out <= r;
            2'b10: out <= mr;
            2'b11: out <= do;
            default: out <= 31'hXXXXXXXX;
        endcase
    end
endmodule

// Register file
module reg_file(read_a, read_b, writen, writed, qa, qb, write_enable);
    input [4:0] read_a, read_b, writen;
    input [31:0] writed;
    input write_enable;
    output [31:0] qa, qb;

    reg [31:0] file [0:255];
    reg [31:0] out_a, out_b;
    integer i;

    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            file[i] <= 0;
        end
        file[0] <= 32'h00000000;
        file[1] <= 32'hA00000AA;
        file[2] <= 32'h10000011;
        file[3] <= 32'h20000022;
        file[4] <= 32'h30000033;
        file[5] <= 32'h40000044;
        file[6] <= 32'h50000055;
        file[7] <= 32'h60000066;
        file[8] <= 32'h70000077;
        file[9] <= 32'h80000088;
        file[10] <= 32'h90000099;
    end 
    
    always @ (*) begin
        if(write_enable) begin
            file[writen] <= writed;
        end
    end
    
    assign qa = file[read_a];
    assign qb = file[read_b];
endmodule

// ID/EXE register
module id_exe(wreg, m2reg, wmem, aluc, aluimm, rn, qa, qb, imm_ext, ewreg, em2reg, ewmem, ealuc, ealuimm, ern, eqa, eqb, eimm_ext, clk);
    input clk;
    
    input wreg, m2reg, wmem, aluimm;
    input [3:0] aluc;
    input [4:0] rn;
    input [31:0] qa, qb, imm_ext;
    
    output reg ewreg, em2reg, ewmem, ealuimm;
    output reg [3:0] ealuc;
    output reg [4:0] ern;
    output reg [31:0] eqa, eqb, eimm_ext;

    always @ (posedge clk) begin
        ewreg <= wreg;
        em2reg <= m2reg;
        ewmem <= wmem;
        ealuimm <= aluimm;
        
        ealuc <= aluc;
        ern <= rn;
        
        eqa <= qa;
        eqb <= qb;
        eimm_ext <= imm_ext;
    end
endmodule

// 32-bit multiplexor
module mux_32(sel, qb, imm, out);
    input sel;
    input [31:0] qb, imm;
    output reg [31:0] out;
    
    always @ (*) begin
        if (sel) begin
            out <= imm;
        end
        else begin
            out <= qb;
        end
    end
endmodule

// ALU
module alu(ealuc, a, b, r);
    input [3:0] ealuc;
    input [31:0] a, b;
    output reg [31:0] r;

    always @(*) begin
        case(ealuc)
            4'b0000: r <= a & b;
            4'b0001: r <= a | b;
            4'b0010: r <= a + b;
            4'b0110: r <= a - b;
            4'b0111: r <= a < b ? 1 : 0;
            4'b1100: r <= ~(a|b);
            4'b1101: r <= a ^ b;
            default: r <= 32'hXXXXXXXX;
        endcase
    end
endmodule

// EXE/MEM register
module exe_mem(ewreg, em2reg, ewmem, ern, er, eqb, mwreg, mm2reg, mwmem, mrn, mr, mqb, clk);
    input clk;
    
    input ewreg, em2reg, ewmem;
    input [4:0] ern;
    input [31:0] er, eqb;
    
    output reg mwreg, mm2reg, mwmem;
    output reg [4:0] mrn;
    output reg [31:0] mr, mqb;
    
    always @ (posedge clk) begin
        mwreg <= ewreg;
        mm2reg <= em2reg;
        mwmem <= ewmem;
        mrn <= ern;
        mr <= er;
        mqb <= eqb;
    end
endmodule

// data memory
module data_mem(a, di, we, do);
    input we;
    input [31:0] a, di;
    
    output [31:0] do;

    reg [31:0] file [0:255];
    
    initial begin
        file[0] <= 32'hA00000AA;
        file[4] <= 32'h10000011;
        file[8] <= 32'h20000022;
        file[12] <= 32'h30000033;
        file[16] <= 32'h40000044;
        file[20] <= 32'h50000055;
        file[24] <= 32'h60000066;
        file[28] <= 32'h70000077;
        file[32] <= 32'h80000088;
        file[36] <= 32'h90000099;
    end
    
    
    always @ (*) begin
        if(we) begin
            file[a] <= di;
        end
    end
    assign do = file[a];
    
endmodule

// MEM/WB register
module mem_wb(mwreg, mm2reg, mrn, mr, mdo, wwreg, wm2reg, wrn, wr, wdo, clk);
    input clk;
    
    input mwreg, mm2reg;
    input [4:0] mrn;
    input [31:0] mr, mdo;
    
    output reg wwreg, wm2reg;
    output reg [4:0] wrn;
    output reg [31:0] wr, wdo;

    always @ (posedge clk) begin
        wwreg <= mwreg;
        wm2reg <= mm2reg;
        wrn <= mrn;
        wr <= mr;
        wdo <= mdo;
    end
endmodule