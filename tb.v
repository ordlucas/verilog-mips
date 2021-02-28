`timescale 1ns / 1ps

module project_tb();
    reg clk, reset;
    
    wire wreg, m2reg, wmem, aluimm, regrt, ewreg, em2reg, ewmem, ealuimm, mwreg, mm2reg, mwmem, wwreg, wm2reg;
    wire [1:0] fwda, fwdb;
    wire [3:0] aluc, ealuc;
    wire [4:0] rn, rs, rd, rt, ern, mrn, wrn;
    wire [5:0] op, func;
    wire [8:0] pc_in, pc_out;
    wire [15:0] imm;
    wire [31:0] imm_ext, qa, qb, do, eqa, eqb, eimm_ext, eb, er, mr, mqb, mdo, wr, wdo, writed, outa, outb;


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

    initial begin
        clk = 1;
        reset = 1;
        #5 reset = 0;
    end
    always begin
        #5 clk <= ~clk;
    end
endmodule
