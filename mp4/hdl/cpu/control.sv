/**
* control.sv
* Control module for RV32I processor
* Author : ECE 411 Staff & Hao Ren (haor2@illinois.edu.cn)
*/
module control
import rv32i_types::*; /* Import types defined in rv32i_types.sv */
(
    input clk,
    input rst,
    input rv32i_opcode opcode,
    input logic [2:0] funct3,
    input logic [6:0] funct7,
    input logic br_en,
    input logic [4:0] rs1,
    input logic [4:0] rs2,
    input logic mem_resp,
    input logic [1:0] real_addr,
    input logic [3:0] mem_byte_enable,
    output pcmux::pcmux_sel_t pcmux_sel,
    output alumux::alumux1_sel_t alumux1_sel,
    output alumux::alumux2_sel_t alumux2_sel,
    output regfilemux::regfilemux_sel_t regfilemux_sel,
    output marmux::marmux_sel_t marmux_sel,
    output cmpmux::cmpmux_sel_t cmpmux_sel,
    output branch_funct3_t cmpop,
    output alu_ops aluop,
    output logic load_pc,
    output logic load_ir,
    output logic load_regfile,
    output logic load_mar,
    output logic load_mdr,
    output logic load_data_out,
    output logic mem_read,
    output logic mem_write
);

/***************** USED BY RVFIMON --- ONLY MODIFY WHEN TOLD *****************/
logic trap;
logic [4:0] rs1_addr, rs2_addr;
logic [3:0] rmask, wmask;
/*****************************************************************************/

branch_funct3_t branch_funct3;
store_funct3_t store_funct3;
load_funct3_t load_funct3;
arith_funct3_t arith_funct3;

assign arith_funct3 = arith_funct3_t'(funct3);
assign branch_funct3 = branch_funct3_t'(funct3);
assign load_funct3 = load_funct3_t'(funct3);
assign store_funct3 = store_funct3_t'(funct3);
assign rs1_addr = rs1;
assign rs2_addr = rs2;

always_comb
begin : trap_check
    trap = '0;
    rmask = '0;
    wmask = '0;

    case (opcode)
        op_lui, op_auipc, op_imm, op_reg, op_jal, op_jalr:;

        op_br: begin
            case (branch_funct3)
                beq, bne, blt, bge, bltu, bgeu:;
                default: trap = '1;
            endcase
        end

        op_load: begin
            case (load_funct3)
                lw: rmask = 4'b1111;
                lh, lhu: 
                begin 
                    unique case(real_addr) 
                        0: rmask = 4'b0011;
                        2: rmask = 4'b1100;
                    endcase
                end
                lb, lbu: 
                begin 
                    unique case(real_addr)
                        0: rmask = 4'b0001;
                        1: rmask = 4'b0010;
                        2: rmask = 4'b0100;
                        3: rmask = 4'b1000;
                    endcase
                end
                default: trap = '1;
            endcase
        end

        op_store: begin
            case (store_funct3)
                sw: wmask = 4'b1111;
                sh: 
                begin 
                    unique case(real_addr) 
                        0: wmask = 4'b0011;
                        2: wmask = 4'b1100;
                    endcase
                end
                sb: 
                begin 
                    unique case(real_addr)
                        0: wmask = 4'b0001;
                        1: wmask = 4'b0010;
                        2: wmask = 4'b0100;
                        3: wmask = 4'b1000;
                    endcase
                end
                default: trap = '1;
            endcase
        end

        default: trap = '1;
    endcase
end
/*****************************************************************************/

enum int unsigned {
    /* List of states */
    FETCH_1, // MAR <- Mem[PC]
    FETCH_2, // mem_resp == 0 ? FETCH_2 : FETCH_3, MDR <- Mem[MAR], mem_read = 1
    FETCH_3, // IR <- MDR
    DECODE, // if(opcode == LOAD || opcode == STORE) next_states = CALC_ADDR;
    CALC_ADDR, // MAR <- rs1 + SignExt(imm), mem_data_out <- rs2 
    LD_1, // mem_resp == 0 ? LD_1 : LD_2, MDR <- Mem[MAR] (mem_read = 1)
    LD_2, // Reg[rd] <- MDR
    AUIPC, // Reg[rd] <- PC + SignExt(imm)
    BR,
    ST_1, // mem_resp == 0 ? ST_1 : ST_2,  Mem[MAR] <- mem_wdata (mem_write = 1)
    ST_2, // load_pc
    LUI, // Reg[rd] <- u_imm
    IMM,
    JAL // Reg[rd] <- PC + 4, PC <- PC + SignExt(imm)
} state, next_states;

/************************* Function Definitions *******************************/
/**
 *  You do not need to use these functions, but it can be nice to encapsulate
 *  behavior in such a way.  For example, if you use the `loadRegfile`
 *  function, then you only need to ensure that you set the load_regfile bit
 *  to 1'b1 in one place, rather than in many.
 *
 *  SystemVerilog functions must take zero "simulation time" (as opposed to 
 *  tasks).  Thus, they are generally synthesizable, and appropraite
 *  for design code.  Arguments to functions are, by default, input.  But
 *  may be passed as outputs, inouts, or by reference using the `ref` keyword.
**/

/**
 *  Rather than filling up an always_block with a whole bunch of default values,
 *  set the default values for controller output signals in this function,
 *   and then call it at the beginning of your always_comb block.
**/
function void set_defaults();
    // select signals don't matter since if load is 0, they are ignored
    // if they matter, they will be set in the state_actions block
    load_pc = '0;
    load_ir = '0;
    load_regfile = '0;
    load_mar = '0;
    load_mdr = '0;
    load_data_out = '0;
    pcmux_sel = 'x;
    alumux1_sel = 'x;
    alumux2_sel = 'x;
    regfilemux_sel = 'x;
    marmux_sel = 'x;
    cmpmux_sel = 'x;
    cmpop = 'x;
    aluop = 'x;
    mem_read = '0;
    mem_write = '0;
endfunction

/**
 *  Use the next several functions to set the signals needed to
 *  load various registers
**/
function void loadPC(pcmux::pcmux_sel_t sel);
    load_pc = 1'b1;
    pcmux_sel = sel;
endfunction

function void loadRegfile(regfilemux::regfilemux_sel_t sel);
    regfilemux_sel = sel;
    load_regfile = 1'b1;
endfunction

function void loadMAR(marmux::marmux_sel_t sel);
    marmux_sel = sel;
    load_mar = 1'b1;
endfunction

function void loadMDR();
    mem_read = 1'b1;
    load_mdr = 1'b1;
endfunction

function void loadIR();
    load_ir = 1'b1;
endfunction

function void setALU(alumux::alumux1_sel_t sel1, alumux::alumux2_sel_t sel2, logic setop, alu_ops op);
    /* Student code here */

    alumux1_sel = sel1;
    alumux2_sel = sel2;
    if (setop)
        aluop = op; // else default value
endfunction

function automatic void setCMP(cmpmux::cmpmux_sel_t sel, branch_funct3_t op);
    cmpmux_sel = sel;
    cmpop = op;
endfunction

/*****************************************************************************/

    /* Remember to deal with rst signal */

always_comb
begin : state_actions
    /* Default output assignments */
    set_defaults();
    /* Actions for each state */
    unique case (state)
        FETCH_1 : loadMAR(marmux::pc_out);
        FETCH_2 : loadMDR();
        FETCH_3 : loadIR();
        DECODE :;
        CALC_ADDR : 
        begin 
            if(opcode == op_load)
                setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_add);
            else
                setALU(alumux::rs1_out, alumux::s_imm, 1'b1, alu_add);
            loadMAR(marmux::alu_out);
            if(opcode == op_store)
                load_data_out = 1'b1;
        end
        LD_1 : loadMDR();
        LD_2 : 
        begin
            unique case (load_funct3)
                lb: loadRegfile(regfilemux::lb);
                lh: loadRegfile(regfilemux::lh);
                lw: loadRegfile(regfilemux::lw);
                lbu: loadRegfile(regfilemux::lbu);
                lhu: loadRegfile(regfilemux::lhu);
            endcase
            loadPC(pcmux::pc_plus4); // last state
        end
        AUIPC :
        begin
            setALU(alumux::pc_out, alumux::u_imm, 1'b1, alu_add);
            loadRegfile(regfilemux::alu_out);
            loadPC(pcmux::pc_plus4); // last state
        end
        BR :
        begin
            setCMP(cmpmux::rs2_out, branch_funct3);
            if(br_en) begin
                setALU(alumux::pc_out, alumux::b_imm, 1'b1, alu_add);
                loadPC(pcmux::alu_out);

            end
            else
                loadPC(pcmux::pc_plus4); // last state
        end
        ST_1 :
        begin
            mem_write = 1'b1;
        end
        ST_2 : loadPC(pcmux::pc_plus4); // last state
        LUI :
        begin
            loadRegfile(regfilemux::u_imm);
            loadPC(pcmux::pc_plus4); // last state
        end
        JAL :
        begin 
            if(opcode == op_jal) 
            begin
                setALU(alumux::pc_out, alumux::j_imm, 1'b1, alu_add);
                loadPC(pcmux::alu_out);
                loadRegfile(regfilemux::pc_plus4);
            end
            else if(opcode == op_jalr)
            begin
                setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_add);
                loadPC(pcmux::alu_mod2);
                loadRegfile(regfilemux::pc_plus4);
            end
        end
        IMM : // @todo, currently reg-imm, needs to add reg-reg
        begin
            if(opcode == op_imm) 
            begin
                unique case (arith_funct3)
                    add : 
                    begin 
                        setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_add);
                        loadRegfile(regfilemux::alu_out);
                    end
                    slt : 
                    begin
                        setCMP(cmpmux::i_imm, blt);
                        loadRegfile(regfilemux::br_en);
                    end
                    sltu :
                    begin
                        setCMP(cmpmux::i_imm, bltu);
                        loadRegfile(regfilemux::br_en);
                    end
                    aand :
                    begin 
                        setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_and);
                        loadRegfile(regfilemux::alu_out);
                    end
                    aor :
                    begin 
                        setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_or);
                        loadRegfile(regfilemux::alu_out);
                    end
                    axor :
                    begin 
                        setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_xor);
                        loadRegfile(regfilemux::alu_out);
                    end
                    sll :
                    begin 
                        setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_sll);
                        loadRegfile(regfilemux::alu_out);
                    end
                    sr :
                    begin 
                        if(funct7[5] == 1'b0)
                            setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_srl);
                        else if(funct7[5] == 1'b1)
                            setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_sra);
                        loadRegfile(regfilemux::alu_out);
                    end
                endcase
            end
            else if(opcode == op_reg)
            begin 
                unique case(arith_funct3)
                    add : 
                    begin 
                        if(funct7[5] == 1'b0)
                            setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_add);
                        else 
                            setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_sub);
                        loadRegfile(regfilemux::alu_out);
                    end
                    slt : 
                    begin
                        setCMP(cmpmux::rs2_out, blt);
                        loadRegfile(regfilemux::br_en);
                    end
                    sltu :
                    begin
                        setCMP(cmpmux::rs2_out, bltu);
                        loadRegfile(regfilemux::br_en);
                    end
                    aand :
                    begin 
                        setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_and);
                        loadRegfile(regfilemux::alu_out);
                    end
                    aor :
                    begin 
                        setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_or);
                        loadRegfile(regfilemux::alu_out);
                    end
                    axor :
                    begin 
                        setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_xor);
                        loadRegfile(regfilemux::alu_out);
                    end
                    sll :
                    begin 
                        setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_sll);
                        loadRegfile(regfilemux::alu_out);
                    end
                    sr :
                    begin 
                        if(funct7[5] == 1'b0)
                            setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_srl);
                        else if(funct7[5] == 1'b1)
                            setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_sra);
                        loadRegfile(regfilemux::alu_out);
                    end
                endcase

            end

            loadPC(pcmux::pc_plus4); // last state
        end
    endcase
end

always_comb
begin : next_state_logic
    /* Next state information and conditions (if any)
     * for transitioning between states */
    next_states = state;
    unique case (state) 
        FETCH_1 : next_states = FETCH_2;
        FETCH_2 : next_states = mem_resp == 0 ? FETCH_2 : FETCH_3;
        FETCH_3 : next_states = DECODE;
        DECODE :
        begin
            unique case (opcode)
                op_load, op_store : next_states = CALC_ADDR;
                op_auipc : next_states = AUIPC;
                op_br : next_states = BR;
                op_lui : next_states = LUI;
                op_imm, op_reg : next_states = IMM;
                op_jal, op_jalr : next_states = JAL;
                default : next_states = FETCH_1; // @todo
            endcase
        end
        CALC_ADDR : 
        begin
            if(opcode == op_load)
                next_states = LD_1;
            else if(opcode == op_store)
                next_states = ST_1;
            else
                next_states = FETCH_1; 
        end
        LD_1 : next_states = mem_resp == 0 ? LD_1 : LD_2;
        LD_2 : next_states = FETCH_1;
        AUIPC : next_states = FETCH_1;
        BR : next_states = FETCH_1;
        ST_1 : next_states = mem_resp == 0 ? ST_1 : ST_2;
        ST_2 : next_states = FETCH_1;
        LUI : next_states = FETCH_1;
        IMM : next_states = FETCH_1;
        JAL : next_states = FETCH_1;
    endcase
end

always_ff @(posedge clk)
begin: next_state_assignment
    /* Assignment of next state on clock edge */
    if (rst)
        state <= FETCH_1;
    else
        state <= next_states;
end

endmodule : control
