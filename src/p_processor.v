`timescale 1ns/10ps

// AD first attempt at processor.  P. 255 in text.

module p_processor(clk, reset, load_pc, zed, alu_result, pc_out); //input: pc counter value; output: instruction

    //signals
    parameter pc_start = 32'h00400020; //this is what we are given for init
    parameter memory_file = "data/bills_branch.dat";
    input clk, reset, load_pc;
    output wire [31:0] zed, alu_result, pc_out;
    // internal DATA wires:
    wire PCSrc, PCWrite, IFID_Write, ControlMuxSel, branch_compare_zf;
    wire [31:0] add_1_out, 
		//add_2_out,
        branch_add_out,
		branch_mux_out, 
        branch_compare_result,
		ins_mem_out,
		ext_out,
		read_data_1,
		read_data_2,
		data_mem_out,
        ForwardB_out;
   
    wire [4:0] 	mux_write_reg;
   
    // internal CONTROL wires:
    // ALU control wires
    wire [2:0] alu_op_in; // check bit numbers
    wire [1:0] ALUOp, ForwardA, ForwardB;
    wire alu_zero;
   
    // CONTROL block single bit
    wire RegDst, 
	 Beq, 
	 MemRead, 
	 MemtoReg,
	 MemWrite,
	 ALUSrc,
	 RegWrite,
	 Bne,// addition to book diagram 
	 Bgtz; // addition to book diagram


    wire [170:0] idex_out;
    wire [31:0] second_a, alu_input_a, second_b, alu_input_b;
    wire [10:0] control_mux_out;
    wire [63:0] ifid_out;
    wire [108:0] exmem_out;
    wire [70:0] memwb_out;

    // mux for branch logic
    gac_mux_32 branch_mux ( // the leftmost mux
	    .sel(PCSrc), 
        .src0(add_1_out), 
        .src1(branch_add_out),
        .z(branch_mux_out)
    );

    //program counter
    register pc( // add a register to be the pc.
        .clk(clk), 
        .areset(reset), 
        .aload(load_pc), 
        .adata(pc_start), //reloads initial value when aload asserted
        .data_in(branch_mux_out), // DEBUG; final output is branch_mux_out
        .write_enable(PCWrite), // want to be able to write at end, always
        .data_out(pc_out) // debug; final value is pc_out
    );

    //first adder (+4)
    adder_32 adder_1 ( // this adder just increments the pc +4 every time
        .a(pc_out), 
        .b(32'h00000004), // constant 4 for incrementing
        .z(add_1_out) 
        );

    //instruction memory
    gac_sram #(.mem_file(memory_file)) ins_mem( 
            .cs(1'b1), // always enable ops
            .oe(1'b1), // always read the ins mem
            .we(1'b0), // never write the ins mem 
            .addr(pc_out), // the address comes from pc
            .din(32'h00000000), // never write the ins mem
            .dout(ins_mem_out) // read out the instruction
    );


    
    ifid_reg_64 IFID(
        .clk(clk), 
        .areset(1'b0), // prev value was reset
        .aload(1'b0), // prev value was load_pc
        .adata(64'b0),
        .data_in({add_1_out[31:0], ins_mem_out[31:0]}), 
        .write_enable(IFID_Write), // want to be able to write at end, always
        .data_out(ifid_out) // ifid_out[0:31] = ins_mem_out, ifid_out[63:32] = add_1_out
    );

    adder_32 branch_add(
        .a({ext_out[29:0], 2'b00}), 
        .b(ifid_out[63:32]), // add_1_out
        .z(branch_add_out)
    );

    ALU branch_compare(
        .ctrl(3'b110), // sub to compare
        .A(read_data_1), 
        .B(read_data_2),
        .shamt(5'b0), // 
        .cout(gnd),
        .ovf(gnd),
        .ze(branch_compare_zf),
        .R(branch_compare_result)
    );

    control_unit control(
    .op_code(ifid_out[31:26]), 
    .reg_dst(RegDst), 
    .alu_src(ALUSrc), 
    .mem_to_reg(MemtoReg), // 
    .reg_write(RegWrite), 
    .mem_read(MemRead), 
    .mem_write(MemWrite), 
    .alu_op(ALUOp), 
    .beq(Beq), 
    .bne(Bne), 
    .bgtz(Bgtz)
    );

    gac_mux_11 control_mux_(
        .sel(ControlMuxSel),
        .src0({RegDst, Beq, Bne, Bgtz, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite}),
        .src1(11'b0),
        .z(control_mux_out)
    );

    // register file
    register_file reg_file(
        .clk(clk), 
        .reset(reset), // prev value was reset
        .read_reg1(ifid_out[25:21]), 
        .read_reg2(ifid_out[20:16]), 
        .write_reg(memwb_out[4:0]), 
        .write_data(zed), //DEBUG - final value is zed
        .write_enable(memwb_out[70]), // from control 
        .read_data1(read_data_1), //DEBUG - final value is read_data_1
        .read_data2(read_data_2)  //DEBUG - final value is read_data_2
        );

     sign_ext extender(
        .a(ifid_out[15:0]),
        .a_ext(ext_out)
    );

    
    register_171 IDEX(
        .clk(clk), 
        .areset(1'b0), //prev value was reset 
        .aload(1'b0), //prev value was load_pc
        .data_in({control_mux_out[10:0], ifid_out[63:32], read_data_1[31:0], read_data_2[31:0], ext_out[31:0], ifid_out[31:0]}), 
        .write_enable(1'b1), // want to be able to write at end, always
        .data_out(idex_out)
    );

    // //second adder (for branch)
    // adder_32 adder_2 ( // this adder just increments the pc +4 every time
    //     .a(idex_out[159:128]), //add_1_out
    //     .b({idex_out[61:32], 2'b00}), // constant 4 for shift (shifting ext_out)
    //     .z(add_2_out) 
    // );


    alu_control_unit alu_control(
        .inst(idex_out[37:32]), //these are the funct bits (LSBs of ext_out)
        .alu_op(idex_out[164:163]), //ALUOp
        .sel(alu_op_in) //actual alu control
    );

    gac_mux_32 alu_src_mux(
        .sel(idex_out[161]), // ALUSrc
        .src0(ForwardB_out),
        .src1(idex_out[63:32]), // ext_out
        .z(alu_input_b)
    );

    ALU alu(
        .ctrl(alu_op_in), 
        .A(alu_input_a), 
        .B(alu_input_b),
        .shamt(idex_out[10:6]), // 
        .cout(gnd),
        .ovf(gnd),
        .ze(alu_zero),
        .R(alu_result)
        );


    // mux for register input
    gac_mux_5 reg_in ( //this needs to be a 5-bit mux
        .sel(idex_out[170]), // RegDst
        .src0(idex_out[20:16]), // 32 vs 5 bits!
        .src1(idex_out[15:11]),
        .z(mux_write_reg)
    );

    
    exmem_reg_109 EXMEM(
        .clk(clk), 
        .areset(1'b0), // prev value was reset
        .aload(1'b0), // prev value was load_pc
        .adata(109'b0),
        .data_in({idex_out[169:165], idex_out[162], idex_out[160], branch_add_out, alu_zero, alu_result, ForwardB_out, mux_write_reg}), 
        .write_enable(1'b1), // want to be able to write at end, always
        .data_out(exmem_out)
    );

    branch_unit branch(
        .beq_f(Beq),
        .bne_f(Bne),
        .bgtz_f(Bgtz),
        .zf(branch_compare_zf),
        .msb(branch_compare_result[31]),
        .br_sel(PCSrc)
    );

   //mymodule modulename(.zero_in(0));
    
    gac_sram #(.mem_file(memory_file)) data_mem (
        .cs(1'b1), //always on
        .oe(exmem_out[105]), //mem_read
        .we(exmem_out[103]), //mem_write
        .addr(exmem_out[68:37]), //alu_result
        .din(exmem_out[36:5]), // alu_input_b
        .dout(data_mem_out)
        );
   

    
    memwb_reg_71 MEMWB(
        .clk(clk),
        .areset(1'b0), // prev value was reset
        .aload(1'b0), // prev value was load_pc
        .adata(71'b0),
        .data_in({exmem_out[102], exmem_out[104], data_mem_out, exmem_out[68:37], exmem_out[4:0]}),
        .write_enable(1'b1), // want to be able to write at end, always
        .data_out(memwb_out)
    );
    


    // the final mux at the end
    gac_mux_32 mux_out ( 
        .sel(memwb_out[69]), //MemtoReg
        .src0(memwb_out[36:5]), // alu_result
        .src1(memwb_out[68:37]), // data_mem_out
        .z(zed)
    );

    
    gac_mux_32 fwd_A_mux(
        .sel(ForwardA[0]),
        .src0(idex_out[127:96]), // read_data_1
        .src1(zed),
        .z(second_a)
    );

    

    gac_mux_32 fwd_A_mux_2_(
        .sel(ForwardA[1]),
        .src0(second_a),
        .src1(exmem_out[68:37]), // alu_result
        .z(alu_input_a)
    );

    
    gac_mux_32 fwd_B_mux(
        .sel(ForwardB[0]),
        .src0(idex_out[95:64]), // read_data_2
        .src1(exmem_out[68:37]), // alu_result
        .z(second_b)
    );

    
    gac_mux_32 fwd_B_mux_2_(
        .sel(ForwardB[1]),
        .src0(second_b),
        .src1(zed),
        .z(ForwardB_out)
    );

    book_forward_unit fwd(
        .EXMEM_RegWrite(exmem_out[102]), 
        .MEMWB_RegWrite(memwb_out[70]),
        .EXMEM_Rd(exmem_out[4:0]), // mux_write_reg
        .IDEX_Rs(idex_out[25:21]),
        .IDEX_Rt(idex_out[20:16]),
        .MEMWB_Rd(memwb_out[4:0]),
        .ForwardA(ForwardA),
        .ForwardB(ForwardB)
    );

    hazard_detection haz(
        .IDEXMemRead(idex_out[166]),
        .IDEXRegisterRt(idex_out[20:16]),
        .IFIDRegisterRs(ifid_out[25:21]),
        .IFIDRegisterRt(ifid_out[20:16]),
        .PCWrite(PCWrite),
        .IFIDWrite(IFID_Write),
        .ControlMuxSel(ControlMuxSel)
    );

endmodule


