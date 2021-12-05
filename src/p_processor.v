`timescale 1ns/10ps

// AD first attempt at processor.  P. 255 in text.

module p_processor(clk, reset, load_pc, z, alu_result); //input: pc counter value; output: instruction

    //signals
    parameter pc_start = 32'h00400020; //this is what we are given for init
    parameter memory_file = "data/sort_corrected_branch.dat";
    input clk, reset, load_pc;
    output wire [31:0] z, alu_result;
    // internal DATA wires:
    wire branch_mux_sel;
    wire [31:0] pc_out, 
		add_1_out, 
		add_2_out, 
		branch_mux_out, 
		ins_mem_out,
		ext_out,
		mux_read_reg,
		read_data_1,
		read_data_2,
		data_mem_out;
   
    wire [4:0] 	mux_write_reg;
   
    // internal CONTROL wires:
    // ALU control wires
    wire [2:0] alu_op_in; // check bit numbers
    wire [1:0] ALUOp;
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



    //program counter
    register pc( // add a register to be the pc.
        .clk(clk), 
        .areset(reset), 
        .aload(load_pc), 
        .adata(pc_start), //reloads initial value when aload asserted
        .data_in(branch_mux_out), // DEBUG; final output is branch_mux_out
        .write_enable(1'b1), // want to be able to write at end, always
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


    wire [170:0] ifid_out;
    register_171 IFID(
        .clk(clk), 
        .areset(reset), 
        .aload(load_pc), 
        .adata(171{1'b0}), //
        .data_in({107{1'b0}, add_1_out, ins_mem_out}), 
        .write_enable(1'b1), // want to be able to write at end, always
        .data_out(ifid_out) // ifid_out[0:31] = ins_mem_out, ifid_out[63:32] = add_1_out
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

    // register file
    register_file reg_file(
        .clk(clk), 
        .reset(reset),
        .read_reg1(ifid_out[25:21]), 
        .read_reg2(ifid_out[20:16]), 
        .write_reg(mux_write_reg), // FIX AT END
        .write_data(z), //DEBUG - final value is z
        .write_enable(RegWrite), // from control 
        .read_data1(read_data_1), //DEBUG - final value is read_data_1
        .read_data2(read_data_2)  //DEBUG - final value is read_data_2
        );

     sign_ext extender(
        .a(ifid_out[15:0]),
        .a_ext(ext_out)
    );

    wire [170:0] idex_out;
    register_171 IDEX(
        .clk(clk), 
        .areset(reset), 
        .aload(load_pc), 
        .adata(171{1'b0}), //
        .data_in({RegDst, Beq, Bne, Bgtz, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite,
        ifid_out[63:32], read_data_1, read_data_2, ext_out, ifid_out[31:0}), 
        .write_enable(1'b1), // want to be able to write at end, always
        .data_out(idex_out)
    );

    //second adder (for branch)
    adder_32 adder_2 ( // this adder just increments the pc +4 every time
        .a(idex_out[159:128]), //add_1_out
        .b({idex_out[61:32],2'b00}), // constant 4 for shift (shifting ext_out)
        .z(add_2_out) 
        );

    // mux for register output
    gac_mux_32 read_data_2_mux (
        .sel(idex_out[161]), // ALUSrc
        .src0(idex_out[95:64]), // read_data_2
        .src1(idex_out[63:32]), //ext_out
        .z(mux_read_reg)
    );

     alu_control_unit alu_control(
        .inst(idex_out[37:32]), //these are the funct bits (LSBs of ext_out)
        .alu_op(idex_out[164:163]), //ALUOp
        .sel(alu_op_in) //actual alu control
    );

    ALU alu(
        .ctrl(alu_op_in), 
        .A(idex_out[127:96]), //read_data_1
        .B(mux_read_reg),
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

    wire [170:0] exmem_out;
    register_171 EXMEM(
        .clk(clk), 
        .areset(reset), 
        .aload(load_pc), 
        .adata(171{1'b0}), //
        .data_in({62{1'b0}, idex_out[169:165], idex_out[162], idex_out[160], add_2_out, alu_zero, alu_result, idex_out[95:64], mux_write_reg}), 
        .write_enable(1'b1), // want to be able to write at end, always
        .data_out(exmem_out)
    );

    wire branch_select_wire;
    branch_unit branch(
        .beq_f(exmem_out[108]),
        .bne_f(exmem_out[107]),
        .bgtz_f(exmem_out[106]),
        .zf(exmem_out[69]),
        .msb(exmem_out[68]),
        .br_sel(branch_select_wire)
    );

   //mymodule modulename(.zero_in(0));

    gac_sram #(.mem_file(memory_file)) data_mem (
        .cs(1'b1), //always on
        .oe(MemRead),
        .we(MemWrite),
        .addr(alu_result), // DEBUG - final value is alu_result
        .din(read_data_2), // DEBUG - final value is read_data_2
        .dout(data_mem_out)
        );
    
    
    
    // mux for branch logic
    gac_mux_32 branch_mux ( // the top one in the schematic
	.sel(branch_mux_sel), // from the and gate
        .src0(add_1_out), 
        .src1(add_2_out), 
        .z(branch_mux_out)
    );


    // the final mux at the end
    gac_mux_32 mux_out ( 
        .sel(MemtoReg),
        .src0(alu_result),
        .src1(data_mem_out),
        .z(z)
    );



    wire beq_out;
    gac_and_gate and_1(
        .x(Beq),
        .y(alu_zero),
        .z(beq_out)
    );



    wire not_zero_;
    gac_not_gate not_zero(
        .x(alu_zero),
        .z(not_zero_)
    );

    wire bne_out;
    gac_and_gate bne_and(
        .x(Bne),
        .y(not_zero_),
        .z(bne_out)
    );

    wire or_zf_msb;
    gac_or_gate or_zf_msb_(
        .x(alu_zero),
        .y(read_data_1[31]),
        .z(or_zf_msb)
    );

    wire bgtz_out;
    gac_not_gate not_or_zf_msb(
        .x(or_zf_msb),
        .z(bgtz_out)
    );


    wire or_beq_bne;
    gac_or_gate or_bne_(
        .x(beq_out),
        .y(bne_out),
        .z(or_beq_bne)
    );

    wire bgtz_flag;
    gac_and_gate and_bgtz(
        .x(Bgtz),
        .y(bgtz_out),
        .z(bgtz_flag)
    );

    gac_or_gate branch_sel_bit(
        .x(or_beq_bne),
        .y(bgtz_flag),
        .z(branch_mux_sel)
    );

endmodule


