
`default_nettype none
`timescale 1ns/100ps

`define WORD 31:0
`define ADD  6'h00
`define ADDI 6'h08
`define BNE  6'h05
`define LW   6'h23
`define SW   6'h2b
`define HALT 6'h2e
`define NOP  6'h2f
`define CREG 5'd26
  
/************************************************/
module top();
    reg CLK, RST_X;

    initial begin
        CLK = 0;
        forever #50 CLK = ~CLK;
    end

    initial begin
        RST_X = 0;
        #300 RST_X = 1;
    end

   integer i,j;
   
   initial begin /* initialize the instruction memory */
      /*
      p1.imem.mem[0] = {`ADD, 5'd1, 5'd26, 5'd26, 5'd0, 6'h20};
      p1.imem.mem[1] = {`ADDI, 5'd1, 5'd1, 16'd1};
      p1.imem.mem[2] = {`BNE, 5'd1, 5'd2, 16'hfffd};
      p1.imem.mem[3] = {`NOP, 26'd0};
      p1.imem.mem[4] = {`HALT, 26'd0};
      p1.imem.mem[5] = {`NOP, 26'd0};
      p2.imem.mem[0] = {`ADD, 5'd1, 5'd26, 5'd26, 5'd0, 6'h20};
      p2.imem.mem[1] = {`ADDI, 5'd1, 5'd1, 16'd1};
      p2.imem.mem[2] = {`BNE, 5'd1, 5'd2, 16'hfffd};
      p2.imem.mem[3] = {`NOP, 26'd0};
      p2.imem.mem[4] = {`HALT, 26'd0};
      p2.imem.mem[5] = {`NOP, 26'd0};
       */
      
      p1.imem.mem[0] = {`ADD, 5'd1, 5'd3, 5'd3, 5'd0, 6'h20};
      p1.imem.mem[1] = {`ADDI, 5'd1, 5'd1, 16'd1};
      p1.imem.mem[2] = {`BNE, 5'd1, 5'd2, 16'hfffd};
      p1.imem.mem[3] = {`NOP, 26'd0};
      p1.imem.mem[4] = {`HALT, 26'd0};
      p1.imem.mem[5] = {`NOP, 26'd0};
      //for(i=3; i< 126;i++)begin
	 //p1.imem.mem[i] = {`NOP, 26'd0};
      //end
      p2.imem.mem[0] = {`ADD, 5'd1, 5'd3, 5'd3, 5'd0, 6'h20};
      p2.imem.mem[1] = {`ADDI, 5'd1, 5'd1, 16'd1};
      p2.imem.mem[2] = {`BNE, 5'd1, 5'd2, 16'hfffd};
      p2.imem.mem[3] = {`NOP, 26'd0};
      p2.imem.mem[4] = {`HALT, 26'd0};
      p2.imem.mem[5] = {`NOP, 26'd0};
      
      //for(j=3;j<126;j++)begin
	 //p2.imem.mem[j] = {`NOP, 26'd0};
      //end
   end

    initial begin /* initialize the register file */
       p1.regfile.r[0] = 0;
       p1.regfile.r[1] = 0; //i
       p1.regfile.r[2] = 101;
       p1.regfile.r[3] = 0; //sum1 
       p2.regfile.r[0] = 0;
       p2.regfile.r[1] = 0; //i
       p2.regfile.r[2] = 101;
       p2.regfile.r[3] = 0; //sum2  
       common_reg.r[26] = 0; //sum       
    end
        
   initial begin
      #50000 $finish();
   end

    always @(posedge CLK) begin
       $write("P1 : %x %x: %d %d %d %d %d %d \n", p1.pc, p1.Exop,
	      p1.regfile.r[3], common_reg.r[26], p1.stall, write1, p1.clk_num, p1.halt);
       $write("P2 : %x %x: %d %d %d %d %d %d \n", p2.pc, p2.Exop,
	      p2.regfile.r[3], common_reg.r[26], p2.stall, write2, p2.clk_num, p2.halt);
       if(halt1 && halt2) $finish();
    end

   initial begin
      $dumpfile("wave.vcd");
      $dumpvars(0, p1);
   end
   
   wire write1, write2, stall1, stall2, halt1, halt2;
   wire [31:0] rs1, rs2, din1, din2, dst1, dst2, dout1, dout2;
   
   CGPR common_reg(CLK, RST_X, write1, write2, din1, din2, rs1, rs2, dst1, dst2, stall1, stall2, dout1, dout2);

   PROCESSOR p1(CLK, RST_X, stall1, dout1, rs1, dst1, din1, write1, halt1);
   PROCESSOR p2(CLK, RST_X, stall2, dout2, rs2, dst2, din2, write2, halt2);
   
   
endmodule

/****************/
module PROCESSOR(CLK, RST_X, stall, reg_data, common_rs, common_dst, common_data, common_reg_write, halt);
   input CLK, RST_X, stall;
   input [31:0] reg_data;
   output [31:0] common_rs, common_dst, common_data;
   output 	 common_reg_write;
   output 	 halt;

   assign common_reg_write = ((Exop == `ADD) || (Exop == `ADDI) || (Exop == `LW)) && (Exrd == `CREG);
   assign common_rs = `CREG;
   assign common_dst = `CREG;
   assign common_data = alu_result;
   
   reg [31:0] pc;
   reg [31:0] clk_num;
   
   //----------IF Stage----------//
   wire [31:0] 	Ifir;
   wire [31:0] 	tpc;
   wire [31:0] 	npc;
   wire [5:0] 	Ifop = Ifir[31:26]; //for debug
   
   MEM imem(CLK, pc, 0, 1'b0, Ifir); /* instruction memory */
   always @(posedge CLK) begin
      if(!stall) begin 
	 if(!RST_X) pc <= 0;
	 else pc <= (pc_src) ? tpc : npc;
      end
   end
   always @(posedge CLK) begin
      if(!RST_X) clk_num <= 0;
      else clk_num <= clk_num+1;
   end
   reg [31:0] 	IfId_ir;
   reg [31:0] 	IfId_npc; 	
   always @(posedge CLK) begin
      if(!stall)begin
	 IfId_ir <= (!RST_X) ? {`NOP, 26'd0} : Ifir;
	 IfId_npc <= (!RST_X) ? 0 : pc + 4;
      end
   end
      
   //----------ID Stage----------//
   wire [31:0]  Idnpc = IfId_npc;
   wire [31:0] 	Idir = IfId_ir;
   wire [4:0] 	Idrs = IfId_ir[25:21];
   wire [4:0] 	Idrt = IfId_ir[20:16];
   wire [4:0] 	Idrd = IfId_ir[15:11];
   wire [5:0] 	Idop = IfId_ir[31:26];
   wire signed [15:0] Idimm = Idir[15:0];
   wire signed [31:0] Ideximm = {{16{Idimm[15]}}, Idimm};
   wire signed [31:0] Idshiftimm = (Ideximm << 2);
   wire 	reg_write = ((Wbop == `ADD) || (Wbop == `ADDI) || (Wbop == `LW));
   
   wire [31:0] 	 Idrrs, Idrrt, Idrrs_local, Idrrt_local;
   
   GPR regfile(CLK, Idrs, Idrt, Wbdst, Wbres, reg_write, Idrrs_local, Idrrt_local);
   
   assign Idrrs = (Idrs == `CREG) ? reg_data : Idrrs_local;
   assign Idrrt = (Idrt == `CREG) ? reg_data : Idrrt_local;
   
   reg [31:0] 	 IdEx_rrs, IdEx_rrt, IdEx_ir;
   reg [4:0] 	 IdEx_rd;
   always @(posedge CLK) begin
      if(!stall) begin
	 IdEx_rrs <= (!RST_X) ? 0 : Idrrs;
	 IdEx_rrt <= (!RST_X) ? 0 : Idrrt;
	 IdEx_rd <= (!RST_X) ? 0 : ((Idop == `BNE) ||(Idop == `SW)) ? 
		    31 : (Idop == `ADD) ? Idrd : Idrt; //hold dst or not
	 IdEx_ir <= (!RST_X) ? {`NOP, 26'd0} : Idir;
      end
   end
   
   //decide branch or not
   //data forwarding
   wire signed [31:0] binp1 = (Idrs == `CREG) ? reg_data : (Idrs == Exrd) ? alu_result : (Idrs == Mard) ? 
	Maalures : (Idrs == Wbdst) ? Wbres : Idrrs;
   wire signed [31:0] binp2 =  (Idrt == `CREG) ? reg_data : (Idrt == Exrd) ? alu_result : (Idrt == Mard) ? 
	Maalures : (Idrt == Wbdst) ? Wbres : Idrrt;
   wire 	      pc_src = (Idop == `BNE) && (binp1 != binp2);   
   //set npc and tpc
   assign npc = pc + 4;
   assign tpc = pc + Idshiftimm;
   
   //----------Ex Stage----------//
   //set alu inputs
   wire [31:0] Exir = IdEx_ir;
   wire [5:0] Exop = IdEx_ir[31:26];
   wire signed [31:0] Eximm = {{16{Exir[15]}}, Exir[15:0]};
   wire [4:0] 	      Exrs = IdEx_ir[25:21];
   wire [4:0] 	      Exrt = IdEx_ir[20:16];
   //data forwarding
   wire [31:0] 	      Exrrs = (Exrs == Mard) ? Maalures : (Exrs == Wbdst) ? 
		      Wbres : IdEx_rrs;
   wire [31:0] 	      Exrrt = (Exrt == Mard) ? Maalures : (Exrt == Wbdst) ?
		      Wbres : IdEx_rrt;
   wire [4:0] 	      Exrd = IdEx_rd;
   wire 	      alu_src;
   wire [31:0] 	      alu_inp2;
   assign alu_src = ((Exop == `ADDI) || (Exop == `LW) || (Exop == `SW));
   assign alu_inp2 = (alu_src) ? Eximm : Exrrt;
   wire 	 alu_zero;
   wire [31:0] 	 alu_result;
   wire 	 alu_ctl = 0;
   
   ALU alu(CLK, alu_ctl, Exrrs, alu_inp2, alu_zero, alu_result);
   
   reg [31:0] 	 ExMa_alures, ExMa_rrt, ExMa_ir;
   reg [4:0] 	 ExMa_rd;
   always @(posedge CLK) begin
      if(!stall) begin
	 ExMa_ir <= (!RST_X) ? {`NOP, 26'd0} : Exir;
	 ExMa_alures <= (!RST_X) ? 0 : alu_result;
	 ExMa_rrt <= (!RST_X) ? 0 : Exrrt;
	 ExMa_rd <= (!RST_X) ? 0 : Exrd;
      end  
   end
   
   //----------Ma Stage----------//
   wire [31:0]  Mair = ExMa_ir;
   wire [31:0]  Maalures = ExMa_alures;
   wire [4:0] 	Mard = ExMa_rd;
   wire [31:0] 	writedata = ExMa_rrt;
   wire [5:0] 	Maop = ExMa_ir[31:26];
   wire 	mem_write = (Maop == `SW);
   wire 	mem_read = (Maop == `LW);	
   wire [31:0] 	 mem_result;
   MEM dmem(CLK, Maalures, writedata, mem_write, mem_result);
   reg [31:0] 	 MaWb_memres, MaWb_alures, MaWb_ir;  
   reg [4:0] 	 MaWb_dst;
   always @(posedge CLK) begin
      if(!stall) begin
	 MaWb_ir <= (!RST_X) ? {`NOP, 26'd0} : Mair;
	 MaWb_memres <= (!RST_X) ? 0 : mem_result;
	 MaWb_alures <= (!RST_X) ? 0 : Maalures;
	 MaWb_dst <= (!RST_X) ? 0 : Mard;
      end
   end
	     
   //----------Wb Stage----------//
   //set mem_to_reg
   wire [5:0] 	  Wbop = MaWb_ir[31:26];
   wire mem_to_reg = (Wbop == `LW); //if op is LW, get memres
   wire [31:0] 	  Wbres = (mem_to_reg) ? MaWb_memres : MaWb_alures;
   wire [4:0] 	  Wbdst = MaWb_dst;
   reg 		  halt_reg;

   assign halt = halt_reg;
   
   always @(posedge CLK) begin
      if(!RST_X) begin
	 halt_reg = 0;
      end
      halt_reg = (halt_reg) ? 1 : (Wbop == `HALT);
   end
    
      
endmodule

/****************************/
module MEM(CLK, ADDR, D_IN, D_WE, D_OUT);
   //read address
   input         CLK;
   input [31:0]   ADDR, D_IN;
   input 	  D_WE;
   output [31:0]  D_OUT;
   
   reg [31:0] 	  mem[1024*8-1:0]; // 8K word memory
   
   assign D_OUT  = mem[ADDR[14:2]];
   always @(posedge CLK) if(D_WE) mem[ADDR[14:2]] <= D_IN;
endmodule

/************
/* 32bitx32 2R/1W General Purpose Registers (Register File)                 
/***********/
module GPR(CLK, REGNUM0, REGNUM1, REGNUM2, DIN0, WE0, DOUT0, DOUT1);
   input             CLK;
   input [4:0] 	      REGNUM0, REGNUM1, REGNUM2;
   input [31:0]       DIN0;
   input 	      WE0;
   output [31:0]      DOUT0, DOUT1;
   output 	      common_reg_write;
   output [31:0]      common_data, common_dst;
   
   reg [31:0] 	      r[0:31];
   
   assign DOUT0 = (REGNUM0==0) ? 0 : (REGNUM0==REGNUM2) ? DIN0 : r[REGNUM0];
   assign DOUT1 = (REGNUM1==0) ? 0 : (REGNUM1==REGNUM2) ? DIN0 : r[REGNUM1];
   
   always @(posedge CLK) if(WE0) r[REGNUM2] <= DIN0;
endmodule // GPR

module CGPR(CLK, RST_X, write1, write2, data1, data2, rs1, rs2, dst1, dst2, stall1, stall2, out1, out2);
   input CLK, RST_X, write1, write2;
   input [31:0] data1, data2;
   input [31:0] rs1, rs2, dst1, dst2;
   output 	stall1, stall2;
   output [31:0] out1, out2;
   
   reg [31:0] r[0:31];

   assign stall1 = 0;
   assign stall2 = (!RST_X) ? 0 : (write1 && write2);
   assign out1 = (write1 && (dst1 == rs1)) ? data1 : (write2 && (dst2 == rs1)) ? data2 : r[rs1];
   assign out2 = (write1 && (dst1 == rs2)) ? data1 : (write2 && (dst2 == rs2)) ? data2 : r[rs2];

   always @(posedge CLK) begin
      if(!RST_X)begin
	 r[26] = 0;
      end
      if(RST_X)begin
	 if(write1) r[dst1] <= data1;
	 if((!write1) && write2) r[dst2] <= data2;
      end
   end
   
endmodule

/*************/

module ALU(CLK, ALU_CTL, NUM1, NUM2, ZERO, ALU_RES);
   input CLK;
   input ALU_CTL;
   input [31:0] NUM1, NUM2;
   output 	ZERO;
   output [31:0] 	ALU_RES;

   assign ZERO = (ALU_RES == 0);
   assign ALU_RES = NUM1 + NUM2;
endmodule // ALU
