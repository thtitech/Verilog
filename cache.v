
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
    
    initial begin /* initialize the instruction memory */
       /*
       p.imem.mem[0] = {`ADD, 5'd0, 5'd0, 5'd1, 5'd0, 6'h20};
       p.imem.mem[1] = {`ADD, 5'd0, 5'd0, 5'd2, 5'd0, 6'h20};
       p.imem.mem[2] = {`ADDI, 5'd0, 5'd3, 16'd100};
       p.imem.mem[3] = {`ADD, 5'd1, 5'd2, 5'd1, 5'd0, 6'h20};
       p.imem.mem[4] = {`ADDI, 5'd2, 5'd2, 16'd1};
       p.imem.mem[5] = {`BNE, 5'd2, 5'd3, 16'hfffe};
       p.imem.mem[6] = {`ADD, 5'd1, 5'd2, 5'd1, 5'd0, 6'h20};
       p.imem.mem[7] = {`HALT, 26'd0};
	*/
       /*
       p.imem.mem[0] = {`ADD, 5'd0, 5'd0, 5'd1, 5'd0, 6'h20};
       p.imem.mem[1] = {`ADD, 5'd0, 5'd0, 5'd2, 5'd0, 6'h20};
       p.imem.mem[2] = {`ADDI, 5'd0, 5'd3, 16'd8192};
       p.imem.mem[3] = {`SW, 5'd1, 5'd2, 16'd0};
       p.imem.mem[4] = {`ADDI, 5'd1, 5'd1, 16'd4};
       p.imem.mem[5] = {`ADDI, 5'd2, 5'd2, 16'd1};
       p.imem.mem[6] = {`BNE, 5'd2, 5'd3, 16'hfffc};
       p.imem.mem[7] = {`NOP, 26'd0};
       p.imem.mem[8] = {`ADDI, 5'd0, 5'd1, 16'd0};
       p.imem.mem[9] = {`ADDI, 5'd0, 5'd2, 16'd1};
       p.imem.mem[10] = {`ADDI, 5'd0, 5'd3, 16'd8191};
       p.imem.mem[11] = {`LW, 5'd1, 5'd4, 16'd0};
       p.imem.mem[12] = {`LW, 5'd1, 5'd5, 16'd4};
       p.imem.mem[13] = {`LW, 5'd1, 5'd6, 16'd8};
       p.imem.mem[14] = {`ADD, 5'd4, 5'd5, 5'd7, 5'd0, 6'h20};
       p.imem.mem[15] = {`ADD, 5'd6, 5'd7, 5'd8, 5'd0, 6'h20};
       p.imem.mem[16] = {`SW, 5'd1, 5'd8, 16'd4};
       p.imem.mem[17] = {`ADDI, 5'd1, 5'd1, 16'd4};
       p.imem.mem[18] = {`ADDI, 5'd2, 5'd2, 16'd1};
       p.imem.mem[19] = {`BNE, 5'd2, 5'd3, 16'hfff7};
       p.imem.mem[20] = {`NOP, 26'd0};
       p.imem.mem[21] = {`HALT, 26'd0};
       /*
       p.imem.mem[0] = {`LW, 5'd4, 5'd1, 16'd4}; //lw $s1, 4($s4)
       p.imem.mem[0] = {`SW, 5'd4, 5'd1, 16'd4}; //sw 4($s4), $s1
       p.imem.mem[1] = {`NOP, 26'd0}; //nop
       p.imem.mem[2] = {`LW, 5'd4, 5'd2, 16'd4}; //lw $s2, 4($s4)
       p.imem.mem[3] = {`NOP, 26'd0}; //nop       
       p.imem.mem[4] = {`ADD, 5'd1, 5'd2, 5'd1, 5'd0, 6'h20}; //add $s1 $s1 $s2
       p.imem.mem[5] = {`NOP, 26'd0}; //nop
       p.imem.mem[6] = {`HALT, 26'd0};
	*/
       /*
       p.imem.mem[0] = {`ADD, 5'd0, 5'd0, 5'd1, 5'd0, 6'h20};
       p.imem.mem[1] = {`ADD, 5'd0, 5'd0, 5'd2, 5'd0, 6'h20};
       p.imem.mem[2] = {`ADDI, 5'd0, 5'd3, 16'd8192};//
       p.imem.mem[3] = {`ADDI, 5'd0, 5'd7, 16'd2};
       p.imem.mem[4] = {`SW, 5'd1, 5'd2, 16'd0};
       p.imem.mem[5] = {`ADDI, 5'd1, 5'd1, 16'd4};
       p.imem.mem[6] = {`ADDI, 5'd2, 5'd2, 16'd1};
       p.imem.mem[7] = {`BNE, 5'd2, 5'd3, 16'hfffc};
       p.imem.mem[8] = {`NOP, 26'd0};
	*/
       /*
       p.imem.mem[0] = {`ADDI, 5'd0, 5'd1, 16'd32764};
       p.imem.mem[1] = {`ADDI, 5'd0, 5'd2, 16'd8191};
       p.imem.mem[2] = {`SW, 5'd1, 5'd2, 16'd0};
       p.imem.mem[3] = {`ADDI, 5'd1, 5'd1, 16'hfffc};
       p.imem.mem[4] = {`ADDI, 5'd2, 5'd2, 16'hffff};
       p.imem.mem[5] = {`BNE, 5'd0, 5'd2, 16'hfffd};
       p.imem.mem[6] = {`SW, 5'd1, 5'd2, 16'd0};
       //p.imem.mem[7] = {`HALT, 26'd0};
       
       
       p.imem.mem[7] = {`ADDI, 5'd0, 5'd4, 16'd0};
       p.imem.mem[8] = {`ADDI, 5'd0, 5'd5, 16'd4};//
       p.imem.mem[9] = {`ADDI, 5'd0, 5'd3, 16'd8192};//
       p.imem.mem[10] = {`ADDI, 5'd0, 5'd1, 16'd0};
       p.imem.mem[11] = {`ADDI, 5'd0, 5'd2, 16'd2};
       p.imem.mem[12] = {`LW, 5'd1, 5'd7, 16'd0};
       p.imem.mem[13] = {`LW, 5'd1, 5'd8, 16'd4};
       p.imem.mem[14] = {`LW, 5'd1, 5'd9, 16'd8};
       p.imem.mem[15] = {`LW, 5'd1, 5'd10, 16'd12};
       p.imem.mem[16] = {`LW, 5'd1, 5'd11, 16'd16};
       p.imem.mem[17] = {`LW, 5'd0, 5'd16, 16'd0};
       p.imem.mem[18] = {`ADD, 5'd7, 5'd8, 5'd12, 5'd0, 6'h20};
       p.imem.mem[19] = {`ADD, 5'd12, 5'd9, 5'd13, 5'd0, 6'h20};
       p.imem.mem[20] = {`ADD, 5'd13, 5'd10, 5'd14, 5'd0, 6'h20};
       p.imem.mem[21] = {`ADD, 5'd14, 5'd11, 5'd15, 5'd0, 6'h20};
       p.imem.mem[22] = {`SW, 5'd1, 5'd15, 16'd8};
       p.imem.mem[23] = {`ADD, 5'd15, 5'd16, 5'd16, 5'd0, 6'h20};
       p.imem.mem[24] = {`SW, 5'd0, 5'd16, 16'd0};
       p.imem.mem[25] = {`ADDI, 5'd1, 5'd1, 16'd4};
       p.imem.mem[26] = {`ADDI, 5'd2, 5'd2, 16'd1};
       p.imem.mem[27] = {`BNE, 5'd2, 5'd3, 16'hfff0};
       p.imem.mem[28] = {`NOP, 26'd0};
       p.imem.mem[29] = {`ADDI, 5'd4, 5'd4, 16'd1};       
       p.imem.mem[30] = {`BNE, 5'd4, 5'd5, 16'hffeb};       
       p.imem.mem[31] = {`NOP, 26'd0};
       p.imem.mem[32] = {`HALT, 26'd0};	
	*/
       p.imem.mem[0] = {`ADDI, 5'd0, 5'd3, 16'd10};
       p.imem.mem[1] = {`ADD, 5'd1, 5'd2, 5'd1, 6'h20};
       p.imem.mem[2] = {`ADDI, 5'd2, 5'd2, 16'd1};
       p.imem.mem[3] = {`BNE, 5'd2, 5'd3, 16'hfffd};
       p.imem.mem[4] = {`NOP, 26'd0};
       p.imem.mem[5] = {`HALT, 26'd0};
		       
    end

   
   initial begin /* initialize the instruction memory */
      p.dmem.m[0] = 5;
      p.dmem.m[1] = 5;
      p.dmem.m[2] = 20;
   end 


    initial begin /* initialize the register file */
       p.regfile.r[0] = 0;
       p.regfile.r[1] = 0;
       p.regfile.r[2] = 0;
       p.regfile.r[3] = 0;
       p.regfile.r[4] = 0;
       p.regfile.r[5] = 0;
       p.regfile.r[6] = 0;
       p.regfile.r[7] = 0;
       p.regfile.r[8] = 0;
    end
        
  //  initial begin
  //      #2000 $finish();
  //  end

    always @(posedge CLK) begin
        $write("%x %x: %d %d %d %d %d \n", p.pc, p.Ifop,
	       p.regfile.r[1], p.regfile.r[2],
	       p.stall_num, p.lw_num, p.clk_num);
    end

   initial begin
      $dumpfile("wave.vcd");
      $dumpvars(0, p);
   end

    PROCESSOR_01 p(CLK, RST_X);
endmodule

/****************/
module PROCESSOR_01(CLK, RST_X);
   input CLK, RST_X;   
   reg [31:0] pc;
   wire       stall;
   reg [31:0] clk_num;
   
   //----------IF Stage----------//
   wire [31:0] 	Ifir;
   wire [31:0] 	tpc;
   wire [31:0] 	npc;
   wire [5:0] 	Ifop = Ifir[31:26]; //for debug
   
   MEM imem(CLK, pc, 0, 4'b0, Ifir); /* instruction memory */
   always @(posedge CLK) begin
      if(!stall)begin
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
   reg [31:0] 	lw_num;
   reg [31:0] 	stall_num;
   always @(posedge CLK) begin
      stall_num <= (!RST_X) ? 0 : (stall && (Maop == `LW)) ? stall_num+1 : stall_num;
      if(!stall)begin
	 IfId_ir <= (!RST_X) ? 0 : Ifir;
	 IfId_npc <= (!RST_X) ? 0 : pc + 4;
	 lw_num <= (!RST_X) ? 0 : (Ifop == `LW) ? lw_num+1 : lw_num;
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
   
   wire [31:0] 	 Idrrs, Idrrt;
   GPR regfile(CLK, Idrs, Idrt, Wbdst, Wbres, reg_write, Idrrs, Idrrt);

   reg [31:0] 	 IdEx_rrs, IdEx_rrt, IdEx_ir;
   reg [4:0] 	 IdEx_rd;
   always @(posedge CLK) begin
      if(!stall) begin
	 IdEx_rrs <= (!RST_X) ? 0 : Idrrs;
	 IdEx_rrt <= (!RST_X) ? 0 : Idrrt;
	 IdEx_rd <= (!RST_X) ? 0 : ((Idop == `BNE) ||(Idop == `SW)) ? 
		    31 : (Idop == `ADD) ? Idrd : Idrt; //hold dst or not
	 IdEx_ir <= (!RST_X) ? 0 : Idir;
      end
   end
   
   //decide branch or not
   //data forwarding
   wire signed [31:0] binp1 = (Idrs == Exrd) ? alu_result : (Idrs == Mard) ? 
	Maalures : (Idrs == Wbdst) ? Wbres : Idrrs;
   wire signed [31:0] binp2 =  (Idrt == Exrd) ? alu_result : (Idrt == Mard) ? 
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
	 ExMa_ir <= (!RST_X) ? 0 : Exir;
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
   MAINMEM dmem(CLK, RST_X, Maalures, writedata, mem_write, mem_read, mem_result, stall);
   
   reg [31:0] 	 MaWb_memres, MaWb_alures, MaWb_ir;  
   reg [4:0] 	 MaWb_dst;
   always @(posedge CLK) begin
      if(!stall) begin
	 MaWb_ir <= (!RST_X) ? 0 : Mair;
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
   
   always @(posedge CLK) begin
      if(Wbop == `HALT) $finish();
   end
      
endmodule

//----------Cache and DataMemory----------//
module MAINMEM(CLK, RST_X, addr, din, d_write, d_read, dout, stall);
   input CLK, RST_X;
   input [31:0] addr, din;
   input 	d_write, d_read;
   output [31:0] dout;
   output 	 stall;

   wire 	 en = d_write || d_read;
   wire 	 hit;
   wire [31:0] 	 cout;
   wire [31:0] 	 inp = (d_write) ? din : (cnt == 1) ? m[addr[14:2]] : 0;
   reg [2:0] 	 cnt;
   reg [31:0] 	 m[1024*8-1:0]; // 8K word memory
   
   CACHE cache(CLK, RST_X, addr, inp, (d_write || (cnt == 1)),
	       d_read, cout, hit);

   assign stall = (!RST_X) ? 0 : (!en) ? 0 : ((cnt == 0) && !hit) ? 1 : (cnt > 1) ? 1 : 0;
   assign dout =  (stall) ? 0 : (hit) ? cout :  m[addr[14:2]];
   always @(posedge CLK) begin
      cnt <= (!RST_X) ? 0 : (!en) ? 0 : (cnt == 0 && !hit) ? 7 : (cnt > 0) ? (cnt - 1) : cnt;
      if(!stall && d_write) m[addr[14:2]] <= din;
   end
endmodule // MAINMEM

//----------Cache----------//
module CACHE(CLK, RST_X, addr, cin, c_write, c_read, cout, hit);
   input CLK, RST_X;
   input [31:0] addr, cin;
   input 	c_write, c_read;
   output [31:0] cout;
   output 	 hit;
   reg 		 valid [256*4-1:0];
   reg [1:0] 	 lru[256*4-1:0]; //use LRU algorithm
   reg [21:0] 	 tag[256*4-1:0]; //22bit
   reg [31:0] 	 data[256*4-1:0];

   wire [21:0] 	 addr_tag = addr[31:10];
   wire [7:0] 	 addr_index = addr[9:2];
   wire [10:0] 	 exindex = {3'b0, addr_index}; //simple extend
   wire [10:0] 	 replace = (lru[exindex] == 3) ? exindex : (lru[exindex+256] == 3) ? 
		 (exindex+256) : (lru[exindex+512] == 3) ?
		 (exindex+512) : (exindex+768);
   wire 	 hit1 = (addr_tag == tag[exindex]) && (valid[exindex]);
   wire 	 hit2 = (addr_tag == tag[exindex+256]) && (valid[exindex+256]);
   wire 	 hit3 = (addr_tag == tag[exindex+512]) && (valid[exindex+512]);
   wire 	 hit4 = (addr_tag == tag[exindex+768]) && (valid[exindex+768]);
   assign hit = (hit1 || hit2 || hit3 || hit4) && c_read && !c_write;
   //if data write, return cin
   assign cout = (c_write) ? cin : (hit1) ? data[exindex] : (hit2) ? data[exindex+256] :
		 (hit3) ? data[exindex+512] : (hit4) ? data[exindex+768] : 0;
   integer 	 i,j,k;
   always @(posedge CLK) begin
      if(!RST_X) begin
	 //initialize
	 for(i = 0; i < 1024; i=i+1) begin
	    valid[i] <= 0;
	    data[i] <= 0;
	 end
	 for(j = 0; j < 256; j=j+1)begin
	    lru[j] <= 3;
	    lru[j+256] <= 2;
	    lru[j+512] <= 1;
	    lru[j+768] <= 0;
	 end
      end else if(c_write) begin
	 for(k = 0; k < 4; k=k+1) begin
	    lru[exindex+k*256] <= (lru[exindex+k*256] + 1) % 4; //set lru
	 end
	 //set data
	 data[replace] <= cin;
	 tag[replace] <= addr_tag;
	 valid[replace] <= 1;
      end
   end
endmodule

/****************************/
module MEM(CLK, ADDR, D_IN, D_WE, D_OUT);
   //read address
    input         CLK;
    input  [31:0] ADDR, D_IN;
    input   [3:0] D_WE;
    output [31:0] D_OUT;
      
    reg [31:0] mem[1024*8-1:0]; // 8K word memory

    assign D_OUT  = mem[ADDR[14:2]];
    always @(posedge CLK) if(D_WE) mem[ADDR[14:2]] <= D_IN;
endmodule

/************
/* 32bitx32 2R/1W General Purpose Registers (Register File)                 
/***********/
module GPR(CLK, REGNUM0, REGNUM1, REGNUM2, DIN0, WE0, DOUT0, DOUT1);
    input             CLK;
    input       [4:0] REGNUM0, REGNUM1, REGNUM2;
    input      [31:0] DIN0;
    input             WE0;
    output     [31:0] DOUT0, DOUT1;

    reg [31:0] r[0:31];

    assign DOUT0 = (REGNUM0==0) ? 0 : (REGNUM0==REGNUM2) ? DIN0 : r[REGNUM0];
    assign DOUT1 = (REGNUM1==0) ? 0 : (REGNUM1==REGNUM2) ? DIN0 : r[REGNUM1];

    always @(posedge CLK) if(WE0) r[REGNUM2] <= DIN0;
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
