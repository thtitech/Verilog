
`default_nettype none
`timescale 1ns/100ps

`define WORD 31:0
`define ADD  6'h00
`define ADDI 6'h08
`define BNE  6'h05
`define LW   6'h23
`define SW   6'h2b
  
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
       p.imem.mem[5] = {`BNE, 5'd2, 5'd3, 16'hfffd};
       p.imem.mem[6] = {`ADD, 5'd1, 5'd2, 5'd1, 5'd0, 6'h20};     
	*/
       p.imem.mem[0] = {`ADD, 5'd0, 5'd0, 5'd1, 5'd0, 6'h20};
       p.imem.mem[1] = {`ADD, 5'd0, 5'd0, 5'd2, 5'd0, 6'h20};
       p.imem.mem[2] = {`ADDI, 5'd0, 5'd3, 16'd8192};
       p.imem.mem[3] = {`SW, 5'd1, 5'd2, 16'd0};
       p.imem.mem[4] = {`ADDI, 5'd1, 5'd1, 16'd4};
       p.imem.mem[5] = {`ADDI, 5'd2, 5'd2, 16'd1};
       p.imem.mem[6] = {`BNE, 5'd2, 5'd3, 16'hfffc};
       p.imem.mem[7] = {`ADDI, 5'd0, 5'd1, 16'd0};
       p.imem.mem[8] = {`ADDI, 5'd0, 5'd2, 16'd1};
       p.imem.mem[9] = {`ADDI, 5'd0, 5'd3, 16'd8191};
       p.imem.mem[10] = {`LW, 5'd1, 5'd4, 16'd0};
       p.imem.mem[11] = {`LW, 5'd1, 5'd5, 16'd4};
       p.imem.mem[12] = {`LW, 5'd1, 5'd6, 16'd8};
       p.imem.mem[13] = {`ADD, 5'd4, 5'd5, 5'd7, 5'd0, 6'h20};
       p.imem.mem[14] = {`ADD, 5'd6, 5'd7, 5'd8, 5'd0, 6'h20};
       p.imem.mem[15] = {`SW, 5'd1, 5'd8, 16'd4};
       p.imem.mem[16] = {`ADDI, 5'd1, 5'd1, 16'd4};
       p.imem.mem[17] = {`ADDI, 5'd2, 5'd2, 16'd1};
       p.imem.mem[18] = {`BNE, 5'd2, 5'd3, 16'hfff7};
       //p.imem.mem[20] = {`NOP, 26'd0};
       //p.imem.mem[21] = {`HALT, 26'd0};  
    end

   initial begin /* initialize the instruction memory */
      p.dmem.mem[0] = 5;
      p.dmem.mem[1] = 10;
      p.dmem.mem[2] = 20;
    end

    initial begin /* initialize the register file */
       p.regfile.r[1] = 0; //$s1
       p.regfile.r[2] = 0; //$s2
       p.regfile.r[3] = 3; //number of loop
       p.regfile.r[4] = 0; //base addr
       p.regfile.r[5] = 0;
    end
        
    initial begin
        #20000000 $finish();
    end

    always @(posedge CLK) begin
        $write("%x %x: %d %d\n", p.pc, p.ir, p.regfile.r[1], p.regfile.r[8]);
    end

   initial begin
      $dumpfile("wave.vcd");
      $dumpvars(0, p);
   end

    PROCESSOR_01 p(CLK, RST_X);
endmodule

module PROCESSOR_01(CLK, RST_X);
   input CLK, RST_X;
   reg [31:0] pc;
   wire [31:0] ir;
   wire [5:0] op = ir[31:26];  
   wire [4:0] rs = ir[25:21];
   wire [4:0] rt = ir[20:16];
   wire [4:0] rd = ir[15:11];
   wire signed [15:0] imm = ir[15:0];
   wire signed [31:0] eximm = {{16{imm[15]}}, imm};
   wire [31:0] shift_imm = (eximm << 2);
   wire [5:0] fct = ir[5:0];
   wire [31:0] rrs, rrt;
   wire [31:0] npc, tpc; //tpc is next pc when branched, npc = pc + 4
   wire        alu_zero;
   wire [31:0] alu_result;
   wire [31:0] mem_result;
   wire [31:0] result;
   wire [4:0]  reg_rd;
   wire 	pc_src;
   wire 	alu_src;
   wire 	reg_write;
   wire 	mem_write;
   wire 	mem_to_reg;	
   wire [31:0] 	alu_inp1, alu_inp2;
   wire [5:0]   alu_ctl; //not user

   MEM imem(CLK, pc, 0, 0, ir); /* instruction memory */
   
   always @(posedge CLK) begin
      if(!RST_X) pc <= 0;
      else pc <= (pc_src) ? tpc : npc;
   end

   //make reg_rd and reg_write
   assign reg_rd = (op) ? rt  : rd;
   assign reg_write = (op == `ADD) || (op == `ADDI) || (op == `LW);
   
   GPR regfile(CLK, rs, rt, reg_rd, result, reg_write, rrs, rrt); 

   //decide branch or not
   assign pc_src = (op == `BNE) ? (rrs != rrt) : 0;   
   //set npc and tpc and set alu inputs
   assign npc = pc + 4;
   assign tpc = pc + 4 + shift_imm;
   assign alu_inp1 = rrs;
   assign alu_src = (op == `ADDI) || (op == `LW) || (op == `SW);
   assign alu_inp2 = (alu_src) ? eximm : rrt;
   
   ALU alu(CLK, alu_ctl, alu_inp1, alu_inp2, alu_zero, alu_result);
   
   //set mem_write
   assign mem_write = (op == `SW);

   MEM dmem(CLK, alu_result, rrt, mem_write, mem_result);

   //set mem_to_reg
   assign mem_to_reg = (op == `LW);
   assign result = (mem_to_reg) ? mem_result : alu_result;   
endmodule 

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
module GPR(CLK, REGNUM0, REGNUM1, REGNUM2, DIN0, WE0, DOUT0, DOUT1);
    input             CLK;
    input       [4:0] REGNUM0, REGNUM1, REGNUM2;
    input      [31:0] DIN0;
    input             WE0;
    output     [31:0] DOUT0, DOUT1;
    reg [31:0] r[0:31];
    assign DOUT0 = (REGNUM0==0) ? 0 : r[REGNUM0];
    assign DOUT1 = (REGNUM1==0) ? 0 : r[REGNUM1];
    always @(posedge CLK) if(WE0) r[REGNUM2] <= DIN0;
endmodule
module ALU(CLK, ALU_CTL, NUM1, NUM2, ZERO, ALU_RES);
   input CLK;
   input ALU_CTL;
   input [31:0] NUM1, NUM2;
   output 	ZERO;
   output [31:0] 	ALU_RES;

   assign ZERO = (ALU_RES == 0);
   assign ALU_RES = NUM1 + NUM2;
endmodule // ALU
