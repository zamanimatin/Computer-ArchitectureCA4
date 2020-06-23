`timescale 1ns/1ns

module Register32BitWithLoad(input [31:0]in, input rst, clk, ldin, output reg[31:0]out);
    always @(posedge clk, posedge rst) begin
        if (rst)
            out <= 32'b0;
        else if (ldin)
            out <= in;
    end
endmodule

module InstructionMemory(input rst, input [31:0]addressin, output reg [31:0]instruction);
    reg [31:0] instructionMemory [0:512];
    always @(negedge rst) begin
        $readmemb("instructionb.mem", instructionMemory);
    end
    integer i = 0;
    always @(posedge rst, addressin)begin
        instruction = 32'b00000000000000000000000000000000;
        if(rst)
            
            for( i = 0; i < 512; i = i + 1)begin
                instructionMemory[i] = 32'b00000000000000000000000000000000;
            end
        else
            instruction = instructionMemory[addressin[31:2]];
    end
endmodule

module Adder32bit(input [31:0]A, B, output reg [31:0]out, output reg cout);
    always @(A, B) begin
        out = 32'b0;
        cout = 1'b0;
        {cout, out} = A + B;
    end
endmodule

module MUX32Bit (input [31:0]ZEROsel, ONEsel, input selector, output reg [31:0] out);
    always @(ZEROsel, ONEsel, selector) begin
        out = 32'b0;
        if(selector == 1'b0)
            out = ZEROsel;
        else if (selector == 1'b1)
            out = ONEsel;
    end
endmodule

module Comparator32Bit(input [31:0]A, B, output reg E);
    always @(A, B) begin
        E = 1'b0;
        if (A == B)
            E = 1'b1; // Note that in case of equality the E becomes 1
    end
endmodule

module ShiftLeft2bit(input [31:0]in, output reg [31:0]out);
    always @(in) begin
        out = 32'b0;
        out = {in[29:0], 2'b00};
    end
endmodule

module  SignExtender(input [15:0]in, output reg [31:0] out);
    always @(in) begin
        out = 32'b0;
        out = 32'(signed'(in));
    end
endmodule

module RegFile (input [4:0]ReadReg1, ReadReg2, input [4:0]WriteReg, input [31:0]Writedata, input clk, rst, regWriteSignal, output reg [31:0]ReadData1, ReadData2);
    reg [31:0] REGFILE [0:31];

    always @(ReadReg1, ReadReg2) begin
        {ReadData1, ReadData2} = 64'b0;
        ReadData1 = REGFILE[ReadReg1];
        ReadData2 = REGFILE[ReadReg2];
    end
    integer i;
    always @(posedge clk, posedge rst)begin
        if (rst) begin
            for(i = 0; i < 32; i = i + 1) begin
                REGFILE[i] = 32'b0;
            end
        end
        if (regWriteSignal)
            REGFILE[WriteReg] = Writedata;
    end
endmodule

