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


module MUX32BitInput3(input [31:0] ZEROsel, ONEsel, TWOsel, input selector, output reg [31:0] Out);
    always @(ZEROsel, ONEsel, TWOsel, selector) begin
        Out = 32'b00000000000000000000000000000000;
        if(selector == 2'b00)begin
            Out = ZEROsel;
        end
        else if (selector == 2'b01)begin
            Out = ONEsel;
        end
        else if (selector == 2'b10)begin
            Out = TWOsel;
        end
    end
endmodule

module  MUX5Bit(input [4:0]ZEROsel, ONEsel, input selector, output reg [4:0] out);
    always @(ZEROsel, ONEsel, selector) begin
        out = 5'b0;
        if(selector == 1'b0)
            out = ZEROsel;
        else if (selector == 1'b1)
            out = ONEsel;
    end
endmodule

module ALU32Bit (input [31:0]A, B, input [2:0]ALUop, output reg[31:0] ALUout, output reg ZeroFlag);
    always @(A, B, ALUop) begin
        ALUout = 32'b0;
        ZeroFlag = 1'b0;
        if (ALUop == 3'b000) // case AND
            ALUout = A & B;
        else if (ALUop == 3'b001) // case OR
            ALUout = A | B;
        else if (ALUop == 3'b010) // case ADD
            ALUout = A + B;
        else if (ALUop == 3'b110) // case SUB
            ALUout = A - B;
        else if (ALUop == 3'b111)begin // case SLT
            if ($signed(A) < $signed(B))  // supposed that A and B are signed check whether comparison happens right?
                ALUout = 32'b00000000000000000000000000000001;
            else
                ALUout = 32'b00000000000000000000000000000000;
        end
        if(ALUout == 32'b0)
            ZeroFlag = 1'b1;
    end
endmodule

module DataMemory (input [31:0]address, writedata, input MemRead, MemWrite, clk, rst, output reg [31:0]ReadData);
    reg [31:0] DMemory [0:512];
    always @(negedge rst) begin
        $readmemb("DataMemory.mem", DMemory);
    end
    always @(address, MemRead) begin
        ReadData = 32'b00000000000000000000000000000000;
        if (MemRead)
            ReadData = DMemory[address[31:2]];
    end
    integer i = 0;
    always@( posedge clk, posedge rst) begin
        if (rst)
            for(i = 0; i < 512; i = i + 1)begin
                DMemory[i] = 32'b00000000000000000000000000000000;
            end
        else if (MemWrite)
            DMemory[address[31:2]] = writedata;
    end
endmodule


// Defining Middle Registers

module IF_ID(input [31:0]AdderIn, InstructionIn, input clk, rst, ldin, output reg [31:0]AdderOut, InstructionOut);
    always @(posedge clk, posedge rst) begin
        if (rst) begin
            AdderOut <= 32'b00000000000000000000000000000000;
            InstructionOut <= 32'b00000000000000000000000000000000;
        end
        else if (ldin) begin
            AdderOut <= AdderIn;
            InstructionOut <= InstructionIn;
        end
    end
endmodule 

module ID_EX(input [31:0]ReadData1In, ReadData2In,input  EorEbarIn, input [31:0]SEXTIn, input[4:0]RtIn, RdIn, RsIn, input ControlWBsignalIn, input [1:0]ControlMemSignalIn, input [4:0]ControlEXSignalIn, input rst, clk, output reg [31:0] ReadData1Out, ReadData2Out, output reg EorEbarOut, output reg [31:0]SEXTOut, output reg [4:0]RtOut, RdOut, RsOut, output reg ControlWBsignalOut, output reg [1:0]ControlMemSignalOut, output reg [4:0]ControlEXSignalOut);
    always @(posedge clk, posedge rst) begin
        if(rst)begin
            ReadData1Out <= 32'b00000000000000000000000000000000;
            ReadData2Out <= 32'b00000000000000000000000000000000;
            EorEbarOut <= 1'b0;
            SEXTOut <= 32'b00000000000000000000000000000000;
            RtOut <= 5'b00000;
            RdOut <= 5'b00000;
            RsOut <= 5'b00000;
            ControlWBsignalOut <= 1'b0;
            ControlMemSignalOut <= 2'b00;
            ControlEXSignalOut <= 5'b00000;
        end
        else begin
            ReadData1Out <= ReadData1In;
            ReadData2Out <= ReadData2In;
            EorEbarOut <= EorEbarIn;
            SEXTOut <= SEXTIn;
            RtOut <= RtIn;
            RdOut <= RdIn;
            RsOut <= RsIn;
            ControlWBsignalOut <= ControlWBsignalIn;
            ControlMemSignalOut <= ControlMemSignalIn;
            ControlEXSignalOut <= ControlEXSignalOut;
        end
    end
endmodule

module EX_MEM(input [31:0]ALUresultIn, input ZeroFlagIn, input [31:0]WriteDataIn, input [4:0]RegDstIn, input ControlWBsignalIn, input [1:0]ControlMemSignalIn, clk, rst, output reg [31:0] ALUresultOut, output reg ZeroFlagOut, output reg [31:0]WriteDataOut, output reg [4:0]RegDstOut, output reg ControlWBsignalOut, output reg [1:0]ControlMemSignalOut);
    always @(posedge clk, posedge rst) begin
        if (rst) begin
            ALUresultOut <= 32'b00000000000000000000000000000000;
            ZeroFlagOut <= 1'b0;
            WriteDataOut <= 32'b00000000000000000000000000000000;
            RegDstOut <= 5'b00000;
            ControlWBsignalOut <= 1'b0;
            ControlMemSignalOut <= 2'b00;
        end
        else begin
            ALUresultOut <= ALUresultIn;
            ZeroFlagOut <= ZeroFlagIn;
            WriteDataOut <= WriteDataIn;
            RegDstOut <= RegDstIn;
            ControlWBsignalOut <= ControlWBsignalIn;
            ControlMemSignalOut <= ControlMemSignalIn;
        end
    end
endmodule

module MEM_WB(input [31:0]ReadDataIn, ALUresultIn, input [4:0]RegDstIn, input ControlWBsignalIn, clk, rst, output reg ControlWBsignalOut, output reg [31:0]ReadDataOut, ALUresultOut, output reg [4:0]RegDstOut);
    always @(posedge clk, posedge rst) begin
        if(rst) begin
            ControlWBsignalOut <= 1'b0;
            ReadDataOut <= 32'b00000000000000000000000000000000;
            ALUresultOut <= 32'b00000000000000000000000000000000;
            RegDstOut <= 5'b00000;
        end
        else begin
            ControlWBsignalOut <= ControlWBsignalIn;
            ReadDataOut <= ReadDataIn;
            ALUresultOut <= ALUresultIn;
            RegDstOut <= RegDstIn;
        end
    end
endmodule 


