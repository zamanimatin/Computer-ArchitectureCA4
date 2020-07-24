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

module SignExtender(input [15:0]in, output reg [31:0] out);
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


module MUX32BitInput3(input [31:0] ZEROsel, ONEsel, TWOsel, input[1:0] selector, output reg [31:0] Out);
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

module MUX5Bit(input [4:0]ZEROsel, ONEsel, input selector, output reg [4:0] out);
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

module MUX9Bit(input [8:0]ZEROsel, ONEsel, input selector, output reg [8:0]out);
    always @(ZEROsel, ONEsel, selector)begin
        if(selector == 1'b0)begin
            out = ZEROsel;
        end
        else begin
            out = ONEsel;
        end
    end
endmodule 

module ZEROExtender(input [25:0]in, output reg [27:0]out);
    always @(in) begin
        out = 28'b0000000000000000000000000000; // output rule for combinationals
        out = {in, 2'b00};
    end
endmodule

// Defining Middle Registers

module IF_ID(input [31:0]AdderIn, InstructionIn, PCIn, input clk, rst, ldin, aclr, output reg [31:0]AdderOut, InstructionOut, PCout);
    always @(posedge clk, posedge rst) begin
        if (rst) begin
            AdderOut <= 32'b00000000000000000000000000000000;
            InstructionOut <= 32'b00000000000000000000000000000000;
            PCout <= 32'b00000000000000000000000000000000;
        end
        else if (ldin) begin
            AdderOut <= AdderIn;
            InstructionOut <= InstructionIn;
            PCout <= PCIn;
        end
    end
    always @(posedge aclr) begin
        if (aclr)begin
            AdderOut <= 32'b00000000000000000000000000000000;
            InstructionOut <= 32'b00000000000000000000000000000000;
            PCout <= 32'b00000000000000000000000000000000;
        end

    end
endmodule 

module ID_EX(input [31:0]ReadData1In, ReadData2In,input  EorEbarIn, input [31:0]SEXTIn, input[4:0]RtIn, RdIn, RsIn, input[1:0] ControlWBsignalIn, input [1:0]ControlMemSignalIn, input [4:0]ControlEXSignalIn, input rst, clk, output reg [31:0] ReadData1Out, ReadData2Out, output reg EorEbarOut, output reg [31:0]SEXTOut, output reg [4:0]RtOut, RdOut, RsOut, output reg[1:0] ControlWBsignalOut, output reg [1:0]ControlMemSignalOut, output reg [4:0]ControlEXSignalOut);
    always @(posedge clk, posedge rst) begin
        if(rst)begin
            ReadData1Out <= 32'b00000000000000000000000000000000;
            ReadData2Out <= 32'b00000000000000000000000000000000;
            EorEbarOut <= 1'b0;
            SEXTOut <= 32'b00000000000000000000000000000000;
            RtOut <= 5'b00000;
            RdOut <= 5'b00000;
            RsOut <= 5'b00000;
            ControlWBsignalOut <= 2'b00;
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


module MIPSDataPath(input rst, clk, PCWrite, IF_IDWrite, HazardSel, input[8:0]ControlOutput, input [1:0]ForwardingSel, output zeroflag, output [31:0]instruction, output [4:0]RTReg, RSReg, RDRegStage4, RDRegStage5, output WriteRegSignalStage4, WriteRegSignalStage5);

    wire [31:0]MUX1Wire, Container4, PCOutputWire, InstructionWireIn, PCAdderOut, JumpAdderOut, PCAdderOutToJumpAdder, InstructionWireOut,
    SHL2Wire, SEXTOutWire, MUX7Wire, ReadData1WireIn, ReadData2WireIn, ReadData1WireOut, ReadData2WireOut, SEXTOutWire2,
    ALUOutWire, ALUOut, WriteDataWire, ReadDataWire, ReadDataOutWire, ALUOutWire2, PCif_idRegOut;
    wire Cout1, Cout2, IF_IdWriteWire, EorEbarIn, EorEbarOut, RegWriteSignal, ALUSrc, regdst, ZeroFlag, MemRead, MemWrite, MemtoReg, aclr;
    wire [4:0]MUX6WireOut2, RtWire, RdWire, RsWire, EXSignalWire, MUX6Wire;
    wire [8:0]Container0, MUX2Wire;
    
    wire [1:0] WBSignalWire, WBSignalWire2, WBSignalWire3, MemSignalWire, MemSignalWire2, ForwardingWire, PCSrc;
    wire [2:0]aluop;
    wire [5:0] MUX6WireOut;
    wire [27:0]AddressZeroExtended;
    

    assign Container0 = 8'b00000000;
    assign Container4 = 32'b00000000000000000000000000000100; // 32bit 4 value
    // Stage1
    // Stage1
    MUX32BitInput3 MUX1(PCAdderOut, JumpAdderOut, AddressZeroExtended, PCSrc, MUX1Wire);
    Register32BitWithLoad PC(MUX1Wire, rst, clk, PCWrite, PCOutputWire);
    InstructionMemory InstMem(rst, PCOutputWire, InstructionWireIn);
    Adder32bit Adder1(PCOutputWire, Container4, PCAdderOut, Cout1);

    IF_ID IFIDreg(PCAdderOut, InstructionWireIn, PCOutputWire, clk, rst, IF_IdWriteWire, aclr, PCAdderOutToJumpAdder, InstructionWireOut, PCif_idRegOut);

    // Stage2
    // Stage2
    ZEROExtender Zextend(InstructionWireOut[25:0], AddressZeroExtended);
    Adder32bit Adder2(PCAdderOutToJumpAdder, SHL2Wire, JumpAdderOut, Cout2);
    ShiftLeft2bit SHL2(SEXTOutWire, SHL2Wire);
    RegFile MainRegFile(InstructionWireOut[25:21], InstructionWireOut[20:16], MUX6WireOut2, MUX7Wire, clk, rst, RegWriteSignal, ReadData1WireIn, ReadData2WireIn);
    Comparator32Bit CMP(ReadData1WireIn, ReadData2WireIn, EorEbarIn);
    SignExtender SEXT(InstructionWireOut[15:0], SEXTOutWire);
    MUX9Bit MUX2(Container0, ControlOutput, HazardSel, MUX2Wire);
    
    ID_EX IDEXreg(ReadData1WireIn, ReadData2WireIn, EorEbarIn, SEXTOutWire, InstructionWireOut[20:16], InstructionWireOut[15:11], InstructionWireOut[25:21], MUX2Wire[1:0], MUX2Wire[8:7], MUX2Wire[6:2], rst, clk, ReadData1WireOut, ReadData2WireOut, EorEbarOut, SEXTOutWire2, RtWire, RdWire, RsWire, WBSignalWire, MemSignalWire, EXSignalWire);

    // Stage3
    // Stage3
    MUX32BitInput3 MUX3(ReadData1WireOut, ALUOutWire, MUX7Wire, ForwardingWire, MUX3wire);
    MUX32BitInput3 MUX4(ReadData2WireOut, ALUOutWire, MUX7Wire, ForwardingWire, MUX4wire);
    MUX32Bit MUX5(MUX4wire, SEXTOutWire2, ALUSrc, MUX5Wire);
    MUX5Bit MUX6(RdWire, RtWire, regdst, MUX6Wire);
    ALU32Bit MainALU(MUX3wire, MUX5Wire, aluop, ALUOut, ZeroFlag); 
    assign ALUSrc = EXSignalWire[4];
    assign regdst = EXSignalWire[3];
    assign aluop = EXSignalWire[2:0];
    
    EX_MEM EXMEMreg(ALUOut, ZeroFlag, MUX4wire, MUX6Wire, WBSignalWire, MemSignalWire, clk, rst, ALUOutWire, zeroflag, WriteDataWire, MUX6WireOut, WBSignalWire2, MemSignalWire2);

    // Stage4
    // Stage4   
    DataMemory MainDataMem(ALUOutWire, WriteDataWire, MemRead, MemWrite, clk, rst, ReadDataWire);
    assign MemRead = MemSignalWire2[1];
    assign memWrite = MemSignalWire2[0];
    
    MEM_WB MEMWBreg(ReadDataWire, ALUOutWire, MUX6WireOut, WBSignalWire2, clk, rst, WBSignalWire3, ReadDataOutWire, ALUOutWire2, MUX6WireOut2);


    // Stage5
    // Stage5
    MUX32Bit MUX7(ALUOutWire2, ReadDataOutWire, MemtoReg, MUX7Wire);
    assign MemtoReg = WBSignalWire3[1];
    assign RegWriteSignal = WBSignalWire3[0];

    // output assignment
    // output assignment
    assign instruction = InstructionWireOut;
    assign RTReg = RtWire;
    assign RSReg = RsWire;
    assign RDRegStage4 = MUX6WireOut;
    assign RDRegStage5 = MUX6WireOut2;
    assign WriteRegSignalStage4 = WBSignalWire2[0];
    assign WriteRegSignalStage5 = WBSignalWire3[0];


endmodule

