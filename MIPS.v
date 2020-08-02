`timescale 1ns/1ns

module MIPSprocessor (input clk, rst);
    
    wire PCWrite, MemWrite, MemRead, RegDst, MemtoReg, RegWrite, ALUSrc, ZeroFlag, IF_IdWriteWire, HazardSel, WriteRegSignalStage4, WriteRegSignalStage5, MemReadNextStage, aclr, EorEbar;
    wire [1:0] PcSrc, ForwardingWire3, ForwardingWire4;
    wire [2:0] ALUop;
    wire [4:0] RtWire, RsWire, RdWire, RDRegstage4, RDRegstage5, Rs, Rt;
    wire [5:0] Opcode;
    wire [8:0] ControlOutput;
    wire [31:0] Instruction;

    assign Rs = Instruction[25:21];
    assign Rt = Instruction[20:16];
    assign firstBitPcSrc = PcSrc[0];
    assign Opcode = Instruction[31:26];
    assign {MemRead, MemWrite, ALUSrc, RegDst, ALUop, MemToReg, RegWrite} = ControlOutput;

    Control CU(Instruction, EorEbar, ControlOutput, PcSrc);
    ForwardingUnit FW(RsWire, RtWire, RDRegstage4, RDRegstage5, WriteRegSignalStage4, WriteRegSignalStage5, ForwardingWire3, ForwardingWire4);
    HazardUnit HU(Rs, Rt, RdWire, MemReadNextStage, firstBitPcSrc, Opcode, PCWrite, IF_IdWriteWire, HazardSel, aclr);
    MIPSDataPath DP(rst, clk, PCWrite, IF_IdWriteWire, HazardSel, aclr, {MemRead, MemWrite, ALUSrc, RegDst, ALUop, MemToReg, RegWrite}, PcSrc, ForwardingWire3, ForwardingWire4, ZeroFlag, Instruction, RtWire, RsWire, RdWire, RDRegstage4, RDRegstage5, WriteRegSignalStage4, WriteRegSignalStage5, MemReadNextStage, EorEbar);
endmodule