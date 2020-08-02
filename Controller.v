`timescale 1ns/1ns

module Control (input [31:0]instruction, input EorEbar, output [8:0] ControlOutput, output reg[1:0] PcSrc);
    wire [5:0]functionType, Opcode;
    assign Opcode = instruction[31:26];
    assign functionType = instruction[5:0];

    reg MemRead, MemWrite, ALUSrc, RegDst, MemToReg, RegWrite;
    reg [2:0] ALUop;
    assign ControlOutput = {MemRead, MemWrite, ALUSrc, RegDst, ALUop, MemToReg, RegWrite};

    always @(instruction, EorEbar)begin
        {MemRead, MemWrite, ALUSrc, RegDst, MemToReg, RegWrite, ALUop, PcSrc} = 11'b0;
        
        case(Opcode)
            6'b000000 : begin //.. ordinary rtype
                RegWrite = 1'b1;
                case(functionType)
                    6'b100000 : begin //.. case add function
                        ALUop = 3'b010;
                    end
                    6'b100010 : begin //.. case sub function
                        ALUop = 3'b110;
                    end
                    6'b100100 : begin //.. case and function 
                        ALUop = 3'b000;
                    end
                    6'b100101 : begin //.. case or function
                        ALUop = 3'b001;
                    end
                    6'b101010 : begin //.. case slt function
                        ALUop = 3'b111;
                    end
                endcase
            end
            6'b001000 : begin //.. addi
                ALUSrc = 1'b1;
                RegDst = 1'b1;
                RegWrite = 1'b1;
                ALUop = 3'b010;
            end
            6'b001100 : begin //.. andi
                ALUSrc = 1'b1;
                RegDst = 1'b1;
                RegWrite = 1'b1;
                ALUop = 3'b000;
            end
            6'b100011 : begin //.. case lw
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
                ALUop = 3'b010;
                RegDst = 1'b1;
                MemRead = 1'b1;
                MemToReg = 1'b1;
            end
            6'b101011 : begin //.. case sw
                ALUSrc = 1'b1;
                ALUop = 3'b010;
                MemWrite = 1'b1;
            end
            6'b000010 : begin //.. case j
                PcSrc = 2'b10;
            end
            6'b000100 : begin //.. case beq
                PcSrc = {1'b0, EorEbar};
            end
            6'b000101 : begin //.. case bne
                PcSrc = {1'b0, ~EorEbar};
            end
        endcase
    end
endmodule

module ForwardingUnit (input [4:0]RsWire, RtWire, RDRegstage4, RDRegstage5, input RegWriteSignalstage4, RegWriteSignalstage5, output reg[1:0] ForwardingWire3, ForwardingWire4);
    always @(RsWire, RtWire, RDRegstage4, RDRegstage5, RegWriteSignalstage4, RegWriteSignalstage5)begin
        {ForwardingWire3, ForwardingWire4} = 4'b0;
        if(RsWire == RDRegstage4 && RegWriteSignalstage4 == 1'b1)
            ForwardingWire3 = 2'b01;
        else if(RsWire == RDRegstage5 && RegWriteSignalstage5 == 1'b1)
            ForwardingWire3 = 2'b10;
        if(RtWire == RDRegstage4 && RegWriteSignalstage4 == 1'b1)
            ForwardingWire4 = 2'b01;
        else if(RtWire == RDRegstage5 && RegWriteSignalstage5 == 1'b1)
            ForwardingWire4 = 2'b10;
    end
endmodule

module HazardUnit (input [4:0]Rs, Rt, RdNextStage, input MemReadNextStage, firstBitPcSrc, input[5:0]Opcode, output reg PcWrite, IFIDWrite, HazardSel, IFIDBubble);
    always @(Rs, Rt, RdNextStage, MemReadNextStage, firstBitPcSrc, Opcode) begin
        {PcWrite, IFIDWrite, HazardSel, IFIDBubble} = 4'b1110;

        case(Opcode)
            6'b000000 : begin //.. ordinary rtype
                if(MemReadNextStage == 1'b1 && (RdNextStage == Rs || RdNextStage == Rt)) // lw <=> MemRead = 1'b1
                    {PcWrite, IFIDWrite, HazardSel} = 3'b000;
            end
            6'b001000 : begin //.. addi
                if(MemReadNextStage == 1'b1 && RdNextStage == Rs) // lw <=> MemRead = 1'b1
                    {PcWrite, IFIDWrite, HazardSel} = 3'b000;
            end
            6'b001100 : begin //.. andi
                if(MemReadNextStage == 1'b1 && RdNextStage == Rs) // lw <=> MemRead = 1'b1
                    {PcWrite, IFIDWrite, HazardSel} = 3'b000;
            end
            6'b101011 : begin //.. case sw
                if(MemReadNextStage == 1'b1 && (RdNextStage == Rs || RdNextStage == Rt)) // lw <=> MemRead = 1'b1
                    {PcWrite, IFIDWrite, HazardSel} = 3'b000;
            end
            6'b000010 : begin //.. case j
                IFIDBubble = 1'b1;

            end
            6'b000100 : begin //.. case beq
                if(firstBitPcSrc == 1'b1)
                    IFIDBubble = 1'b1;
            end
            6'b000101 : begin //.. case bne
                if(firstBitPcSrc == 1'b1)
                    IFIDBubble = 1'b1;
            end
            default : begin
                {PcWrite, IFIDWrite, HazardSel, IFIDBubble} = 4'b1110;
            end
        endcase
    end
endmodule