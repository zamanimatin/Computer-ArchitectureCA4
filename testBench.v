module ATB();

    reg clk = 1'b0;
    reg rst = 1'b1;

    always #10 clk = ~clk;



    MIPSprocessor  MIPS(clk, rst);

    initial begin
        #32 rst = 1'b0;
        #15000 $stop;
    end


endmodule