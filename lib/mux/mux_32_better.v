module mux_32_better (sel, src0, src1, z);
  
  input sel; // MODIFIED BY CAM
  input [31:0] src0;
  input [31:0] src1;
  output reg [31:0] z;
  

  always @(sel or src0 or src1)
      begin
        if (sel == 1'b0) z <= src0;
        else z <= src1;
      end
   
endmodule