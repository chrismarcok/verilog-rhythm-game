module RateDivider(in_clock, reset_val, out_clock);
	input in_clock;
	input [24:0] reset_val;
	reg [24:0] counter;
	output reg out_clock;
	
	initial begin
		counter = 0;
		out_clock = 0;
	end
	
	always @(posedge in_clock)
	begin
		if(counter == 0)
			begin
				counter <= reset_val;
				out_clock <= ~out_clock;
			end
		else begin
			counter <= counter - 25'd1;
		end
	end
endmodule	

module caseSelect(s, out);
    input [1:0] s;
    output reg [24:0] out; 
	
	always @(*)
	begin
		case(s[1:0])
			2'b00: out = 25'd49999999;
			2'b01: out = 25'd24999999;
			2'b10: out = 25'd12499999;
			2'b11: out = 25'd6249999;
			default: out = 0;
		endcase
	end
endmodule