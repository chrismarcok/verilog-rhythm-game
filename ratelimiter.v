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
				counter <= 25'd49999999;
				out_clock <= ~out_clock;
			end
		else begin
			counter <= counter - 25'd1;
		end
	end
endmodule	