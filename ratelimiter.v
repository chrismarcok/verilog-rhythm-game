module RateDivider(in_clock, out_clock);
	input in_clock;
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
				counter <= 25'd49999999;//reset the clock
				out_clock <= ~out_clock;
			end
		else begin
			counter <= counter - 25'd1;//decrement
		end
	end
endmodule	