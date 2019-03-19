module lab5pt2(SW, CLOCK_50, HEX0);
	input [3:0] SW;
	input CLOCK_50;
	output [6:0] HEX0;
	
	runMe r0(
		.clock(CLOCK_50),
		.select(SW[1:0]),
		.reset(SW[2]),
		.enable(SW[3]),
		.hex(HEX0)
		);
endmodule 

module runMe(clock, select, reset, enable, hex);
	input clock;
	input [1:0] select;
	input reset, enable;
	output [6:0] hex;
	
	wire [24:0] caseWire;
	wire clockWire;
	wire [3:0] decoderWire;
	
	caseSelect c0(
		.s(select[1:0]),
		.out(caseWire)
		);
	
	RateDivider r0(
		.in_clock(clock),
		.reset_val(caseWire),
		.out_clock(clockWire)
		);
	
	DisplayCounter dc0(
		.clock(clockWire),
		.reset_n(reset),
		.enable(enable),
		.q(decoderWire)
		);
		
	decoder d0(
		.x(decoderWire),
		.z(hex)
		);
	
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

module DisplayCounter(clock, reset_n, enable, q);
	input clock;
	input reset_n;
	input enable;
	output reg [3:0] q;
	
	always @(posedge clock)
	begin
		if(reset_n == 1'b0)
			q <= 0;
		else if (enable == 1'b1)
			begin
				if (q == 4'b1111)
					q <= 0;
				else
					q <= q + 1'b1;
			end
	end
endmodule	

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

module decoder(
	input  [3:0]x,
    output reg [6:0]z
    );
	always @*
	case (x)
		4'b0000 :      	//Hexadecimal 0
		z = 7'b1000000;
		4'b0001 :    		//Hexadecimal 1
		z = 7'b1111001  ;
		4'b0010 :  		// Hexadecimal 2
		z = 7'b0100100 ; 
		4'b0011 : 		// Hexadecimal 3
		z = 7'b0000110 ;
		4'b0100 :		// Hexadecimal 4
		z = 7'b0011001;
		4'b0101 :		// Hexadecimal 5
		z = 7'b0010010;  
		4'b0110 :		// Hexadecimal 6
		z = 7'b0000010 ;
		4'b0111 :		// Hexadecimal 7
		z = 7'b1111000;
		4'b1000 :     		 //Hexadecimal 8
		z = 7'b0000000;
		4'b1001 :    		//Hexadecimal 9
		z = 7'b0010000;
		4'b1010 :  		// Hexadecimal A
		z = 7'b0001000; 
		4'b1011 : 		// Hexadecimal B
		z = 7'b0000011;
		4'b1100 :		// Hexadecimal C
		z = 7'b1000110;
		4'b1101 :		// Hexadecimal D
		z = 7'b0100001;
		4'b1110 :		// Hexadecimal E
		z = 7'b0000110;
		4'b1111 :		// Hexadecimal F
		z = 7'b0001110 ;
	endcase
 
endmodule