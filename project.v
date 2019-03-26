// Part 2 skeleton

module project
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
        KEY,
        SW,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		LEDR
	);

	input			CLOCK_50;				//	50 MHz
	input   [9:0]   SW;
	input   [3:0]   KEY;

	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	output [9:0] LEDR;
	
	wire resetn;
	assign resetn = KEY[0];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(KEY[0]),
			.clock(CLOCK_50),
			.colour(colour_out),
			.x(xout),
			.y(yout),
			.plot(plot),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.

   wire [7:0] counter;
	// States of FSM that we are in.
	wire shift, load, plot, wait_out;
	wire [11:0] reg0, reg1, reg2, reg3, reg4;
	wire [2:0] colour_out;
	wire [3:0] xout, yout;
	assign LEDR[9:0] = reg0[9:0];

	datapath d0(.counter(counter), .reg0(reg0), .reg1(reg1), .reg2(reg2), .reg3(reg3), .reg4(reg4), .xout(xout), .yout(yout), .colour_out(colour_out));

   control c0(.clk(CLOCK_50), .resetn(KEY[1]), .go(1'b1), .shift(shift), .load(load), .plot(plot), .wait_out(wait_out), .counter(counter));

	fiveReg ourFiveReg(.clk(CLOCK_50), .in(SW[4:0]), .reg0(reg0), .reg1(reg1), .reg2(reg2), .reg3(reg3), .reg4(reg4), .shift(shift), 
	.load(load));
    
endmodule

module fiveReg(clk, in, reg0, reg1, reg2, reg3, reg4, shift, load);
	input clk;
	input [4:0] in;
	//shiftClk is the clk for shifting the bits right, setClk is for setting the new bits based on IN.
	input shift, load;
	output reg [11:0] reg0, reg1, reg2, reg3, reg4;
	
	initial reg0 = 12'b000000000000;
	initial reg1 = 12'b000000000000;
	initial reg2 = 12'b000000000000;
	initial reg3 = 12'b000000000000;
	initial reg4 = 12'b000000000000;

	always @(posedge clk)
	begin
		if (shift)
			begin
				reg0 <= reg0 << 1;
				reg1 <= reg1 << 1;
				reg2 <= reg2 << 1;
				reg3 <= reg3 << 1;
				reg4 <= reg4 << 1;
			end
		if (load)
			begin
				if (in[0] == 1'b1)
					reg0[0] <= 1'b1;
				if (in[1] == 1'b1)
					reg1[0] <= 1'b1;
				if (in[2] == 1'b1)
					reg2[0] <= 1'b1;
				if (in[3] == 1'b1)
					reg3[0] <= 1'b1;
				if (in[4] == 1'b1)
					reg4[0] <= 1'b1;
			end
	end
endmodule

module datapath(counter, reg0, reg1, reg2, reg3, reg4, xout, yout, colour_out);
	input [7:0] counter; 
	input [11:0] reg0, reg1, reg2, reg3, reg4;
	output reg [7:0] xout;
	output reg [6:0] yout;
	wire [7:0] temp;
	
	output reg [2:0] colour_out;
	
	always @ (*) 
	begin //counter
		xout = 0;
		colour_out = 3'b111;
		yout = 0;
		if(counter < 2400) begin // 0 - 2399
			xout = 30 + (counter % 20);
			if (reg0[yout] == 1'b1)
				colour_out = 3'b010;
		end
		else if ((counter > 2399)&&(counter < 4800)) begin //2400 - 4799
			xout = 50 + (counter % 20);
			if (reg1[yout] == 1'b1)
				colour_out = 3'b100;
		end
		else if ((counter > 4799)&&(counter < 7200)) begin//4800 - 7199
			xout = 70 + (counter % 20);
			if (reg2[yout] == 1'b1)
				colour_out = 3'b110;
		end
		else if ((counter > 7199)&&(counter < 9600)) begin //7200 - 9599
			xout = 90 + (counter % 20);
			if (reg3[yout] == 1'b1)
				colour_out = 3'b001;
		end
		else if (counter > 9599) begin //9600 - 12000
			xout = 110 + (counter % 20);
			if (reg4[yout] == 1'b1)
				colour_out = 3'b101;
		end
		
	end
endmodule

module control(
    input clk,
    input resetn,
    input go,
	output reg  shift, load, plot, wait_out,
	output reg [15:0] counter
    );
	reg [24:0] wait_counter;
	reg [24:0] wait_reset;

    reg [3:0] current_state, next_state; 
	 initial current_state = S_RESET;
	 initial counter = 1'b0;
	 initial wait_counter = 25'd49999999;
    
    localparam  S_RESET      = 4'd0,
                S_SHIFT   	 = 4'd1,
                S_LOAD       = 4'd2,
                S_PRINT   	 = 4'd3,
                S_WAIT       = 4'd4;
    
    // Next state logic aka our state table
    always@(*)
    begin: state_table 
            case (current_state)
                S_RESET: next_state = go ? S_SHIFT : S_RESET; // Loop in current state until value is input
                S_SHIFT: next_state = go ? S_LOAD : S_SHIFT; // Loop in current state until go signal goes low
                S_LOAD: next_state = go ? S_PRINT : S_LOAD; // Loop in current state until value is input
                S_PRINT: next_state = go ? S_WAIT : S_PRINT; // Loop in current state until go signal goes low
                S_WAIT: next_state = go ? S_SHIFT : S_WAIT; // Loop in current state until value is input
            default:     next_state = S_RESET;
        endcase
    end // state_table
   
//number: 25'd49999999
    // Output logic aka all of our datapath control signals
    always @(*)
    begin: enable_signals
        // By default make all our signals 0
        shift = 1'b0;
        load = 1'b0;
        plot = 1'b0;
			wait_out = 1'b0;

        case (current_state)
            S_LOAD: begin
                load = 1'b1;
                end
            S_SHIFT: begin
                shift = 1'b1;
                end
            S_PRINT: begin
                plot = 1'b1;
                end
				S_WAIT: begin
					wait_out = 1'b1;
					end
        endcase
    end // enable_signals
   
    // current_state registers
    always@(posedge clk)
    begin: state_FFs
        if(!resetn)
			begin
            current_state <= S_RESET;
				counter <= 1'b0;
				wait_counter <= 25'd49999999;
			end
        else
		begin
         if(current_state == S_PRINT)
			begin
				if(counter == 16'd11999)
					begin
						current_state <= next_state;
						counter <= 1'b0;
					end
				else
					counter <= counter + 1;
			end
			//END PRINT COUNTER

			else if (current_state == S_WAIT)
			begin
				if(wait_counter == 1'b0)
					begin
						current_state <= next_state;
						wait_counter <= 25'd49999999;
					end
				else
					wait_counter <= wait_counter - 1;
			end

			//END WAIT COUNTER
			else
				current_state <= next_state;
		end
    end // state_FFS
endmodule

//module project2(CLOCK_50, SW, KEY, LEDR, HEX0, xBus, yBus, plot, colour_out, shift, load);
//	input CLOCK_50;
//	input [9:0] SW;
//	input [3:0] KEY;
//	input shift;
//	input load;
//	output [9:0] LEDR;
//	output [7:0] xBus;
//	output [6:0] yBus;
//	output [2:0] colour_out;
//	output [6:0] HEX0;
//	output plot;
//	
//	wire [1:0] xoffset, yoffset;
//	wire loadx, loady, p;
//	
//
//
//
//endmodule
//	
//		