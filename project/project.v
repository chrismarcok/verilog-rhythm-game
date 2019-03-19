// Part 2 skeleton

module part2
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
		VGA_B   						//	VGA Blue[9:0]
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
		defparam VGA.RESOLUTION = "11x12";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "background.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.

    reg [7:0] counter;
	// States of FSM that we are in.
	wire shift, load, plot, wait_out;
	reg [11:0] reg0, reg1, reg2, reg3, reg4;
	reg [2:0] colour_out;
	reg [3:0] xout, yout;

    // Instansiate datapath
	datapath d0(.counter(counter), .reg0(reg0), .reg1(reg1), .reg2(reg2), .reg3(reg3), .reg4(reg4), .xout(xout), .yout(yout), .colour_out(colour_out));

    // Instansiate FSM control
    control c0(.clk(CLOCK_50), .resetn(KEY[0]), .go(1'b1), .shift(shift), .load(load), .plot(plot), .wait_out(wait_out), .counter(counter));

	//Five registers for the 5 columns
	fiveReg ourFiveReg(.in(SW[4:0]), .reg0(reg0), .reg1(reg1), .reg2(reg2), .reg3(reg3), .reg4(reg4), .shift(shift), .load(load), .resetn(KEY[0]));
    
endmodule

module fiveReg(in, reg0, reg1, reg2, reg3, reg4, shift, load, resetn);
	input [4:0] in;
	//shiftClk is the clk for shifting the bits right, setClk is for setting the new bits based on IN.
	input shift, resetn, load;
	output reg [11:0] reg0, reg1, reg2, reg3, reg4;

	always @(posedge shift, posedge load, negedge resetn)
	begin
		if (!resetn)
			begin
				reg0 <= 1'b0;
				reg1 <= 1'b0;
				reg2 <= 1'b0;
				reg3 <= 1'b0;
				reg4 <= 1'b0;
			end
		if (shift)
			begin
				reg0 <= reg0 >> 1;
				reg1 <= reg1 >> 1;
				reg2 <= reg2 >> 1;
				reg3 <= reg3 >> 1;
				reg4 <= reg4 >> 1;
			end
		if (load)
			begin
				if (in[0] == 1'b1)
					reg0[0] <= 1'b1;
				if (in[1] == 1'b1)
					reg1[1] <= 1'b1;
				if (in[2] == 1'b1)
					reg2[2] <= 1'b1;
				if (in[3] == 1'b1)
					reg3[3] <= 1'b1;
				if (in[4] == 1'b1)
					reg4[4] <= 1'b1;
			end
	end
endmodule

module datapath(counter, reg0, reg1, reg2, reg3, reg4, xout, yout, colour_out);
	input [7:0] counter; 
	input [11:0] reg0, reg1, reg2, reg3, reg4;
	output reg [3:0] xout;
	output reg [3:0] yout;
	
	output [2:0] colour_out;
	
	always @ (*) 
	begin: counter
		xout = 0;
		yout = counter % 12;
		colour_out = 111;
		case (counter)
			[0:11]: begin
				xout = 3;
				if (reg0[yout] == 1'b1)
					colour_out = 010;
			end
			[12:23]: begin
				xout = 4;
				if (reg1[yout] == 1'b1)
					colour_out = 100;
			end
			[24:35]: begin
				xout = 5;
				if (reg2[yout] == 1'b1)
					colour_out = 110;
			end
			[36:47]: begin
				xout = 6;
				if (reg3[yout] == 1'b1)
					colour_out = 001;
			end
			[48:59]: begin
				xout = 7;
				if (reg4[yout] == 1'b1)
					colour_out = 101;
			end
		endcase
	end
endmodule

module control(
    input clk,
    input resetn,
    input go,

    output reg  shift, load, plot, wait_out;
	output reg [7:0] counter;
    );
	reg [24:0] wait_counter;
	reg [24:0] wait_reset = 25'd49999999;

    reg [3:0] current_state, next_state; 
    
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
			wait_counter <= 1'b0;
			end
        else
		begin
            if(current_state == S_PRINT)
			begin
				if(counter == 8'd59)
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
						wait_counter <= wait_reset;
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

module top_level(CLOCK_50, SW, KEY, xBus, yBus, plot, colour_out);
	input CLOCK_50;
	input [9:0] SW;
	input [3:0] KEY;
	output [7:0] xBus;
	output [6:0] yBus;
	output [2:0] colour_out;
	output plot;
	
	wire [1:0] xoffset, yoffset;
	wire loadx, loady, p;
	
	datapath d0(
		.colour(SW[9:7]),
		.reset_n(KEY[0]),
		.xoffset(xoffset),
		.yoffset(yoffset),
		.position(SW[6:0]),
		.loadX(loadx),
		.loadY(loady),
		.xout(xBus),
		.yout(yBus),
		.colour_out(colour_out)
		);
	
	control c0(
		.clk(CLOCK_50),
		.resetn(KEY[0]),
		.go(KEY[1]),
		.loadX(loadx),
		.loadY(loady),
		.plot(p),
		.xoffset(xoffset),
		.yoffset(yoffset)
		);
endmodule
	
		