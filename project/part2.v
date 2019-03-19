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
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
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
	
	wire [6:0] xin;
	wire [6:0] yin;
	wire draw;
	
	wire [6:0] xout;
	wire [6:0] yout;
	wire [2:0] colour_out;
    
    // Instansiate datapath
	// datapath d0(...);
		

    // Instansiate FSM control
    // control c0(...);
    
endmodule

module datapath(colour, reset_n, xoffset, yoffset, position, loadX, loadY, xout, yout, colour_out);
	input [2:0] colour;
	input reset_n;
	input [1:0] xoffset;
	input [1:0] yoffset; 
	input [6:0] position;
	input loadX, loadY;
	
	output [7:0] xout;
	output [6:0] yout;
	
	output [2:0] colour_out;
	
	reg [7:0] regx;
	reg [6:0] regy;
	
		
	assign xout = regx + xoffset;
	assign yout = regy + yoffset;
	assign colour_out = colour;
	
	always @ (posedge loadX, posedge loadY) 
	begin 
		if(loadX)
			regx <= {1'b0, position};
		if(loadY)
			regy <= position;
	end
	
	always @ (negedge reset_n) 
	begin
		regx <= 1'b0;
		regy <= 1'b0;
	end
endmodule

module control(
    input clk,
    input resetn,
    input go,

    output reg  loadX, loadY, plot,
    output [1:0]  xoffset,
	output [1:0]  yoffset
    );
	
	reg [3:0] counter;
	assign xoffset = counter[1:0];
	assign yoffset = counter[3:2];

    reg [3:0] current_state, next_state; 
    
    localparam  S_LOAD_X        = 4'd0,
                S_LOAD_X_WAIT   = 4'd1,
                S_LOAD_Y        = 4'd2,
                S_LOAD_Y_WAIT   = 4'd3,
                S_LOAD_DRAW        = 4'd4,
                S_LOAD_DRAW_WAIT   = 4'd5,
                S_DRAW        = 4'd6;
    
    // Next state logic aka our state table
    always@(*)
    begin: state_table 
            case (current_state)
                S_LOAD_X: next_state = go ? S_LOAD_X_WAIT : S_LOAD_X; // Loop in current state until value is input
                S_LOAD_X_WAIT: next_state = go ? S_LOAD_X_WAIT : S_LOAD_Y; // Loop in current state until go signal goes low
                S_LOAD_Y: next_state = go ? S_LOAD_Y_WAIT : S_LOAD_Y; // Loop in current state until value is input
                S_LOAD_Y_WAIT: next_state = go ? S_LOAD_Y_WAIT : S_DRAW; // Loop in current state until go signal goes low
                S_LOAD_DRAW: next_state = go ? S_LOAD_DRAW_WAIT : S_LOAD_DRAW; // Loop in current state until value is input
                S_LOAD_DRAW_WAIT: next_state = go ? S_LOAD_DRAW_WAIT : S_DRAW; // Loop in current state until go signal goes low
                S_DRAW: next_state = S_LOAD_X;
            default:     next_state = S_LOAD_X;
        endcase
    end // state_table
   

    // Output logic aka all of our datapath control signals
    always @(*)
    begin: enable_signals
        // By default make all our signals 0
        loadX = 1'b0;
        loadY = 1'b0;
        plot = 1'b0;

        case (current_state)
            S_LOAD_X: begin
                loadX = 1'b1;
                end
            S_LOAD_Y: begin
                loadY = 1'b1;
                end
            S_DRAW: begin
                plot = 1'b1;
                end
        // default:    // don't need default since we already made sure all of our outputs were assigned a value at the start of the always block
        endcase
    end // enable_signals
   
    // current_state registers
    always@(posedge clk)
    begin: state_FFs
        if(!resetn)
			begin
            current_state <= S_LOAD_X;
			counter <= 1'b0;
			end
        else
		begin
            if(current_state == S_DRAW)
			begin
				if(counter == 4'b1111)
					begin
					current_state <= next_state;
					counter <= 1'b0;
					end
				else
					counter <= counter + 1;
			end
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
	
		