module proj
	(
		CLOCK_50,						//	On Board 50 MHz
		KEY,
		SW,
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		LEDR,
		HEX0, HEX1, HEX2, HEX5, HEX4,
		PS2_CLK,
		PS2_DAT
	);

	input			CLOCK_50;				//	50 MHz
	input   [9:0]   SW;
	input   [3:0]   KEY;
  
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	output [9:0] LEDR;
	output [6:0] HEX0, HEX1, HEX2, HEX5, HEX4;
	inout PS2_CLK;
	inout PS2_DAT;
	
	wire resetn;
	assign resetn = KEY[0];

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


  wire [15:0] block_counter, x_counter, y_counter;
	// States of FSM that we are in.
	wire shift, load, plot, wait_out, reset_out;
	wire [11:0] reg0, reg1, reg2, reg3, reg4;
	wire [2:0] colour_out;
	wire [7:0] xout;
	wire [6:0] yout;
	
	wire [4:0] randowire;
	wire end_game;
	fibonacci_lfsr_5bit f0(.clk(shift), .rst_n(KEY[2]), .data(randowire));

	datapath d0(.block_counter(block_counter), .x_counter(x_counter), .y_counter(y_counter), 
	.reg0(reg0), .reg1(reg1), .reg2(reg2), .reg3(reg3), .reg4(reg4), 
  .xout(xout), .yout(yout), .colour_out(colour_out));

  control c0(.clk(CLOCK_50), .resetn(KEY[1]), .go(1'b1), .shift(shift), 
	.load(load), .plot(plot), .wait_out(wait_out), .reset_out(reset_out),
	.block_counter(block_counter), .x_counter(x_counter), .y_counter(y_counter));

	fiveReg ourFiveReg(.clk(CLOCK_50), .in(randowire), 
  .reg0(reg0), .reg1(reg1), .reg2(reg2), .reg3(reg3), .reg4(reg4), 
  .shift(shift), .load(load), .reset(end_game));
	
	wire [7:0] score, lives;
	wire [3:0] score_ones, score_tens, score_hunds;
	wire life_trigger, score_trigger, game_start, mult_reset;
	wire [2:0] mult;
	scoreReg r0(.clk(score_trigger), .reset(end_game), .q(score), .mult_reset(mult_reset), .mult(mult));
	lifeReg l0(.clk(life_trigger), .q(lives), .end_game(end_game), .game_start(game_start), .mult_reset(mult_reset));
	
	BCD my_BCD(.data(score), .hundreds(score_hunds), .tens(score_tens),.ones(score_ones));
	
	my_hex_decoder h0(.hex_digit(score_ones), .segments(HEX0[6:0]));
	my_hex_decoder h1(.hex_digit(score_tens), .segments(HEX1[6:0]));
	my_hex_decoder h2(.hex_digit(score_hunds), .segments(HEX2[6:0]));
	my_hex_decoder h5(.hex_digit(lives), .segments(HEX5[6:0]));
	my_hex_decoder h4(.hex_digit(mult), .segments(HEX4[6:0]));
	
	wire [11:0] keyboard_out;
	assign LEDR[9:0] = keyboard_out[9:0];
	my_keyboard_module k0(.CLOCK_50(CLOCK_50), .reset(KEY[0]), .PS2_CLK(PS2_CLK), .PS2_DAT(PS2_DAT), .out(keyboard_out));
	
	wire [4:0] bottomBlocks = {reg0[11], reg1[11], reg2[11], reg3[11], reg4[11]};
	
	noteRegister noteReg(.clk(CLOCK_50), .wait_in(wait_out), .bottomBlocks(bottomBlocks), 
	.keyboard_in(keyboard_out), .life_out(life_trigger), .game_start(game_start), .score_out(score_trigger));
	
endmodule

module noteRegister(clk, bottomBlocks, keyboard_in, wait_in, life_out, game_start, score_out);
	input clk, wait_in;
	input [4:0] bottomBlocks;
	input [11:0] keyboard_in; //ENTER == 1, A-G == 5-9, SPACE == 0
	output reg life_out, score_out, game_start;
	
	always @(posedge clk)
	begin
		life_out <= 1'b0;
		score_out <= 1'b0;
		game_start <= 1'b0;
		if (keyboard_in[0])
		begin
			life_out <= 1'b1;
			game_start <= 1'b1;
		end
		else if (keyboard_in[1] && wait_in)
		begin
			if (keyboard_in[5] && bottomBlocks[4])
			begin
				score_out <= 1'b1;
			end
			else if (keyboard_in[5] && ~bottomBlocks[4])
			begin
				life_out <= 1'b1;
			end
			else if (keyboard_in[6] && bottomBlocks[3])
			begin						
				score_out <= 1'b1;
			end
			else if (keyboard_in[6] && ~bottomBlocks[3])
			begin
				life_out <= 1'b1;
			end
			else if (keyboard_in[7] && bottomBlocks[2])
			begin
				score_out <= 1'b1;
			end
			else if (keyboard_in[7] && ~bottomBlocks[2])
			begin
				life_out <= 1'b1;
			end
			else if (keyboard_in[8] && bottomBlocks[1])
			begin
				score_out <= 1'b1;
			end
			else if (keyboard_in[8] && ~bottomBlocks[1])
			begin
				life_out <= 1'b1;
			end
			else if (keyboard_in[9] && bottomBlocks[0])
			begin
				score_out <= 1'b1;
			end
			else if (keyboard_in[9] && ~bottomBlocks[0])
			begin
				life_out <= 1'b1;
			end
			else 
			begin
				life_out <= 1'b1;
			end
		end
	end
	
endmodule

module my_keyboard_module(
    input CLOCK_50,
	 input reset,
	 
	 inout PS2_CLK,
	 inout PS2_DAT,
	 
	 output [11:0] out
	 );
	 
	 keyboard_tracker #(.PULSE_OR_HOLD(0)) tester(
	     .clock(CLOCK_50),
		  .reset(reset),
		  .PS2_CLK(PS2_CLK),
		  .PS2_DAT(PS2_DAT),
		  .w(out[4]),
		  .a(out[5]),
		  .s(out[6]),
		  .d(out[7]),
		  .f(out[8]),
		  .g(out[9]),
		  .left(out[11]),
		  .right(out[10]),
		  .up(out[2]),
		  .down(out[3]),
		  .space(out[0]),
		  .enter(out[1])
		  );
endmodule

module lifeReg(clk, q, end_game, game_start, mult_reset);
	input clk, game_start;
	output reg [7:0] q;
	output reg end_game, mult_reset;
	initial q = 16'd5;
	initial end_game = 1'b1;
	initial mult_reset = 1'b0;
	
	always @(posedge clk)
	begin
		end_game = 1'b0;
		mult_reset = 1'b0;
		if (game_start)
		begin
			//DO NOTHING
		end
		else if (q == 8'b1)
			begin
				end_game = 1'b1;
				q <= 16'd5;
				mult_reset <= 1'b1
			end
		else
			begin
				q <= q - 1'b1;
				mult_reset <= 1'b1
			end
	end
endmodule

module scoreReg(clk, reset, mult_reset, mult, q);
	input clk, reset, mult_reset;
	output reg [9:0] q;
	output reg [2:0] mult;
	initial mult = 1'b1;
	
	always @(posedge clk, posedge reset)
	begin
		if (reset)
		begin
			q <= 1'b0;
		end
		else if (mult_reset)
		begin
			mult <= 1'b1;
		end
		else
		begin
			if (mult == 3'b100) //4
			begin
				// do nothing
			end
			else //Mult is 1, 2, or 3.
			begin
				mult <= mult + 1'b1;
			end
			
			q <= q + mult;
		end
	end
endmodule

module fiveReg(clk, in, reg0, reg1, reg2, reg3, reg4, shift, load, reset);
	input [4:0] in;
	input clk, shift, load, reset;
	output reg [11:0] reg0, reg1, reg2, reg3, reg4;
	
	wire [2:0] out;
	assign out = in % 5;
	
	initial reg0 = 12'b000000000000;
	initial reg1 = 12'b000000000000;
	initial reg2 = 12'b000000000000;
	initial reg3 = 12'b000000000000;
	initial reg4 = 12'b000000000000;

	always @(posedge clk)
	begin
		if (reset)
			begin
				reg0 <= 1'b0;
				reg1 <= 1'b0;
				reg2 <= 1'b0;
				reg3 <= 1'b0;
				reg4 <= 1'b0;
			end
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
				if (out == 1'b0)
					reg0[0] <= 1'b1;
				if (out == 1'b1)
					reg1[0] <= 1'b1;
				if (out == 2'b10)
					reg2[0] <= 1'b1;
				if (out == 2'b11)
					reg3[0] <= 1'b1;
				if (out == 3'b100)
					reg4[0] <= 1'b1;
			end
	end
endmodule

module datapath( block_counter, x_counter, y_counter, reg0, reg1, reg2, reg3, reg4, xout, yout, colour_out);
	
	input [15:0] block_counter, x_counter, y_counter; 
	input [11:0] reg0, reg1, reg2, reg3, reg4;
	output reg [7:0] xout;
	output reg [6:0] yout;
  output reg [2:0] colour_out;
	reg [7:0] block;
	
	always @ (*) 
	begin //counter
		colour_out = 3'b111;
		xout = 16'd0;
		yout = ((block_counter % 16'd12) * 16'd10) + y_counter;
		if(block_counter < 16'd12) begin // 0 - 11
			xout = 16'd30 + x_counter;
			if (reg0[block_counter % 16'd12] == 1'b1)
				colour_out = 3'b010;
		end
		else if ((block_counter > 16'd11)&&(block_counter < 16'd24)) begin //12 - 23
			xout = 16'd50 + x_counter;
			if (reg1[block_counter % 16'd12] == 1'b1)
				colour_out = 3'b100;
		end
		else if ((block_counter > 16'd23)&&(block_counter < 16'd36)) begin//24 - 35
			xout = 16'd70 + x_counter;
			if (reg2[block_counter % 16'd12] == 1'b1)
				colour_out = 3'b110;
		end
		else if ((block_counter > 16'd35)&&(block_counter < 16'd48)) begin //36 - 47
			xout = 16'd90 + x_counter;
			if (reg3[block_counter % 16'd12] == 1'b1)
				colour_out = 3'b001;
		end
		else if (block_counter > 16'd47) begin //48 - 59
			xout = 16'd110 + x_counter;
			if (reg4[block_counter % 16'd12] == 1'b1)
				colour_out = 3'b101;
		end
	end
endmodule

module control(
  input clk,
  input resetn,
  input go,
  output reg  shift, load, plot, wait_out, reset_out,
  output reg [15:0] block_counter, x_counter, y_counter);

  reg [63:0] wait_counter;
  reg [63:0] wait_reset;

  reg [3:0] current_state, next_state; 

  localparam
    S_RESET      = 4'd0,
    S_SHIFT   	 = 4'd1,
    S_LOAD       = 4'd2,
    S_PRINT   	 = 4'd3,
    S_WAIT       = 4'd4;

  initial current_state = S_RESET;
  initial block_counter = 1'b0;
  initial x_counter = 1'b0;
  initial y_counter = 1'b0;
  initial wait_counter = 64'd39999999;
  initial wait_reset = 64'd39999999;

  // Next state logic aka our state table
  always@(*)
  begin: state_table 
          case (current_state)
              S_RESET: next_state = go ? S_SHIFT : S_RESET; // Loop in current state until value is input
              S_SHIFT: next_state = go ? S_LOAD : S_RESET; // Loop in current state until go signal goes low
              S_LOAD: next_state = go ? S_PRINT : S_RESET; // Loop in current state until value is input
              S_PRINT: next_state = go ? S_WAIT : S_RESET; // Loop in current state until go signal goes low
              S_WAIT: next_state = go ? S_SHIFT : S_RESET; // Loop in current state until value is input
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
	reset_out = 1'b0;
	 
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
		S_RESET: begin
			reset_out = 1'b1;
			end
    endcase
  end // enable_signals
  
  // current_state registers
  always@(posedge clk)
  begin: state_FFs
    if(!resetn)
    begin
      current_state <= S_RESET;
      block_counter <= 1'b0;
      wait_counter <= wait_reset;
    end
    else
    begin // <--- BEGIN PRINT COUNTER
      if(current_state == S_PRINT)
      begin
        if(block_counter == 16'd59 && x_counter == 16'd19 && y_counter == 16'd9)
          begin
            current_state <= next_state;
            block_counter <= 1'b0;
            x_counter <= 1'b0;
            y_counter <= 1'b0;
          end	
        else if (y_counter == 16'd9 && x_counter == 16'd19)
          begin
            x_counter <= 1'b0;
            y_counter <= 1'b0;
            block_counter <= block_counter + 1'd1;
          end
        else if (x_counter == 16'd19)
          begin
            x_counter <= 1'b0;
            y_counter <= y_counter + 1'd1;
          end
        else
          begin
            x_counter <= x_counter + 1'd1;
          end
      end
    //END PRINT COUNTER

    else if (current_state == S_WAIT)
    begin
      if(wait_counter == 1'b0)
        begin
          current_state <= next_state;
          wait_reset <= wait_reset - 16'd1000;
          wait_counter <= wait_reset;
        end
      else
        wait_counter <= wait_counter - 1;
    end //END WAIT COUNTER
    else
      current_state <= next_state;
  end //end else
end // state_FFS
endmodule

module fibonacci_lfsr_5bit(
  input clk,
  input rst_n,
  output reg [4:0] data);

  initial data <= 5'h1f;
  reg [4:0] data_next;

  always @* begin
    data_next[4] = data[4]^data[1];
    data_next[3] = data[3]^data[0];
    data_next[2] = data[2]^data_next[4];
    data_next[1] = data[1]^data_next[3];
    data_next[0] = data[0]^data_next[2];
  end

  always @(posedge clk or negedge rst_n)
    if(!rst_n)
      data <= 5'h1f;
    else
      data <= data_next;

endmodule

module my_hex_decoder(hex_digit, segments);
  input [3:0] hex_digit;
  output reg [6:0] segments;
  
  always @(*)
    case (hex_digit)
      4'h0: segments = 7'b100_0000;
      4'h1: segments = 7'b111_1001;
      4'h2: segments = 7'b010_0100;
      4'h3: segments = 7'b011_0000;
      4'h4: segments = 7'b001_1001;
      4'h5: segments = 7'b001_0010;
      4'h6: segments = 7'b000_0010;
      4'h7: segments = 7'b111_1000;
      4'h8: segments = 7'b000_0000;
      4'h9: segments = 7'b001_1000;
      4'hA: segments = 7'b000_1000;
      4'hB: segments = 7'b000_0011;
      4'hC: segments = 7'b100_0110;
      4'hD: segments = 7'b010_0001;
      4'hE: segments = 7'b000_0110;
      4'hF: segments = 7'b000_1110;   
      default: segments = 7'b111_1111;
    endcase
endmodule

module BCD(
	input [7:0] data,
	output reg [3:0] hundreds,
	output reg [3:0] tens,
	output reg [3:0] ones
	);
	
	integer i;
	always@(data)
	begin
		hundreds = 4'd0;
		tens = 4'd0;
		ones = 4'd0;
		
		for (i=7; i>= 0; i = i-1)
		begin
			if(hundreds >= 5)
				hundreds = hundreds + 3;
			if(tens >= 5)
				tens = tens + 3;
			if(ones >= 5)
				ones = ones + 3;
			
			hundreds = hundreds << 1;
			hundreds[0] = tens[3];
			tens = tens << 1;
			tens[0] = ones[3];
			ones = ones << 1;
			ones[0] = data[i];
		end //end FOR
	end //end ALWAYS
endmodule

