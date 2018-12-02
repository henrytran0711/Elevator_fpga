

module hex_display_state (state, direction, seg) ;
input [1:0] state;
input direction;
output reg [0:6] seg;
//State truth table//
parameter IDLE =   2'b01;
parameter BUSY =   2'b10;
parameter TRAVELING =     2'b11;   
parameter NULL =   2'b00;
parameter DOWN = 1'b0;
parameter UP = 1'b1;

//HEX Truth Table//
parameter BUSY_HEX =  7'b1100000;
parameter UP_HEX =    7'b1000001;
parameter DOWN_HEX =  7'b1000010;   
parameter IDLE_HEX =  7'b1100000;


always @*
	begin
		case (state)
			TRAVELING: //0 decimal
			begin
				if(direction == UP)
				 seg = UP_HEX;
				else if (direction == DOWN)
				seg = DOWN_HEX;	
			end
			BUSY:
				begin
					seg = BUSY_HEX;
				end
			IDLE:
				begin
					seg = IDLE_HEX;
				end
			
		endcase
end

 
endmodule

module hex_display (w, seg0,seg1) ;
input [3:0] w;
output reg [0:6] seg0;
output reg [0:6] seg1;
//HEX Truth Table//
parameter ZERO =  7'b0000001;
parameter ONE =   7'b1001111;
parameter TWO =   7'b0010010;   
parameter THREE = 7'b0000110;
parameter FOUR =  7'b1001100;
parameter FIVE =  7'b0100100;
parameter SIX =   7'b0100000;
parameter SEVEN = 7'b0001111;
parameter EIGHT = 7'b0000000;
parameter NINE =  7'b0000100;

always @*
	begin
		case (w)
			4'b0000: //0 decimal
			begin
				 seg0 = ZERO;
				 seg1 = ZERO;
			end
			
			4'b0001: //1 decimal
			begin
				 seg0 = ONE;
				 seg1 = ZERO;
			end
			
			4'b0010: //2 decimal
			begin
				 seg0 = TWO;
				 seg1 = ZERO;
			end
			
			4'b0011: //3 decimal
			begin
				 seg0 = THREE;
				 seg1 = ZERO;
			end
			
			4'b0100: //4 decimal
			begin
				 seg0 = FOUR;
				 seg1 = ZERO;
			end
			
			4'b0101: //5 decimal
			begin
				 seg0 = FIVE;
				 seg1 = ZERO;
			end
			
			4'b0110: //6 decimal
			begin
				 seg0 = SIX;
				 seg1 = ZERO;
			end
			
			4'b0111: //7 decimal
			begin
				 seg0 = SEVEN;
				 seg1 = ZERO;	
			end
			4'b1000: //8 decimal
			begin
				 seg0 = EIGHT;
				 seg1 = ZERO;
			end
			4'b1001: // 9decimal
			begin
				 seg0 = NINE;
				 seg1 = ZERO;
			end
			4'b1010: //10 decimal
			begin
				 seg0 = ZERO;
				 seg1 = ONE;
			end
			4'b1011: //11 decimal
			begin
				 seg0 = ONE;
				 seg1 = ONE;
			end
			4'b1100: //12 decimal
			begin
				 seg0 = TWO;
				 seg1 = ONE;
			end
			4'b1101: //13 decimal
			begin
				 seg0 = THREE;
				 seg1 = ONE;
			end
			4'b1110: //14 decimal
			begin
				 seg0 = FOUR;
				 seg1 = ONE;
			end
			4'b1111: //15 decimal
			begin
				 seg0 = FIVE;
				 seg1 = ONE;
			end
			
		
			
		endcase
end

 
endmodule


module hex_display_floor (w, seg0,seg1) ;
input [9:0] w;
output reg [0:6] seg0;
output reg [0:6] seg1;
//HEX Truth Table//
parameter ZERO =  7'b0000001;
parameter ONE =   7'b1001111;
parameter TWO =   7'b0010010;   
parameter THREE = 7'b0000110;
parameter FOUR =  7'b1001100;
parameter FIVE =  7'b0100100;
parameter SIX =   7'b0100000;
parameter SEVEN = 7'b0001111;
parameter EIGHT = 7'b0000000;
parameter NINE =  7'b0000100;

always @*
	begin
		case (w)
			
			10'b0000000001: //1 decimal
			begin
				 seg0 = ONE;
				 seg1 = ZERO;
			end
			
			10'b0000000010:  //2 decimal
			begin
				 seg0 = TWO;
				 seg1 = ZERO;
			end
			
			10'b0000000100:  //3 decimal
			begin
				 seg0 = THREE;
				 seg1 = ZERO;
			end
			
			10'b0000001000: //4 decimal
			begin
				 seg0 = FOUR;
				 seg1 = ZERO;
			end
			
			10'b0000010000: //5 decimal
			begin
				 seg0 = FIVE;
				 seg1 = ZERO;
			end
			
			10'b0000100000: //6 decimal
			begin
				 seg0 = SIX;
				 seg1 = ZERO;
			end
			
			10'b0001000000://7 decimal
			begin
				 seg0 = SEVEN;
				 seg1 = ZERO;
			end
			10'b0010000000://8 decimal
			begin
				 seg0 = EIGHT;
				 seg1 = ZERO;
			end
			10'b0100000000: // 9decimal
			begin
				 seg0 = NINE;
				 seg1 = ZERO;
			end
			10'b1000000000: //10 decimal
			begin
				 seg0 = ZERO;
				 seg1 = ONE;
			end			
			
		endcase
end

 
endmodule




// 1 sec clock
module	counter_1s(CLOCK_50,CLOCK_1);
output	reg CLOCK_1;
input	CLOCK_50;
reg	[25:0] count; //26 bits to represents 50M
always @(posedge CLOCK_50)
begin
  if(count==26'd49_999_999) // if = 50M/ reset
    begin
    count<=26'd0;
    CLOCK_1<=1;   //set CLOCK_1 to 1
    end 
  else //else count up
    begin
  count<=count+1;
  CLOCK_1<=0;
     end// end begin
  end 

endmodule

// 10 Floor Counter

module	counter_floors(CLOCK_1, state, rst,direction, cur_floor);

input [2:0] state;
input direction;
input	CLOCK_1;
input rst;

parameter IDLE =   2'b01;
parameter BUSY =   2'b10;
parameter TRAVELING =     2'b11;   
parameter NULL =   2'b00;
parameter DOWN = 1'b0;
parameter UP = 1'b1;

output reg [9:0] cur_floor; //4 bits to represents 50M
initial 
	begin 	
		cur_floor = 10'b0000000001;
	
	end

always @(posedge CLOCK_1)
begin
	if (rst)
	cur_floor <=  10'b0000000001;
  else if( state == TRAVELING && direction == UP) //If state is traveling up
    begin
		cur_floor <= (cur_floor << 1);
    end 
  else if ( state == TRAVELING && direction == DOWN) // If state is traveling down
    begin
	 	cur_floor <= (cur_floor >> 1);
    end
  else
		cur_floor <= cur_floor;
  end 
endmodule

//Idle Counter

// 10 Floor Counter

module	busy_counter(CLOCK_1, state, busy_clear);

input [2:0] state;
input	CLOCK_1;


parameter IDLE =   2'b01;
parameter BUSY =   2'b10;
parameter TRAVELING =   2'b11;   
parameter NULL =   2'b00;
parameter DOWN = 1'b0;
parameter UP = 1'b1;

reg [3:0]busy_count;

output reg busy_clear;

initial 
	begin 	
		busy_clear = 1'b0;
		busy_count = 3'b000;
		
	end

always @(posedge CLOCK_1)
begin
  if( state == BUSY && busy_count < 3'b101) //If counting
    begin
		busy_count <= (busy_count + 1);
		busy_clear <= 1'b0;
    end 
  else if(state == BUSY && busy_count == 3'b101)
	 begin
	busy_count <= 3'b000;//reset count
	busy_clear <= 1'b1;
	 end
end
  

 endmodule
 
module	check_lower(cur_floor,floor_reg, is_lower);

input [9:0] cur_floor;
input [9:0] floor_reg;

output reg is_lower;
initial 
	begin 	
		is_lower = 1'b0;
		
	end

always @(floor_reg)
begin
  case(cur_floor) 
	
	10'b1000000000:
		if(floor_reg[8]|floor_reg[7]|floor_reg[6]|floor_reg[5]|floor_reg[4]|
		floor_reg[3]|floor_reg[2]|floor_reg[1]|floor_reg[0])
		 is_lower = 1'b1;
		else
		 is_lower = 1'b0;
	10'b0100000000:
		if(floor_reg[7]|floor_reg[6]|floor_reg[5]|floor_reg[4]|
		floor_reg[3]|floor_reg[2]|floor_reg[1]|floor_reg[0])
		 is_lower = 1'b1;
		else
		 is_lower = 1'b0;
	10'b0010000000:
		if(floor_reg[6]|floor_reg[5]|floor_reg[4]|
		floor_reg[3]|floor_reg[2]|floor_reg[1]|floor_reg[0])
		 is_lower = 1'b1;
		else
		 is_lower = 1'b0;
	10'b0001000000:
		if(floor_reg[5]|floor_reg[4]|
		floor_reg[3]|floor_reg[2]|floor_reg[1]|floor_reg[0])
		 is_lower = 1'b1;
		else
		 is_lower = 1'b0;
	10'b0000100000:
		if(floor_reg[4]|
		floor_reg[3]|floor_reg[2]|floor_reg[1]|floor_reg[0])
		 is_lower = 1'b1;
		else
		 is_lower = 1'b0;
	10'b0000010000:
		if(floor_reg[3]|floor_reg[2]|floor_reg[1]|floor_reg[0])
		 is_lower = 1'b1;
		else
		 is_lower = 1'b0;
	10'b0000001000:
		if( floor_reg[2]|floor_reg[1]|floor_reg[0])
		 is_lower = 1'b1;
		else
		 is_lower = 1'b0;
	10'b0000000100:
		if(floor_reg[1]|floor_reg[0])
		 is_lower = 1'b1;
		else
		 is_lower = 1'b0;
	10'b0000000010:
		if(floor_reg[0])
		 is_lower = 1'b1;
		else
		 is_lower = 1'b0;
		 
	endcase
	
end
  

 endmodule



module	state_controller(CLOCK_50,state,is_lower, rst, busy_clear, direction, floor_reg,  cur_floor, direction_out, state_out);
//State: idle, busy,up,down, 2bit?
// 00
// 01
// 10:
// 11:
input [2:0] state;
input [9:0] floor_reg;
input direction;
input [9:0] cur_floor;
input	CLOCK_50;
input busy_clear;
input rst;
input is_lower;

output reg direction_out;
output reg state_out;

parameter IDLE =   2'b01;
parameter BUSY =   2'b10;
parameter TRAVELING =     2'b11;   
parameter NULL =   2'b00;
parameter DOWN = 1'b0;
parameter UP = 1'b1;

initial begin
state_out = IDLE;//initial state
end


always @(posedge CLOCK_50)
begin
  if(rst) begin
  state_out = IDLE;
  end
  case(state)
  
   IDLE:
		begin
			if (floor_reg == 10'b0000000000) //no requested floor
			state_out <= IDLE; // idle
			else if (floor_reg > cur_floor)// Floor requested is higher
					begin
					direction_out <= UP;
					state_out <= TRAVELING;
					end
			else if (floor_reg < cur_floor)// Floor requested is Lower
					begin
					direction_out <= DOWN;
					state_out <= TRAVELING;
					end
			else if (floor_reg == cur_floor)// Floor requested is itself
					begin
					direction_out <= direction_out;// Open door again
					state_out <= BUSY;
					end
		end
	BUSY:
		begin
			if( busy_clear)//Busy clear from busy counter
				begin
					if (floor_reg == 10'b0000000000) //no requested floor
						state_out <= IDLE; // idle
					else if (direction == UP) // If going up, and any floor_reg is high
						begin
						if(floor_reg > cur_floor)
							begin
							direction_out <= UP;
							state_out <= TRAVELING;
							end					
						
						
						else if (floor_reg < cur_floor)
							
							begin
							direction_out <= DOWN;
							state_out <= TRAVELING;
							end
						end
					else if (direction == DOWN) // If going down
						begin
							if(floor_reg > cur_floor && is_lower)//If lower floor requested
							begin
								direction_out <= DOWN;// Keep traveling down
								state_out <= TRAVELING;
							end
							if(floor_reg > cur_floor && !is_lower )//If lower floor isnt
							begin
								direction_out <= UP;//Start to go UP
								state_out <= TRAVELING;
							end
							if(floor_reg < cur_floor  )//lower requested for sure
							begin
								direction_out <= DOWN;//Start to go down
								state_out <= TRAVELING;
							end
						end
				end
			else 
				state_out <= BUSY; //Keep busy till timer clear
		end
	TRAVELING:
		begin
			if((cur_floor[9] & floor_reg[9]) |
			   (cur_floor[8] & floor_reg[8]) |
			   (cur_floor[7] & floor_reg[7]) |
			   (cur_floor[6] & floor_reg[6]) |
			   (cur_floor[5] & floor_reg[5]) |
			   (cur_floor[4] & floor_reg[4]) |
			   (cur_floor[3] & floor_reg[3]) |
			   (cur_floor[2] & floor_reg[2]) |
			   (cur_floor[1] & floor_reg[1]) |
			   (cur_floor[0] & floor_reg[0]) ) //Arrives at requested floor
				state_out <= BUSY;// opening elevator door
		end
		
		
	endcase
end
endmodule


module elevator(CLOCK_50,SW,KEY,HEX0,HEX1,HEX2,HEX3,LEDR);
//SWitches
input [9:0]SW;
input [3:0]KEY;
//Clock
input CLOCK_50;


//LEDs
output [9:0]LEDR;


//HEX
output [0:6]HEX0;
output [0:6]HEX1;
output [0:6]HEX2;
output [0:6]HEX3;
//reg
reg [3:0]floor;
reg [9:0] floor_reg ;



//Assignment

wire rst;
wire CLOCK_1;
wire [1:0]state;
wire direction;
wire busy_clear;
wire [9:0] cur_floor ;
wire is_lower;

assign rst = SW[9];

assign LEDR[9] = floor_reg[9];
assign LEDR[8] = floor_reg[8];
assign LEDR[7] = floor_reg[7];
assign LEDR[6] = floor_reg[6];
assign LEDR[5] = floor_reg[5];
assign LEDR[4] = floor_reg[4];
assign LEDR[3] = floor_reg[3];
assign LEDR[2] = floor_reg[2];
assign LEDR[1] = floor_reg[1];
assign LEDR[0] = floor_reg[0];

initial
	begin
	floor_reg = 10'b0000000000;
	
	
	end

 

hex_display hex_display01(.w({SW[3],SW[2],SW[1],SW[0]}),.seg1(HEX1),.seg0(HEX0) );
hex_display_floor hex_display02(.w(cur_floor),.seg1(HEX3),.seg0(HEX2) );


state_controller state_controller(.CLOCK_50(CLOCK_50),.is_lower(is_lower),.state(state), .rst(rst), .busy_clear(busy_clear), .direction(direction)
,.floor_reg(floor_reg),  .cur_floor(cur_floor), .direction_out(direction), .state_out(state));

busy_counter busy_counter(.CLOCK_1(CLOCK_1), .state(state), .busy_clear(busy_clear));

hex_display_state hex_display_state(.state(state), .direction(direction), .seg(HEX4)) ;
counter_floors	counter_floors(.CLOCK_1(CLOCK_1), .state(state), .direction(direction), .cur_floor(cur_floor));

check_lower check_lower(.cur_floor(cur_floor),.floor_reg(floor_reg), .is_lower(is_lower));

counter_1s counter_1s(.CLOCK_50(CLOCK_50),.CLOCK_1(CLOCK_1));



always @(posedge CLOCK_50, posedge rst, posedge ~KEY[0])
begin
	if(~KEY[0])
	floor <= {SW[3],SW[2],SW[1],SW[0]};
	else 
	floor <= 4'b0000;
	
	if(rst)
	begin
	floor_reg = 10'b0000000000;

	end
	
	case(floor)
	
		4'b0001: //1 decimal
		begin
			floor_reg[0] <= 1'b1;
		
		end	
		
		4'b0010: //2 decimal
		begin
			floor_reg[1] <= 1'b1;
		
		end

		4'b0011: //3 decimal
		begin
			floor_reg[2] <= 1'b1;
		end
		
		4'b0100: //4 decimal
		begin
			floor_reg[3] <= 1'b1;
			
		end
		
		4'b0101: //5 decimal
		begin
			floor_reg[4] <= 1'b1;
			
		end
		
		4'b0110: //6 decimal
		begin
		floor_reg[5] <= 1'b1;
		end
		
		4'b0111: //7 decimal
		begin
		floor_reg[6] <= 1'b1;
			
		end
		
		4'b1000: //8 decimal
		begin
			floor_reg[7] <= 1'b1;
			
		end
		
		4'b1001: // 9 decimal
		begin
			floor_reg[8] <= 1'b1;
		
		end
		
		4'b1010: // 10 decimal
		begin
			floor_reg[9] <= 1'b1;
			
		end

		
		
		default:
			floor_reg <= floor_reg;
			
		endcase
		
	
		case(cur_floor)
				10'b1000000000:
					if(busy_clear)
						floor_reg[9] <= 1'b0;
				10'b0100000000: 
					if(busy_clear)
						floor_reg[8] <= 1'b0;
				10'b0010000000:
					if(busy_clear)
						floor_reg[7] <= 1'b0;
				10'b0001000000:
					if(busy_clear)
						floor_reg[6] <= 1'b0; 
				10'b0000100000: 
					if(busy_clear)
						floor_reg[5] <= 1'b0;
				10'b0000010000: 
					if(busy_clear)
						floor_reg[4] <= 1'b0;
				10'b0000001000:
					if(busy_clear) 
						floor_reg[3] <= 1'b0;
				10'b0000000100:
					if(busy_clear)
						floor_reg[2] <= 1'b0;
				10'b0000000010: 
			        if(busy_clear)
						floor_reg[1] <= 1'b0;
				10'b0000000001: 
				    if(busy_clear)
						floor_reg[0] <= 1'b0;
			
		endcase
		
end


endmodule


