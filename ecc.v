`include "defines.v"
module ecurve_add(input clk,
				  input reset,
				  input [`R_Bits-1:0] p_x,
				  input [`R_Bits-1:0] p_y,
				  input  p_infinity,
				  input [`R_Bits-1:0] q_x,
				  input [`R_Bits-1:0] q_y,
				  input q_infinity,
				  output [`R_Bits-1:0] r_x,
				  output [`R_Bits-1:0] r_y,
				  output r_infinity,
				  output done);
/////State Machine////////////
reg[2:0] state;
//parameter state_init = 0;
parameter state_calc = 0;
parameter state_done = 1;
assign done = (state == state_done);
reg[15:0] cycle_count;
//// Instruction Memory////////////
wire[7:0] instr_addr;
reg [7:0] instr_code;
always @(*) begin
			   //00AAABBB: Load Operands - op1 <= AAA, op2 <= BBB
			   //01AAABBB: Arithmetic Op - AAA:000=sub,001=mul,010=inv,011=mov,100=tstZero	BBB:dest_register
			   //11AAAAAA: jnz
			   //10AAAAAA: jz
			   //11000000: terminate successful
			   //10000000: terminate unsuccessful
	case (instr_addr)// 8'bAABBBCCC	p_x - r0	p_y - r1	q_x = r2	q_y - r3
	8'd0:	 instr_code <= 8'b01100100; //tstz r4				pi
	8'd1:	 instr_code <= 8'b10001001; //jz @p_ni
	8'd2:	 instr_code <= 8'b01100101; //tstz r5				qi
	8'd3:	 instr_code <= 8'b11000110; //jnz @p_i_q_i
	8'd4:	 instr_code <= 8'b00010xxx; //ldo r2, r?			(q_x, ?)
	8'd5:	 instr_code <= 8'b01011000; //mov r0
	8'd6:	 instr_code <= 8'b00011xxx; //ldo r3, r?			(q_y, ?)
	8'd7:	 instr_code <= 8'b01011001; //mov r1
	8'd8:	 instr_code <= 8'b11000000; //return true
	8'd9:	 instr_code <= 8'b10000000; //@p_i_q_i// return false
	8'd10:	 instr_code <= 8'b01100101; //p_ni//tstz r5		(q_i, ?)
	8'd11:	 instr_code <= 8'b10000010; //jz p_ni_q_ni
	8'd12:	 instr_code <= 8'b11000000; //return true
	8'd13:	 instr_code <= 8'b00000010; //p_ni_q_ni//ldo r0, r2	(p_x, q_x)
	8'd14:	 instr_code <= 8'b01000100; //sub r4 				(r4 = p_x - q_x)
	8'd15:	 instr_code <= 8'b01100100; //tstz r4				p_x-q_x
	8'd16:	 instr_code <= 8'b10001000; //jz @DOUBLE
	8'd17:	 instr_code <= 8'b00100xxx; //ldo r4,r?
	8'd18:	 instr_code <= 8'b01010100; //inv r4 				r4 = (p_x-q_x)^(-1)
	8'd19:	 instr_code <= 8'b00001011; //ldo r1, r3 			(p_y, q_y)
	8'd20:	 instr_code <= 8'b01000101; //sub r5 				r5 = p_y - q_y
	8'd21:	 instr_code <= 8'b00100101; //ldo r4, r5 			[(p_x-q_x)^(-1), p_y - q_y]
	8'd22:	 instr_code <= 8'b01001100; //mul r4 				r4 = s = (p_y - q_y)/(p_x-q_x)
	8'd23:	 instr_code <= 8'b11010110; //jnz COMMON
	8'd24:	 instr_code <= 8'b00001011; //@DOUBLE// ldo r1, r3 	(p_y, q_y)
	8'd25:	 instr_code <= 8'b01000100; //sub r4 				r4 = p_y - q_y
	8'd26:	 instr_code <= 8'b01100100; //tstz r4				p_y - q_y
	8'd27:	 instr_code <= 8'b10000010; //jz DOUBLE_1
	8'd28:	 instr_code <= 8'b10000000; //return false
	8'd29:	 instr_code <= 8'b00111011; //DOUBLE_1//ldo r7, r3	(0, q_y)
	8'd30:	 instr_code <= 8'b01000100; //sub r4				r4 = -q_y
	8'd31:	 instr_code <= 8'b00011100; //ldo r3, r4			(q_y, -q_y)
	8'd32:	 instr_code <= 8'b01000011; //sub r3				r3 = q_y - (-q_y) = 2y
	8'd33:	 instr_code <= 8'b01010011; //inv r3				r3 = (2y)^(-1)
	8'd34:	 instr_code <= 8'b00010010; //ldo r2, r2			(q_x, q_x)
	8'd35:	 instr_code <= 8'b01001101; //mul r5				r5 =q_x^2
	8'd36:	 instr_code <= 8'b00111101; //ldo r7, r5			(0, q_x^2)
	8'd37:	 instr_code <= 8'b01000100; //sub r4				r4 = -x^2
	8'd38:	 instr_code <= 8'b00101100; //ldo r5, r4			(x^2, -x^2)
	8'd39:	 instr_code <= 8'b01000101; //sub r5				r5 = 2*x^2
	8'd40:	 instr_code <= 8'b01000101; //sub r5				r5 = 3*x^2
	8'd41:	 instr_code <= 8'b00101110; //ldo r5, r6			(3x^2, -A)
	8'd42:	 instr_code <= 8'b01000100; //sub r4				r4 = 3x^2+A
	8'd43:	 instr_code <= 8'b00011100; //ldo r3, r4			(2y^-1, 3x^2+A)
	8'd44:	 instr_code <= 8'b01001100; //mul r4				r4 = s = (3x^2+A)/2y;
	8'd45:	 instr_code <= 8'b00100100; //COMMON//ldo r4, r4	(s, s)
	8'd46:	 instr_code <= 8'b01001101; //mul r5				r5 = s*s
	8'd47:	 instr_code <= 8'b00101000; //ldo r5, r0			s^2, p_x
	8'd48:	 instr_code <= 8'b01000101; //sub r5				r5 = s^2-px
	8'd49:	 instr_code <= 8'b00101010; //ldo r5, r2			s^2-px, qx
	8'd50:	 instr_code <= 8'b01000010; //sub r2				r2 = s^2-px-qx <== X
	8'd51:	 instr_code <= 8'b00010000; //ldo r2, r0			X, px
	8'd52:	 instr_code <= 8'b01000011; //sub r3				r3 = X-px
	8'd53:	 instr_code <= 8'b00011100; //ldo r3, r4			X-px, s
	8'd54:	 instr_code <= 8'b01001011; //mul r3				r3 = s*(X-px)
	8'd55:	 instr_code <= 8'b00111001; //ldo r7, r1			0, py 
	8'd56:	 instr_code <= 8'b01000001; //sub r1				r1 = -py
	8'd57:	 instr_code <= 8'b00001011; //ldo r1, r3			-py, s*(X-px)
	8'd58:	 instr_code <= 8'b01000001; //sub r1				r1 = -[py+s*(X-px)] <<Y
	8'd59:	 instr_code <= 8'b00010xxx; //ldo r2, r?			X, ?
	8'd60:	 instr_code <= 8'b01011000; //mov r0
	8'd61:	 instr_code <= 8'b11000000; //return true
	default: instr_code <= 8'bx;
	endcase
end	

//// Processor Connection /////////////////////
wire proc_done, proc_succeed;
reg proc_rst;
processor proc( .clk(clk), .rst(proc_rst), .done(proc_done), .succeed(proc_succeed), 
				.data_0(p_x), .data_1(p_y), .data_2(q_x), .data_3(q_y),
				.pi(p_infinity), .qi(q_infinity),
				.result_0(r_x), .result_1(r_y),
				.instr_addr(instr_addr), .instruction(instr_code) 
				);
assign r_infinity = ~proc_succeed;
//// State Transition /////////////////////////
always @(posedge clk or posedge reset) 
if (reset == 1) begin
	state <= state_calc;
	proc_rst <= 1'b1;
	cycle_count <= 0;
end else begin
	cycle_count <= cycle_count + 1'b1;
	proc_rst <= 1'b0;
	case (state)
	state_calc: if (proc_done) begin
		state <= state_done;
	end
	state_done:
		if (cycle_count > 10) $display("ECA: %d cycles.", cycle_count);
	endcase
		
end

endmodule

module old_ecurve_add(input clk,
				  //input trigger,
				  input reset,
				  input [`R_Bits-1:0] p_x,
				  input [`R_Bits-1:0] p_y,
				  input  p_infinity,
				  input [`R_Bits-1:0] q_x,
				  input [`R_Bits-1:0] q_y,
				  input q_infinity,
				  output reg [`R_Bits-1:0] r_x,
				  output reg [`R_Bits-1:0] r_y,
				  output reg r_infinity,
				  //output  [`R_Bits-1:0] d1,
				  //output  [`R_Bits-1:0] d2,
				  //output  [`R_Bits-1:0] d3,
				  //output  [`R_Bits-1:0] d4,
				  output done);


reg[4:0] state;			
reg[4:0] delay_target;
reg[1:0] delay_counter;
reg[19:0] cycle_count;
parameter starting_state = 0;
parameter double_state_0 = 1;
parameter double_state_1 = 2;
parameter double_state_2 = 3;
parameter double_state_3 = 4;
parameter double_state_4 = 5;
parameter double_state_5 = 6;
parameter add_state_0 	 = 7;
parameter add_state_1 	 = 8;
parameter add_state_2 	 = 9;
parameter commonstate_0	 = 10;
parameter commonstate_1	 = 11;
parameter commonstate_2	 = 12;
parameter donestate 	 = 13;
parameter delay_state	 = 14;
parameter commonstate_3	 = 15;

assign done = (state ==donestate);
reg inverter_reset;
wire inverter_done;
wire [`R_Bits-1:0] inverter_r;
reg  [`R_Bits-1:0] inverter_a;
wire same_x;
assign same_x = (p_x == q_x);
mont_invert_p inverter(.clk(clk),.reset(inverter_reset),.a(inverter_a),.result(inverter_r),
			           .done(inverter_done));

reg mul1_reset;
wire mul1_done;
reg [`R_Bits-1:0] mul1_a,mul1_b;
wire [`R_Bits-1:0] mul1_r;
mont_mul_p mul1(.clk(clk),.reset(mul1_reset),.done(mul1_done),
				.a(mul1_a), .b(mul1_b), .r(mul1_r));

reg [`R_Bits-1:0] s;
reg [`R_Bits-1:0] sSquared;

reg [`R_Bits+3:0] t;

reg [`R_Bits-1:0] px_plus_qx,sub_a,sub_b,add_a, add_b;
wire [`R_Bits-1:0] sub_r, add_r;
add_p_p adder(.clk(clk), .a(add_a), .b(add_b), .r(add_r));
subtract_p_p subtractor(.clk(clk),.a(sub_a), .b(sub_b), .r(sub_r));

always @(posedge clk or posedge reset) 
if (reset == 1) begin
	state <= starting_state;
	cycle_count <= 0;
end else begin
	cycle_count <= cycle_count + 1'b1;
	case (state)
		delay_state: begin
			delay_counter <= delay_counter + 1;
			if (delay_counter == 2'b1) begin
				state <= delay_target;
				delay_counter <= 0;
			end
		end
		starting_state : begin
			mul1_reset <= 1;
			delay_counter <= 0;
			inverter_reset <= 1;
			if (p_infinity) begin	//Add a NULL point
				r_infinity <= q_infinity;
				r_x		   <= q_x;
				r_y		   <= q_y;
				state 	   <= donestate;
			end else if (q_infinity) begin
				r_infinity <= p_infinity;
				r_x		   <= p_x;
				r_y		   <= p_y;
				state 	   <= donestate;
			end else if (same_x == 1) begin///Either Double or infinity
				if (p_y == q_y) begin
					delay_target	<=	double_state_0 ;//Double point
					state 			<= 	delay_state;
					add_a <= p_y;
					add_b <= q_y;
				end
				else begin 				  //Infinity
						state <= donestate;
						r_infinity <= 1'b1;
						r_x	<= 0;
						r_y	<= 0;
					 end
			end else begin
				delay_target <= add_state_0 ;	//Normail adding 
				state <= delay_state;
				sub_a <= p_x;
				sub_b <= q_x;
				end
			
		   end //Case 0:
		double_state_0 : begin  //Double point
				inverter_reset <= 0; 					//starting inverter
				//if (inverter_reset == 1)   //First time in this state?
				inverter_a <= add_r;
				mul1_reset <= 0;     					//starting t <= p_x^2
				mul1_a <= p_x; //p_x^2
				mul1_b <= p_x;
				//t	<=  mul1_r + {mul1_r,1'b0} + `EC_A;//starting t <= 3*p_x^2+A
				
				if (mul1_done == 1) begin 				//t <=  is ready
					delay_target <= double_state_1 ;	 
				   state <= delay_state;
				   add_a <= mul1_r;
					add_b <= mul1_r;
					//mul1_reset <= 1;
				end 
		   end
		double_state_1: begin  //Double point 
				mul1_reset <= 1;
				add_a <= mul1_r;
				add_b <= add_r;
				delay_target <= double_state_2; 	
			   state <= delay_state;
			end
		double_state_2: begin
				add_a <= add_r;
				add_b <=  `EC_A;
				delay_target <= double_state_3;	
			   state <= delay_state;
			end
		double_state_3: begin
				mul1_a	<= add_r;			//1264 2e300bcd a9c86b31 dfb301b0 
				mul1_b	<= inverter_r;	
				//s <= mul1_r;		//Move this assignment to the next state
				if (inverter_done == 1) begin
				    inverter_reset <= 1;
		            //$display("ECA: %30H,%30H",inverter_a, inverter_r);
				   mul1_reset <= 0; 				//Start s <=  t*inverter_r
					state  <= double_state_4;
				end
		   end  
		double_state_4: begin //Double point
		      s <= mul1_r;	//1F 4C 2C DB 04 2D 56 B4 C1 39 38 07 55 CD 
				if (mul1_done) begin 					// s ready
					mul1_reset <= 1;
					mul1_a <= mul1_r;
					mul1_b <= mul1_r;      //Start  sSquared <= s*s
					//muls_reset <= 0; 					
					add_a <= p_x;
					add_b <= q_x;
					state <= delay_state;
					delay_target <= commonstate_0;
				end 
		   end
		add_state_0:begin  //Adding point
				inverter_reset <= 0;					//Start inverter
				if(inverter_reset == 1) begin
					inverter_a <= sub_r;//p_x - q_x
					sub_a <= p_y;
					sub_b <= q_y;
				end
				mul1_a <= sub_r;//(p_y - q_y)  ASSUME inverter take more than 2 clks to finish!!
				mul1_b <= inverter_r;
				//s	<= mul1_r;		//Move this assignment to the next state
				if (inverter_done == 1) begin			//inverter done
					inverter_reset<= 1;
					mul1_reset <= 0; 					//start s = gradient
					state <= add_state_1;
				end
		   end
		add_state_1: begin //Adding point
		      s	<= mul1_r;
				if (mul1_done) begin 					//s done
					mul1_reset <= 1;
					mul1_a <= mul1_r; //Prepare data for sSquared
					mul1_b <= mul1_r;
					add_a <= p_x;
					add_b <= q_x;
					delay_target <= commonstate_0;
					state <= delay_state;
				end
		   end
		
///////		
		commonstate_0: begin //Double/ADD common: Wait for sSquared and starting r_x
			px_plus_qx <= add_r;
		    mul1_reset <= 0;
			if (mul1_done) begin					//sSquared done
				//sSquared <= mul1_r;
				sub_a <= mul1_r; //i.e. sSquared
				sub_b <= px_plus_qx;
			   mul1_reset <= 1;
				delay_target <= commonstate_1;
				state <= delay_state;
			end 
		end
		commonstate_1: begin
			delay_target <= commonstate_2;
			state <= delay_state;
			r_x <= sub_r; 
			sub_a <= p_x;
			sub_b <= sub_r;
			end
		commonstate_2: begin // Do r_x and r_y
		   r_infinity <= 0;
			//r_x <= rx_wire;
			mul1_a <= sub_r;//px_sub_rx;
			mul1_b <= s;
			//r_y <= (mul1_r >= p_y)?(mul1_r - p_y):(mul1_r + `Modulus - p_y);
			mul1_reset <= 0;			//start r_y <= -(p_y + s*(rx_sub_px))			
			if (mul1_done) begin
				sub_a <= mul1_r;
				sub_b <= p_y;
				delay_target <= commonstate_3;
				state <= delay_state;
			end	
		   end
		commonstate_3: begin
			r_y <= sub_r;
			state <= donestate;
		end
		donestate: begin/*
		if (r_infinity != 1)
		if (r_y * r_y * 220'h1 %`Modulus  != (r_x*r_x*r_x*512'h1 + r_x*`EC_A*256'h1 + `EC_B) % `Modulus)
          $display("ERR:ECA:(%30H,%30H,%1d)+(%30H,%30H,%1d)=(%30H,%30H,%1d) ",
          p_x*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
          p_y*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
          p_infinity,
          q_x*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
          q_y*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
          q_infinity,
          r_x*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
          r_y*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
          r_infinity)*/
	      //$display("ecc_adder:state done")
			cycle_count <= 0;
			//if (cycle_count > 10) $display("ECA: %d cycles.", cycle_count);
		end
	endcase
end
endmodule

module ecurve_mul(input clk,
                  input reset,
                  input [`R_Bits-1:0] p_x,
                  input [`R_Bits-1:0] p_y,
                  input  p_infinity,
                  input [`EC_Order_Bits-1:0] factor,
                  output [`R_Bits-1:0] r_x,
                  output [`R_Bits-1:0] r_y,
                  output r_infinity,
		            output done);
reg[1:0] state;
reg [8:0] index;

reg adder_reset;
wire adder_done;
reg [`R_Bits-1:0] qx,qy,ac_x,ac_y;
wire [`R_Bits-1:0] rx,ry;
reg qinfinity,ac_infinity;
wire rinfinity;
ecurve_add adder(.clk(clk), .reset(adder_reset), .done(adder_done),
                 .p_x(ac_x), .p_y(ac_y), .p_infinity(ac_infinity),
                 .q_x(qx), .q_y(qy), .q_infinity(qinfinity),
                 .r_x(rx), .r_y(ry), .r_infinity(rinfinity)
                ); 
assign r_x = ac_x;
assign r_y = ac_y;
assign r_infinity = ac_infinity;

parameter starting_state = 0;
parameter doubling_state = 1;
parameter addingP_state = 2;
parameter done_state = 3;
assign done = (state == done_state);
always @(posedge clk or posedge reset) 
if (reset) begin
    state <= starting_state;
end else begin
    case (state)
        starting_state:begin
           index <=  `EC_Order_Bits-1;
           ac_infinity <= 1'b1;
           state <= doubling_state;
           adder_reset <= 1'b1;
          end
        doubling_state:begin //doubling r(x,y)
           adder_reset <= 1'b0;
           qx <= ac_x;
           qy <= ac_y;
           qinfinity <= ac_infinity;
           if (adder_done) begin
              ac_x <= rx;
              ac_y <= ry;
              ac_infinity <= rinfinity;
              adder_reset <= 1'b1;
              state <= addingP_state;    
           end
          end
        addingP_state:begin //adding p(x,y)
           adder_reset <= 1'b0;
           qx <= p_x;
           qy <= p_y;
           qinfinity <= (factor[index]==1'b1)?p_infinity:1'b1;
           if (adder_done) begin
               ac_x <= rx;
               ac_y <= ry;
               ac_infinity <= rinfinity;
               adder_reset <= 1'b1;
               index <= index - 1;
               if (index == 0)
                  state <= done_state;
                else
                  state <= doubling_state;
           end
          end
        done_state:;
   endcase
end
endmodule
		


module ecurve_mul2add(input clk,
           					input reset,
								output done,
           					input[`R_Bits-1:0] GX,
           					input[`R_Bits-1:0] GY,
           					input[`R_Bits-1:0] PX,
           					input[`R_Bits-1:0] PY,
								input[`EC_Order_Bits-1:0] GFactor,
           					input[`EC_Order_Bits-1:0] PFactor,
                  		output [`R_Bits-1:0] r_x,
                  		output [`R_Bits-1:0] r_y,
                  		output r_infinity);
reg[2:0] state;
reg [8:0] index;

reg adder_reset;
wire adder_done;
reg [`R_Bits-1:0] qx,qy,ac_x,ac_y;
wire [`R_Bits-1:0] rx,ry;
reg qinfinity,ac_infinity;
wire rinfinity;
ecurve_add adder(.clk(clk), .reset(adder_reset), .done(adder_done),
                 .p_x(ac_x), .p_y(ac_y), .p_infinity(ac_infinity),
                 .q_x(qx), .q_y(qy), .q_infinity(qinfinity),
                 .r_x(rx), .r_y(ry), .r_infinity(rinfinity)
                ); 
assign r_x = ac_x;
assign r_y = ac_y;
assign r_infinity = ac_infinity;

parameter starting_state = 0;
parameter doubling_state = 1;
parameter addingG_state = 2;
parameter addingP_state = 3;
parameter done_state = 4;
assign done = (state == done_state );

always @(posedge clk or posedge reset) 
if (reset) begin
    state <= starting_state;
end else begin
    case (state)
        starting_state:begin
           index <=  `EC_Order_Bits-1;
           ac_infinity <= 1'b1;
           state <= 1;
           adder_reset <= 1'b1;
          end
        doubling_state :begin //doubling r(x,y)
           adder_reset <= 1'b0;
           qx <= ac_x;
           qy <= ac_y;
           qinfinity <= ac_infinity;
           if (adder_done) begin
              ac_x <= rx;
              ac_y <= ry;
              ac_infinity <= rinfinity;
              adder_reset <= 1'b1;
              state <= addingG_state ;    
           end
          end
        addingG_state :begin //adding G(x,y)
           adder_reset <= 1'b0;
           qx <= GX;
           qy <= GY;
           qinfinity <= (GFactor[index]==1'b1)?1'b0:1'b1;
           if (adder_done) begin
               ac_x <= rx;
               ac_y <= ry;
               ac_infinity <= rinfinity;
               adder_reset <= 1'b1;
               state <= addingP_state ;
           end
          end
        addingP_state :begin //adding P(x,y)
           adder_reset <= 1'b0;
           qx <= PX;
           qy <= PY;
           qinfinity <= (PFactor[index]==1'b1)?1'b0:1'b1;
           if (adder_done) begin
               ac_x <= rx;
               ac_y <= ry;
               ac_infinity <= rinfinity;
               adder_reset <= 1'b1;
               index <= index - 1;
               if (index == 0)
                  state <= done_state ;
                else
                  state <= doubling_state;
           end
          end
        done_state :/*$display("m2a:%x*(%28x,%28x)+%x*(%28x,%28x) = (%28x,%28x)",
                 GFactor,
                 GX*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
                 GY*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
                 PFactor,
                 PX*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
                 PY*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
                 r_x*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
                 r_y*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus
                 )*/ ;
   endcase
end
endmodule
		