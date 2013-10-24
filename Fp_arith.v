`include "defines.v"
/*
module add_p(input [`R_Bits-1:0] a,
			 input [`R_Bits-1:0] b,
			 output [`R_Bits-1:0] r);
wire [`R_Bits:0] t,t2;
wire cout;
lpm_adder a1(.dataa({1'b0,a}), .datab({1'b0,b}), .result(t));

lpm_adder a2(.dataa(t), .datab(~{1'b0,`Modulus}),
			 .cin(1'b1),.cout(cout), .result(t2));
assign r = (cout == 1) ? t2 : t;
endmodule
*/
module add_p_p(input clk, 
			   input [`R_Bits-1:0] a,
			   input [`R_Bits-1:0] b,
			   output [`R_Bits-1:0] r);
/// Result ready after 2 cycles //
reg [`R_Bits:0] r1;
reg [`R_Bits-1:0] r2;
reg cout;
always @(posedge clk) begin
	r1 <= {1'b0,a} + {1'b0,b};
	{cout,r2} <= r1 + (~{1'b0,`Modulus} + 1);
end
assign r = (cout == 1) ? r1 : r2;
endmodule

module add_Mod_Order(input clk, 
			   input [`R_Bits-1:0] a,
			   input [`R_Bits-1:0] b,
			   output [`R_Bits-1:0] r);
/// Result ready after 2 cycles //
reg [`R_Bits:0] r1;
reg [`R_Bits-1:0] r2;
reg cout;
always @(posedge clk) begin
	r1 <= {1'b0,a} + {1'b0,b};
	{cout,r2} <= r1 + (~{1'b0,`EC_Order} + 1);
end
assign r = (cout == 1) ? r1 : r2;
endmodule

module subtract_p_p(input clk,
					input [`R_Bits-1:0] a,
			        input [`R_Bits-1:0] b,
			        output [`R_Bits-1:0] r);
reg [`R_Bits-1:0] r1, r2;
reg cout;
always @(posedge clk) begin
	{cout, r1} <= a - b;
	r2 <= r1 + `Modulus;
end
assign r = (cout == 0) ? r1 : r2;
endmodule

module DualModulusSubtractor(input clk,
                  input alternativeModulus,
		            input [`R_Bits-1:0] a,
	               input [`R_Bits-1:0] b,
			        output [`R_Bits-1:0] r);
reg [`R_Bits-1:0] r1, r2;
reg cout;
always @(posedge clk) begin
	{cout, r1} <= a - b;
	if (alternativeModulus) 
	   r2 <= r1 + `EC_Order;
	else
	   r2 <= r1 + `Modulus;
end
assign r = (cout == 0) ? r1 : r2;
endmodule

/*
module subtract_p(input clk,
					input [`R_Bits-1:0] a,
			        input [`R_Bits-1:0] b,
			        output [`R_Bits-1:0] r);
wire[`R_Bits-1:0] t,t_0,t2;
wire cout;

//lpm_adder	 s1(.dataa({1'b0,a}), .datab({1'b0,~b}),
//			    .cin(1'b1), .result({cout,t}));
lpm_suber	 s1(.dataa(a), .datab(b),
			    .cout(cout), .result(t));
lpm_adder a2(.dataa({1'b0,t}), .datab({1'b0,`Modulus}), .result(t2));

assign r = (cout == 1) ? t : t2;
endmodule
*/

module mont_invert_p(input clk,
				     input reset,
				     input  [`R_Bits-1:0] a,
			         output [`R_Bits-1:0] result,
			         output done
					);

parameter state_init = 0;
parameter state_delay = 4;
parameter state_phase1 = 1;
parameter state_phase2 = 2;
parameter state_done = 3;
reg [2:0] state;
reg [`R_Bits-1:0] u;
reg [`R_Bits-1:0] v;
reg [`R_Bits-1:0] r;
reg [`R_Bits-1:0] s;

reg [8:0] k;

//Overflow detecting Adder
/*
wire [`R_Bits-1:0] x,y;
wire carry_x;
assign {carry_x, x} = u+v;
assign y = r + s;
*/
reg [`R_Bits-1:0] x,y;
reg carry_x;

//Overflow detecting Adder
// 2s-p adder
wire [`R_Bits+1:0] s_0;
assign s_0 = {1'b0,s,1'b0} + {1'b0,~{1'b0,`Modulus}+1};

assign done = (state == state_done);
assign result = s;

always @(posedge clk or posedge reset) 
if (reset == 1) begin
   state <= state_init;
end else begin
{carry_x, x} <= u+v;
y <= r + s;

   case (state) 
   state_init: begin//Load Phase
		u <= ~`Modulus + 1;
		v <= a;
		r <= 0;
		s <= 1;
		k <= 0;
		state <= state_delay;
	 end
  state_delay: state <= state_phase1;
  state_phase1: begin //Phase 1
      if(u[0] == 1'b0) begin
          u[`R_Bits-2:0] <= u >> 1;
          u[`R_Bits-1] <= 1'b1;
          s <= s << 1;
      end else if (v[0] == 1'b0) begin
          v <= v >> 1;
          r <= r << 1;
      end else begin
         if(carry_x == 1'b0) begin
             u[`R_Bits-2:0] <= (x >> 1); //Keep the MSB 1
             u[`R_Bits-1] <= 1;
             r <= y;
             s <= s << 1;
         end else begin
             v[`R_Bits-2:0] <= (x >> 1); //Keep the MSB 1
             v[`R_Bits-1] <= 1'b0;
             if ((|x) == 1) s <= y;
             r <= r << 1;
         end
      end
          
	  if((|x) == 1'b1) begin
		k <= k + 1'b1;
		state <= state_delay;
	  end else begin
		state <= state_phase2;
	  end
		//k <= k + ( ((|x) == 1) ? 1'b1 : 0);
		//state <= ( (|x) == 1 ) ? state_delay : state_phase2;
	end
  state_phase2: begin // Phase 2	 
		k <= k + 1'b1;
      ///CAUTION:: CHANGED MONTGOMERY DOMAIN
      //state <= (k==(2*`R_Bits - 1) ) ? 3 : 2;
      state <= (k==(2*128 - 1) ) ? state_done : state_phase2;
      if (s_0[`R_Bits+1] == 1'b1) // 2s >= p
         s <= s_0[`R_Bits-1:0];
      else
         s <= s << 1;
	end
  state_done: //if (a*result*512'h1 %`Modulus != 110'h14F6139E57115ABC343DE5F3C33A)     
     //$display("INVERTER FAILED: x:%30H, r:%30H, %30H", a, result, a*result*512'h1 %`Modulus )
  ;
  endcase
end
endmodule


module mont_mul_p(input clk,
				     input reset,
				     input [`R_Bits-1:0] a,
			         input [`R_Bits-1:0] b,
			         output [`R_Bits-1:0] r,
			         output done
					);
parameter state_init = 0;
parameter state_loop_ij = 1;
parameter state_loop_j1 = 2;
parameter state_loop_jend = 3;
parameter state_loop_i1 = 4;
parameter state_loop_i2 = 5;
parameter state_loop_i3 = 6;
parameter state_loop_i4 = 7;
parameter state_loop_i5 = 8;
parameter state_loop_j2_1 = 9;
parameter state_loop_j2_2 = 10;
parameter state_loop_j2_end = 11;
parameter state_loop_i6 = 12;
parameter state_loop_i7 = 13;
parameter state_done = 14;
reg  [3:0] state;

wire mul_done;
reg mul_rst;
wire [63:0] mul_r;
reg [31:0] mul_a, mul_b;
mul_32 mul32(.clk(clk), .reset(mul_rst), .done(mul_done), 
             .a(mul_a), .b(mul_b), .r(mul_r));
         

reg [2:0] i,j;
reg [31:0] C,S,m;
//reg [2:0] t[31:0];
reg [191:0] t0;
reg [`R_Bits-1-32:0] n;

reg [`R_Bits-1:0]a_reg;
reg [`R_Bits-1:0]b_reg;
//wire[31:0] Bi;
//assign Bi = b[i*32+31:i*32];
reg[31:0] t0_j;
//assign t0_j = (j (t0>>(32*j));
always @(*)
  case (j[1:0])
	0: t0_j <= t0[31:0];
	1: t0_j <= t0[63:32];
	2: t0_j <= t0[95:64];
	3: t0_j <= t0[127:96];
endcase
assign done = ( state == state_done);


reg cout;
reg [`R_Bits-1:0] adder_r;
//lpm_adder a1(.dataa(t0[`R_Bits:0]), .datab(~{1'b0,`Modulus}),
//			 .cin(1'b1), .cout(cout), .result(adder_r));
assign r = (cout == 0) ? adder_r : t0[`R_Bits-1:0];
always @(posedge clk or posedge reset)
if (reset) 
   state <= state_init;
else begin
   {cout, adder_r} <= t0[`R_Bits:0] - {1'b0,`Modulus}; 
   case (state) 
    state_init: begin //Initialisation
        t0 <= 0;
        mul_rst <= 1;
        i <= 0;
        j <= 0;
        C <= 0;
        S <= 0;
        a_reg <= a;
		  b_reg <= b;
        state <= state_loop_ij;
    end
    state_loop_ij: begin //Loop for i starting, Loop for j_1 start
       mul_b <= (b>>(i*32));//b_reg[31:0];//
       mul_a <= (a>>(j*32));//a_reg[31:0];//
       mul_rst <= 0;
       state <= state_loop_j1; 
    end
       state_loop_j1: begin //Loop for j_1 body
          if (mul_done) begin
              mul_rst <= 1;
             state <= state_loop_jend;
             {C,S} <= mul_r + C + t0_j;
         end
       end
       state_loop_jend: begin //Loop j_1 control
       case (j)
           0:t0[32*0+31:32*0] <= S;
           1:t0[32*1+31:32*1] <= S;
           2:t0[32*2+31:32*2] <= S;
           3:t0[32*3+31:32*3] <= S;
       endcase
          j <= j + 1'b1;
          if ( j != 3 ) begin
		     a_reg <= (a_reg>>32);
             state <= state_loop_ij;
          end else begin
			 a_reg <= a;
             state <= state_loop_i1; 
             j <= 1; //Prepare j for loop j_2
         end
       end
    
    state_loop_i1: begin
        {C,S} <= t0[4*32+31:4*32] + C;
        state <= state_loop_i2;
    end
    state_loop_i2: begin
        t0[4*32+31:4*32] <= S;
        t0[5*32+31:5*32] <= C;
        C <= 0;
        
        mul_a <= t0[0*32+31:0*32];
        mul_b <= `Modulus_1;
        mul_rst <= 0;
        state <= state_loop_i3;
    end
    state_loop_i3: begin
        if (mul_done) begin
            m <= mul_r[31:0];
            mul_rst <= 1;
            state <= state_loop_i4;
        end
    end
    state_loop_i4: begin
        mul_a <= `Modulus;
        mul_b <= m;
        mul_rst <= 0;
        state <= state_loop_i5;
    end
    state_loop_i5: begin
        if(mul_done) begin
            mul_rst <= 1;
            {C,S} <= mul_r + t0[0*32+31:0*32];
            state <= state_loop_j2_1;
            n <= (`Modulus >> 32);
        end
    end
    state_loop_j2_1: begin //Loop for j_2 start
       mul_a <= n[31:0];
       n <= (n >> 32);
       mul_b <= m;
       mul_rst <= 0;
       state <= state_loop_j2_2;
    end
    state_loop_j2_2: begin //Loop for j_2 body
       if(mul_done) begin
           mul_rst <= 1;
           {C,S} <= t0_j + mul_r + C;
           state <= state_loop_j2_end;
       end
   end
   state_loop_j2_end: begin //Loop for j_2 finish
      case (j)
          1:t0[32*0+31:32*0] <= S;
          2:t0[32*1+31:32*1] <= S;
          3:t0[32*2+31:32*2] <= S;
      endcase
      j <= j + 1'b1;
      if (j != 3) 
         state <= state_loop_j2_1;
       else begin
           state <= state_loop_i6;
           j <= 0;
           {C,S} <= t0[4*32+31:4*32] + C;
       end
   end
   state_loop_i6: begin
       t0[3*32+31:3*32] <= S;
       t0[4*32+31:4*32] <= t0[5*32+31:5*32] + C;
       state <= state_loop_i7;
   end
       
   state_loop_i7: begin //Loop for i
       i <= i + 1'b1;
	   b_reg <= (b_reg>>32);
       if (i != 3) begin //Looping
          state <= state_loop_ij;
          C <= 0;
      end else begin
         state <= state_done;//Finishing
	  end
          
   end
   state_done: begin //final state
   //if (a*b*512'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus != r)
   //   $display("mont_mul, a:%40h, b:%40h, r:%40h, %40h", a, b, r, (a*b*512'h0BF0F3C55F4AA2A18DB7B588F916) % `Modulus);
   end
   endcase
end
    
endmodule



/********************OLD IMPLEMENTATION*****************
********************************************************
********************************************************

module mont_mul_pold(input clk,
				  input reset,
				  input [`R_Bits-1:0] a,
			      input [`R_Bits-1:0] b,
			      output [`R_Bits-1:0] r,
			      output done
					);

reg  [1:0] state;

wire [`R_Bits-1:0] mul_a;
wire [`R_Bits-1:0] mul_b;
wire [2*`R_Bits-1:0] mul_r;
reg  mul_reset;
wire mul_done;

reg  [2*`R_Bits:0] T;
reg  [2*`R_Bits:0] temp;
reg  [2*`R_Bits:0] temp2;


mult_110 mul(.clk(clk), .a(mul_a), .b(mul_b), .r(mul_r),
             .reset(mul_reset), .done(mul_done) );
assign mul_a = (state == 0) ? a
              :(state == 1) ? T[`R_Bits-1:0]
              :(state == 2) ? temp
              :255'bx;
assign mul_b = (state == 0) ? b
              :(state == 1) ? `Modulus_1
              :(state == 2) ? `Modulus
              :255'bx;
always @(posedge clk or posedge reset) 
  if (reset) begin
	state <= 0;
	mul_reset <= 1;
  end else begin
  case (state)
	  0:begin /// Do T <= a * b first until done
	    mul_reset <= 0;
		//mul_a <= a; 	//Continuous assignment moved to top
	    //mul_b <= b;
		T     <= mul_r;	//07 16 AE E3 C2 18 3A 8F 69 84 E6 8C 0A 81 5C CB C4 25 91 B1 73 AE D3 7F 5C 88 AF 90 
		state 		<= (mul_done == 1) ? 1 : 0;
		mul_reset 	<= (mul_done == 1) ? 1 : 0;
		end
	 1:begin ///DO temp <= T * Modulus_1 mod R
	    mul_reset <= 0;
		//mul_a <= T[`R_Bits-1:0];		//Continuous assignment moved to top
	    //mul_b <= Modulus_1;
		temp  <= mul_r[`R_Bits-1:0];	//34 D6 52 64 AA E4 F4 01 7F 97 ED DB 0F B0 
		state 		<= (mul_done == 1) ? 2 : 1;
		mul_reset 	<= (mul_done == 1) ? 1 : 0;
	   end
	 2:begin ///Do temp2 <= temp * m
	    mul_reset <= 0;
	    //mul_a <= temp;	//Continuous assignment moved to top
	    //mul_b <= Modulus;
		temp2 <= mul_r;  //09 05 0E B7 BE 03 B2 A1 99 8D 50 5A 75 CD 63 34 3B DA 6E 4E 8C 51 2C 80 A3 77 50 70 
		state 		<= (mul_done == 1) ? 3 : 2;
		mul_reset 	<= (mul_done == 1) ? 1 : 0;
	   end
	 3:;
    endcase
end
wire[`R_Bits:0] temp3;
assign temp3 = (temp2 + T)>>`R_Bits;//40 6E F6 6E 00 6F B4 C4 0C 48 DB 9A 01 3B 
assign r = (temp3 >= `Modulus) ? (temp3 -`Modulus) : temp3 ;//14 BB 69 D2 E2 E5 79 41 FF E3 B7 45 BA D6 
assign done = (state == 3);
endmodule


module mont_invert_p(input clk,
				     input reset,
				     input  [`R_Bits-1:0] a,
			         output [`R_Bits-1:0] result,
			         output done
					);
reg [2:0] state;
reg [`R_Bits:0] u;
reg [`R_Bits:0] v;
reg [`R_Bits+1:0] r;
reg [`R_Bits:0] s;

reg [8:0] k;
wire u_g_v;

assign u_g_v = (u > v) ? 1 : 0;
assign done = (state == 4);
assign result = r;

always @(posedge clk or posedge reset) 
if (reset == 1) begin
   state <= 0;
end else begin
   case (state) 
   0: begin//Load Phase
		u <= `Modulus;
		v <= a;
		r <= 0;
		s <= 1;
		k <= 0;
		state <= 1;
	 end
  1: begin //Phase 1
		u <= (u[0] == 0)       			? u >> 1
		    :((v[0]&u[0]&u_g_v) == 1) 	? ( (u >> 1) - (v >> 1))
		    :u; 
		v <= (u[0] == 1 && v[0] == 0)       ? v >> 1
		    :((v[0]&u[0]&(~u_g_v)) == 1) 	? ((v >> 1) - (u >> 1))
		    :v;
		r <= r + (  (u[0] == 0) ? 0
		           :((v[0]&u[0]&u_g_v) == 1) ? s
		           : r
		         );
		s <= s + (  (u[0] == 1 && v[0] == 0) ? 0
		           :((v[0]&u[0]&(~u_g_v)) == 1) ? r
		           : s
				 );
		k <= k + ( ((|v) == 1) ? 1 : 0);
		state <= ( (|v) == 1 ) ? 1 : 2;
	end
  2: begin
		r <= ( (r>>1) >= `Modulus) ? (2*`Modulus - (r>>1) ) : `Modulus - (r>>1);
      /////CAUTION:: CHANGED MONTGOMERY DOMAIN
		//state <= (k!=2*`R_Bits) ? 3 : 4;
		state <= (k!=2*128) ? 3 : 4;
	end	
  3: begin // Phase 2	 
		k <= k + 1;
      ///CAUTION:: CHANGED MONTGOMERY DOMAIN
      //state <= (k==(2*`R_Bits - 1) ) ? 4 : 3;
      state <= (k==(2*128 - 1) ) ? 4 : 3;
		r <= (r<<1) - (((r<<1)>=`Modulus) ? `Modulus:0);
	end
  4: //if (a*result*512'h1 %`Modulus != 110'h14F6139E57115ABC343DE5F3C33A)     
     //$display("INVERTER FAILED: x:%30H, r:%30H, %30H", a, result, a*result*512'h1 %`Modulus )
  ;
  
  endcase
end
endmodule
*/