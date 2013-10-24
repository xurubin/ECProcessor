`include "defines.v"

module RndGen(input clk,
              input reset,
              input next,
              output done,
              input [`R_Bits-1:0] seed,
              output reg [`R_Bits-1:0] data1,
              output reg [`R_Bits-1:0] data2
              );
reg [1:0] state;
assign done = (state == 2);
reg [8:0] counter;

always @(posedge clk or posedge reset)
if (reset) begin
    counter <= 0;
    state <= 0;
end else begin
    case (state)
      0: begin //New sequence
			data1 <= seed;
			data2 <= seed ^ `SeedDuplicationFactor;
			state <= 1;
         end
      1: begin //Generate new 110 bits
            //if (counter == `R_Bits-1) 
            //   state <= state + 1;
            counter <= counter + 1;
            data1 <= ((data1<<1)|(data1[109]^data1[108]^data1[97]^data1[96]));
            data2 <= ((data2<<1)|(data2[109]^data2[108]^data2[97]^data2[96]));
         end
      2: begin
         // counter <= 0;
         //if (next == 1)
         //   state <= 1;
       end
    endcase 
end
endmodule

                 

module Kangaroo(input clk,
                input reset,
                output done,
                input[`EC_Order_Bits-1:0] initA, //coud reduce them
                input[`EC_Order_Bits-1:0] initB,
                input[`R_Bits-1:0] initX,
                input[`R_Bits-1:0] initY,
                output reg requestRndInit,  
                input RndInitDone,
				    input resume,

                output reg[`EC_Order_Bits-1:0] CoefA,
                output reg[`EC_Order_Bits-1:0] CoefB,
                output reg[`R_Bits-1:0] RX,
                output reg[`R_Bits-1:0] RY,
                output reg RInfinity
                );
reg[2:0] state;
parameter init_state  = 0;
parameter lookup_state = 1;
parameter calc_state = 2;
parameter done_state = 3;

//////////Instantiation of ecurve_adder///////////////////////
reg eca_reset;
wire eca_done;
wire [`R_Bits-1:0] eca_rx,eca_ry;
reg  [`R_Bits-1:0] eca_px, eca_py;
wire eca_rinfinity;
ecurve_add adder(.clk(clk), .reset(eca_reset), .done(eca_done),
                 .p_x(RX), .p_y(RY), .p_infinity(RInfinity),
                 .q_x(eca_px), .q_y(eca_py), .q_infinity(1'b0),
                 .r_x(eca_rx), .r_y(eca_ry), .r_infinity(eca_rinfinity)
                ); 
             
wire[`EC_Order_Bits:0] nextCoefA,nextCoefB;
reg[`EC_Order_Bits:0] NAt,NA,NB,NBt;
reg[`EC_Order_Bits-1:0] NAt2,NBt2;
reg NA_carry,NB_carry;
reg[2:0] delay;

wire[109:0]storedA, storedB, storedX, storedY;
reg[`RandomWalkBranchBits-1:0] rm_addr;
myrom RM(.addr(rm_addr), .clk(clk), .data({storedA, storedB, storedX, storedY}));

always @(posedge clk) begin
	NA <= storedA;
	NAt <= CoefA + NA;
	{NA_carry, NAt2} <= NAt + ( ~{1'b1, `EC_Order} + 1'b1);
	NB <= storedB;
	NBt <= CoefB + NB;
	{NB_carry, NBt2} <= NBt + ( ~{1'b1, `EC_Order} + 1'b1);
end
assign nextCoefA = (NA_carry == 0) ? NAt : NAt2;
assign nextCoefB = (NB_carry == 0) ? NBt : NBt2;

assign done = (state == done_state);

reg[2*`Mask_Bit_Len:0] iterCnt;
always @(posedge clk or posedge reset) 
if (reset == 1) begin
    state <= init_state;
    requestRndInit <= 1;
end else begin
    case (state)
      init_state:begin //Initialise random walk
		  delay <= 2'b0;
        rm_addr <= initX[`RandomWalkBranchBits + `Mask_Bit_Len - 1 : `Mask_Bit_Len];
        if (RndInitDone == 1) begin
              requestRndInit <= 0;
              CoefA <= initA;
              CoefB <= initB;
              RX    <= initX;
              RY    <= initY;
              eca_reset <= 1;
              iterCnt <= 0;
              if ((|initX[`Mask_Bit_Len-1:0]) == 0) //Happen to be a distinguished pt
                 state <= done_state;
              else begin
                 state <= lookup_state;
                 eca_reset <= 1'b1;
		      end
        end
	  end
	  lookup_state:begin //Request a Lookup
		//if (LookupDone) begin
			delay <= delay + 2'b1;
			if (delay == 3'd4) begin
				delay <= 2'b0;
            CoefA <= nextCoefA;
            CoefB <= nextCoefB;
            eca_px <= storedX;
            eca_py <= storedY;
				/*
			  $display("Kangaroo:Next Pt Incr(%d): %h G+%h K = (%28x,%28x)A",
					 LookupID, initA, initB,
					 initX*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
					 initY*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
			  $display("Kangaroo:Current Pt:  %h G+%h K = (%28x,%28x)A",
					 CoefA, CoefB,
					 RX*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
					 RY*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
					 */
				eca_reset <= 1'b0;
				state <= calc_state;
			end
		//end
	  end
	  calc_state:begin
          rm_addr <= eca_rx[`RandomWalkBranchBits + `Mask_Bit_Len - 1 : `Mask_Bit_Len];
          if (eca_done == 1) begin//next point calculated
             //$display("Kangaroo:Point calculated:%x", eca_rx);
			    eca_reset <= 1'b1;
             iterCnt <= iterCnt + 1;
			    RX <= eca_rx;
			    RY <= eca_ry;
             RInfinity <= eca_rinfinity;
             if(eca_rinfinity == 1 || ((|eca_rx[`Mask_Bit_Len-1:0]) == 0))
                state <= done_state;
			 else begin
			    state <= lookup_state;
			 end
		  end
      end
      done_state:begin//DP found, restart 
          if (resume == 1) state <= init_state;
          requestRndInit <= 1;
          if (requestRndInit == 0)begin
          $display("Kangaroo:DP found after %d iterations", iterCnt);
          $display("Kangaroo:(%28x,%28x)M, (%28x,%28x)A",
                 RX, RY,
                 RX*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
                 RY*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
             end
                    
      end
  endcase
end
endmodule


module arbitrator(input rst, input clk,
				  input A_Req,
				  input [4:0] A_Addr,
				  output reg A_Done,
				  output reg [439:0] A_Data,
				  input B_Req,
				  input [4:0] B_Addr,
				  output reg B_Done,
				  output reg [439:0] B_Data,
				  input C_Req,
				  input [4:0] C_Addr,
				  output reg C_Done,
				  output reg [439:0] C_Data,
				  input D_Req,
				  input [4:0] D_Addr,
				  output reg D_Done,
				  output reg [439:0] D_Data);

reg [1:0] client;				  
wire[4:0] rm_addr;
wire[439:0] rm_output;
//if (`SIMULATION == 1) 
   myrom RM(.addr(rm_addr), .clk(clk), .data(rm_output));
// else
//   random_mapping RM(.address(rm_addr),.clock(clk),.q(rm_output));
   
assign rm_addr = (client == 2'd0) ? A_Addr
				:(client == 2'd1) ? B_Addr
				:(client == 2'd2) ? C_Addr
				: D_Addr;
				
reg [3:0]state, last_state;
parameter s_Standby = 0;
parameter s_ServingA = 1;
parameter s_ServingB = 2;
parameter s_ServingC = 3;
parameter s_ServingD = 4;
always @(posedge clk or posedge rst)
if (rst) begin
	state <= s_Standby;
	last_state <= s_Standby;
end else begin
	last_state <= state;
	case (state) 
		s_Standby:	if(A_Req)		state <= s_ServingA;
					else if (B_Req) state <= s_ServingB;
					else if (C_Req) state <= s_ServingC;
					else if (D_Req) state <= s_ServingD;
					else state <= s_Standby;
		s_ServingA : if (!A_Req)	state <= s_Standby;
					else state <= s_ServingA;
		s_ServingB : if (!B_Req)	state <= s_Standby;
					else state <= s_ServingB;
		s_ServingC : if (!C_Req)	state <= s_Standby;
					else state <= s_ServingC;
		s_ServingD : if (!D_Req)	state <= s_Standby;
					else state <= s_ServingD;
	endcase
end

always @(*) begin
	case (state)
		s_ServingA: client <= 0;
		s_ServingB: client <= 1;
		s_ServingC: client <= 2;
		s_ServingD: client <= 3;
		default: client <= 2'bx;
	endcase
	A_Done <= (state == s_ServingA)&&(last_state == s_ServingA);
	B_Done <= (state == s_ServingB)&&(last_state == s_ServingB);
	C_Done <= (state == s_ServingC)&&(last_state == s_ServingC);
	D_Done <= (state == s_ServingD)&&(last_state == s_ServingD);
	if ((state == s_ServingA)&&(last_state == s_ServingA))
		A_Data <= rm_output;
	else A_Data <= 439'bx;
	if ((state == s_ServingB)&&(last_state == s_ServingB))
		B_Data <= rm_output;
	else B_Data <= 439'bx;
	if ((state == s_ServingC)&&(last_state == s_ServingC))
		C_Data <= rm_output;
	else C_Data <= 439'bx;
	if ((state == s_ServingD)&&(last_state == s_ServingD))
		D_Data <= rm_output;
	else D_Data <= 439'bx;
end

always @(posedge clk) begin
	/*
	case (state)
		s_ServingA: if (last_state == s_ServingA)
						A_Data <= rm_output;
		s_ServingB: if (last_state == s_ServingB)
						B_Data <= rm_output;
		s_ServingC: if (last_state == s_ServingC)
						C_Data <= rm_output;
		s_ServingD: if (last_state == s_ServingD)
						D_Data <= rm_output;
	endcase
	*/
end

endmodule

module Zoo(input clk,
           input reset,
		   input resume,
           input[`R_Bits-1:0] masterSeed,
           input[`R_Bits-1:0] GX,
           input[`R_Bits-1:0] GY,
           input[`R_Bits-1:0] PX,
           input[`R_Bits-1:0] PY,
           output [2*`R_Bits-1:0] T1,
			output [3:0] internal_state,
			output [8:0] count
			);
reg [9:0] state;
///////////Psedurandom number generation///////////////
wire rnd_done;
reg  rnd_reset,rnd_next;
wire [`EC_Order_Bits-1:0] rnd_data1,rnd_data2; //Generating COefA&CoefB
//reg  [`R_Bits-1:0] rnd_seed;

RndGen rnd(.clk(clk), .reset(rnd_reset), .done(rnd_done), .next(1'b1),
            .seed(masterSeed),.data1(rnd_data1),.data2(rnd_data2));

//////////Intialise a*G+b*P mulAdder ////////////////////////////
reg m2a_reset;
wire m2a_done;

wire [`R_Bits-1:0] m2a_rx,m2a_ry;
wire m2a_ri;
reg  [`EC_Order_Bits-2:0] m2a_GFactor,m2a_PFactor;

ecurve_mul2add mul2adder(.clk(clk),.reset(m2a_reset),.done(m2a_done),
           				       .GX(GX),.GY(GY),.PX(PX),.PY(PY),
							 .GFactor({1'b0,m2a_GFactor}),
							 .PFactor({1'b0,m2a_PFactor}),
                  	 .r_x(m2a_rx),.r_y(m2a_ry),.r_infinity(m2a_ri));

/////////Initialise Kangaroos////////////////////////////////////
reg k1_rst;
wire k1_done;
wire k1_requestInit;
wire [`EC_Order_Bits-1:0] k1_ra,k1_rb;
wire [`R_Bits-1:0] k1_rx,k1_ry;
reg k1_initDone,k1_resume;
wire k1_ri;

Kangaroo K1(.clk(clk),.reset(k1_rst),.done(k1_done),
					 .initA( {1'b0,m2a_GFactor} ),
					 .initB( {1'b0,m2a_PFactor} ),
					 .initX( m2a_rx 	 		),
					 .initY( m2a_ry 	 		),
					 .requestRndInit(k1_requestInit),  
					 .resume(k1_resume),
					 .RndInitDone(k1_initDone),
					 .CoefA(k1_ra), .CoefB(k1_rb), 
					 .RX(k1_rx), .RY(k1_ry), .RInfinity(k1_ri)
				);
/////////Initialise Kangaroos////////////////////////////////////
reg k2_rst;
wire k2_done;
wire k2_requestInit;
reg k2_initDone,k2_resume;
wire [`EC_Order_Bits-1:0] k2_ra,k2_rb;
wire [`R_Bits-1:0] k2_rx,k2_ry;
wire k2_ri;
//assign k2_done = 0;	assign k2_requestInit = 0;

Kangaroo K2(.clk(clk),.reset(k2_rst),.done(k2_done),
					 .initA( {1'b0,m2a_GFactor} ),
					 .initB( {1'b0,m2a_PFactor} ),
					 .initX( m2a_rx 	 		),
					 .initY( m2a_ry 	 		),
					 .requestRndInit(k2_requestInit),  
					 .resume(k2_resume),
                .RndInitDone(k2_initDone),
					 .CoefA(k2_ra), .CoefB(k2_rb), 
					 .RX(k2_rx), .RY(k2_ry), .RInfinity(k2_ri)
				);

/////////Initialise Kangaroos////////////////////////////////////
reg k3_rst;
wire k3_done;
wire k3_requestInit;
reg k3_initDone,k3_resume;
wire [`EC_Order_Bits-1:0] k3_ra,k3_rb;
wire [`R_Bits-1:0] k3_rx,k3_ry;
wire k3_ri;
//assign k3_done = 0;	assign k3_requestInit = 0;

Kangaroo K3(.clk(clk),.reset(k3_rst),.done(k3_done),
					 .initA( {1'b0,m2a_GFactor} ),
					 .initB( {1'b0,m2a_PFactor} ),
					 .initX( m2a_rx 	 		),
					 .initY( m2a_ry 	 		),
					 .requestRndInit(k3_requestInit),  
					 .resume(k3_resume),
                .RndInitDone(k3_initDone),
					 .CoefA(k3_ra), .CoefB(k3_rb), 
					 .RX(k3_rx), .RY(k3_ry), .RInfinity(k3_ri)
				);

/////////Initialise Kangaroos////////////////////////////////////
reg k4_rst;
wire k4_done;
wire k4_requestInit;
reg k4_initDone,k4_resume;
wire [`EC_Order_Bits-1:0] k4_ra,k4_rb;
wire [`R_Bits-1:0] k4_rx,k4_ry;
wire k4_ri;
assign k4_done = 0;	assign k4_requestInit = 0;
/*
Kangaroo K4(.clk(clk),.reset(k4_rst),.done(k4_done),
					 .initA( {1'b0,m2a_GFactor} ),
					 .initB( {1'b0,m2a_PFactor} ),
					 .initX( m2a_rx 	 		),
					 .initY( m2a_ry 	 		),
					 .requestRndInit(k4_requestInit),  
					 .resume(k4_resume),
                .RndInitDone(k4_initDone),
					 .CoefA(k4_ra), .CoefB(k4_rb), 
					 .RX(k4_rx), .RY(k4_ry), .RInfinity(k4_ri)
				);
*/
////////Initialisation Kangaroos finished///////////////////////
////////Psuedorandom Mapping function (ROM)///////////////////////
/*
arbitrator arb(.rst(reset), .clk(clk),
		.A_Req(k4_reqLookup), .A_Addr(k1_index),.A_Done(k1_lookupDone), .A_Data(k1_mapping),
		.B_Req(k2_reqLookup), .B_Addr(k2_index),.B_Done(k2_lookupDone), .B_Data(k2_mapping),
		.C_Req(k3_reqLookup), .C_Addr(k3_index),.C_Done(k3_lookupDone), .C_Data(k3_mapping),
		.D_Req(k4_reqLookup), .D_Addr(k4_index),.D_Done(k4_lookupDone), .D_Data(k4_mapping)
);
*/
////////DP Point Storage RAM///////////////////////////////////////
reg[2*`R_Bits-1:0] mem_data;
reg[8:0] mem_addr;
reg mem_wren;
ram mem(.address(mem_addr),.clock(clk),.data(mem_data),.wren(mem_wren), .q(T1));
reg[`RandomWalkBranchBits+2:0] i;

////////Output debug information//////////////////////
assign count = mem_addr;
assign internal_state = {k4_requestInit, k3_requestInit, k2_requestInit, k1_requestInit};
parameter done_state = 8;
always @(posedge clk or posedge reset) 
if(reset) begin
    state <= 0;
	rnd_reset <= 1;
end else begin case (state)
0:begin/////Initialse
   //rnd_seed <= masterSeed;
   rnd_reset <= 0;
   m2a_reset <= 1;
   k1_rst <= 1;   k2_rst <= 1;   k3_rst <= 1;   //k4_rst <= 1;
   state <= 1;
   mem_wren <= 0;
   mem_addr <= 0;
end
1:begin //Wait for PRNG to initialise
   state <= 2;
end
2:begin ///Initialse all Kangaroos
	//rnd_next <= 0;
	k1_rst <= 0;		k2_rst <= 0;		k3_rst <= 0;		//k4_rst <= 0;
	k1_initDone <= 0;	k2_initDone <= 0;	k3_initDone <= 0;	//k4_initDone <= 0;
	rnd_reset <= 0;
	mem_wren <= 1'b0;

	k1_resume <= 0;		k2_resume <= 0;		k3_resume <= 0;		//k4_resume <= 0;
	if (mem_addr == 9'h1FF)	
		state <= done_state;
	else if (k1_done&!k1_resume) begin
		$display("ZOO_K1: Write to RAM: %h", {k1_ra,k1_rb});
		/// Write to Memory
		mem_data <= {k1_ra, k1_rb};
		mem_wren <= 1'b1;
		mem_addr <= mem_addr + 1'b1;
		k1_resume <= 1;
	end else if (k2_done&!k2_resume) begin
		$display("ZOO_K2: Write to RAM: %h", {k2_ra,k2_rb});
		/// Write to Memory
		mem_data <= {k2_ra, k2_rb};
		mem_wren <= 1'b1;
		mem_addr <= mem_addr + 1'b1;
		k2_resume <= 1;
	end else if (k3_done&!k3_resume) begin
		$display("ZOO_K3: Write to RAM: %h", {k3_ra,k3_rb});
		/// Write to Memory
		mem_data <= {k3_ra, k3_rb};
		mem_wren <= 1'b1;
		mem_addr <= mem_addr + 1'b1;
		k3_resume <= 1;
	end/* else if (k4_done&!k4_resume) begin
		$display("ZOO_K4: Write to RAM: %h", {k4_ra,k4_rb});
		/// Write to Memory
		mem_data <= {k4_ra, k4_rb};
		mem_wren <= 1'b1;
		mem_addr <= mem_addr + 1'b1;
		k4_resume <= 1;
	end	*//////////MEMORY WRITE DONE, CHECK Kn Initialisation request. 
	else if (k1_requestInit|k2_requestInit|k3_requestInit) begin//|k4_requestInit) begin
		m2a_reset <= 0;
		m2a_GFactor <= (`SIMULATION == 1) ? {106'b0,rnd_data1[3:0]} : rnd_data1;
		m2a_PFactor <= (`SIMULATION == 1) ? {106'b0,rnd_data2[3:0]} : rnd_data2;
		$display("Initialisation factor:%h %h", rnd_data1, rnd_data2);
		if (k1_requestInit)
		   state <= 3;
		else if (k2_requestInit)
		   state <= 4;
		else if (k3_requestInit)
		   state <= 5;
		/*else if (k4_requestInit)
		   state <= 6;*/
		else state <= state;
	end
end
3:begin //K1 Initialisation 
	if (k1_requestInit&m2a_done) begin
		k1_initDone <= 1;
		m2a_reset <= 1;
		$display("Kangaroo 1 Initialised.");
	end
	if(k1_initDone)//Delay initDone signal for 1 clk edge
		state <= 2;
end
4:begin //K2 Initialisation 
	if (k2_requestInit&m2a_done) begin
		k2_initDone <= 1;
		m2a_reset <= 1;
		$display("Kangaroo 2 Initialised.");
	end
	if(k2_initDone)//Delay initDone signal for 1 clk edge
		state <= 2;
end
5:begin //K3 Initialisation 
	if (k3_requestInit&m2a_done) begin
		k3_initDone <= 1;
		m2a_reset <= 1;
		$display("Kangaroo 3 Initialised.");
	end
	if(k3_initDone)//Delay initDone signal for 1 clk edge
		state <= 2;
end/*
6:begin //K4 Initialisation 
	if (k4_requestInit&m2a_done) begin
		k4_initDone <= 1;
		m2a_reset <= 1;
		$display("Kangaroo 4 Initialised.");
	end
	if(k4_initDone)//Delay initDone signal for 1 clk edge
		state <= 2;
end*/
done_state:begin
	if (resume) state <= 0;
end
endcase
end

endmodule
				 
/* 		

module Kangaroo(input clk,
                input reset,
                output done,
                input[`EC_Order_Bits-2:0] initA, //coud reduce them
                input[`EC_Order_Bits-2:0] initB,
                input[`R_Bits-1:0] initX,
                input[`R_Bits-1:0] initY,
                output[`RandomWalkBranchBits-1:0] LookupID,
                output reg requestRndInit,  
                //requestRndInit == 1 initA/B/X/Y is the initial values
                //Otherwise they represents the LookupID entry 
                input RndInitDone,
				input resume,

                output reg[`EC_Order_Bits-1:0] CoefA,
                output reg[`EC_Order_Bits-1:0] CoefB,
                output reg[`R_Bits-1:0] RX,
                output reg[`R_Bits-1:0] RY,
                output reg RInfinity
                );
reg[3:0] state;

//////////Instantiation of ecurve_adder///////////////////////
reg eca_reset;
wire eca_done;
wire [`R_Bits-1:0] eca_rx,eca_ry;
wire eca_rinfinity;
ecurve_add adder(.clk(clk), .reset(eca_reset), .done(eca_done),
                 .p_x(RX), .p_y(RY), .p_infinity(RInfinity),
                 .q_x(initX), .q_y(initY), .q_infinity(1'b0),
                 .r_x(eca_rx), .r_y(eca_ry), .r_infinity(eca_rinfinity)
                ); 
                
assign LookupID = RX[`RandomWalkBranchBits + `Mask_Bit_Len - 1 : `Mask_Bit_Len];
wire[`EC_Order_Bits:0] nextCoefA,nextCoefB;
wire[`EC_Order_Bits:0] NAt,NBt;
wire[`EC_Order_Bits-1:0] NAt2,NBt2;
wire NA_carry;
assign NAt = CoefA + initA;
assign {NA_carry, NAt2} = NAt + ( ~{1'b1, `EC_Order} + 1'b1);
assign nextCoefA = (NA_carry == 0) ? NAt : NAt2;
assign NBt = CoefB + initB;
assign {NB_carry, NBt2} = NBt + ( ~{1'b1, `EC_Order} + 1'b1);
assign nextCoefB = (NB_carry == 0) ? NBt : NBt2;
//assign nextCoefA = CoefA + initA;
//assign nextCoefB = CoefB + initB;

assign done = (state == 2);

reg[2*`Mask_Bit_Len:0] iterCnt;
always @(posedge clk or posedge reset) 
if (reset == 1) begin
    state <= 0;
    requestRndInit <= 1;
end else begin
    case (state)
      0:begin //Initialise random walk
          if (RndInitDone == 1) begin
              requestRndInit <= 0;
              CoefA <= initA;
              CoefB <= initB;
              RX    <= initX;
              RY    <= initY;
              eca_reset <= 1;
              iterCnt <= 0;
              if ((|initX[`Mask_Bit_Len-1:0]) == 0) //Happen to be a distinguished pt
                 state <= 2;
                else
                 state <= state + 1;
                 
          end
      end
      1:begin
          eca_reset <= 0;
          if (eca_done == 1) begin//next point calculated
             RX <= eca_rx;
             RY <= eca_ry;
             $display("Kangaroo:Point calculated:%x", eca_rx);
             iterCnt <= iterCnt + 1;
             //CoefA <= (nextCoefA>=`EC_Order)?(nextCoefA-`EC_Order):nextCoefA;
             //CoefB <= (nextCoefB>=`EC_Order)?(nextCoefB-`EC_Order):nextCoefB;
			 CoefA <= nextCoefA;
			 CoefB <= nextCoefB;
             RInfinity <= eca_rinfinity;
             if(eca_rinfinity == 1 || ((|eca_rx[`Mask_Bit_Len-1:0]) == 0))
                state <= state +1;
             eca_reset <= 1;
          end
      end
      2:begin//DP found, restart 
          if (resume == 1) state <= 0;
          requestRndInit <= 1;
          if (requestRndInit == 0)begin
          $display("Kangaroo:DP found after %d iterations", iterCnt);
          $display("Kangaroo:(%28x,%28x)M, (%28x,%28x)A",
                 RX, RY,
                 RX*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus,
                 RY*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
             end
                    
      end
  endcase
end
endmodule

			
module Zoo(input clk,
           input reset,
           input[`R_Bits-1:0] masterSeed,
           input[`R_Bits-1:0] sessionSeed,
           input[`R_Bits-1:0] GX,
           input[`R_Bits-1:0] GY,
           input[`R_Bits-1:0] PX,
           input[`R_Bits-1:0] PY,
           output [2*`R_Bits-1:0] T1,
			output [3:0] internal_state,
			output [7:0] count
			);
reg [9:0] state;
assign internal_state = state;
reg[`R_Bits-1:0] f[((1'b1<<`RandomWalkBranchBits)*4-1):0];
///////////Psedurandom number generation///////////////
wire rnd_done;
reg  rnd_reset,rnd_next;
wire [`EC_Order_Bits-1:0] rnd_data1,rnd_data2; //Generating COefA&CoefB
reg  [`R_Bits-1:0] rnd_seed;

RndGen rnd(.clk(clk), .reset(rnd_reset), .done(rnd_done), .next(1'b1),
            .seed(rnd_seed),.data1(rnd_data1),.data2(rnd_data2));

//////////Intialise a*G+b*P mulAdder ////////////////////////////
reg m2a_reset;
wire m2a_done;

wire [`R_Bits-1:0] m2a_rx,m2a_ry;
wire m2a_ri;
reg  [`EC_Order_Bits-2:0] m2a_GFactor,m2a_PFactor;

ecurve_mul2add mul2adder(.clk(clk),.reset(m2a_reset),.done(m2a_done),
           				       .GX(GX),.GY(GY),.PX(PX),.PY(PY),
							 .GFactor({1'b0,m2a_GFactor}),
							 .PFactor({1'b0,m2a_PFactor}),
                  	 .r_x(m2a_rx),.r_y(m2a_ry),.r_infinity(m2a_ri));

/////////Initialise Kangaroos////////////////////////////////////
reg k1_rst;
wire k1_done;
wire[`RandomWalkBranchBits - 1:0] k1_index;
wire k1_requestInit;
reg k1_initDone,k1_resume;
wire [`EC_Order_Bits-1:0] k1_ra,k1_rb;
wire [`R_Bits-1:0] k1_rx,k1_ry;
wire k1_ri;
Kangaroo K1(.clk(clk),.reset(k1_rst),.done(k1_done),
					 .initA( (k1_requestInit == 1) ? m2a_GFactor : (f[k1_index*4+0]) ),
					 .initB( (k1_requestInit == 1) ? m2a_PFactor : (f[k1_index*4+1]) ),
					 .initX( (k1_requestInit == 1) ? m2a_rx 	 : (f[k1_index*4+2]) ),
					 .initY( (k1_requestInit == 1) ? m2a_ry 	 : (f[k1_index*4+3]) ),
					 .LookupID(k1_index),.requestRndInit(k1_requestInit),  
					 .resume(k1_resume),
                //requestRndInit == 1 initA/B/X/Y is the initial values
                //Otherwise they represents the LookupID entry 
					 .RndInitDone(k1_initDone),
					 .CoefA(k1_ra), .CoefB(k1_rb), 
					 .RX(k1_rx), .RY(k1_ry), .RInfinity(k1_ri)
				);
/////////Initialise Kangaroos////////////////////////////////////
reg k2_rst;
wire k2_done;
wire[`RandomWalkBranchBits - 1:0] k2_index;
wire k2_requestInit;
reg k2_initDone,k2_resume;
wire [`EC_Order_Bits-1:0] k2_ra,k2_rb;
wire [`R_Bits-1:0] k2_rx,k2_ry;
wire k2_ri;
assign k2_done = 0;
assign k2_requestInit = 0;
////////Initialisation Kangaroos finished///////////////////////
reg[2*`R_Bits-1:0] mem_data;
reg[8:0] mem_addr;
reg mem_wren;
ram mem(.address(mem_addr),.clock(clk),.data(mem_data),.wren(mem_wren), .q(T1));
assign count = mem_addr;

always @(posedge clk or posedge reset) 
if(reset) begin
    state <= 0;
	 rnd_reset <= 1;
end else begin case (state)
0:begin/////Initialse PseduoMapping f - Init Rnd
   $display("Initialising f...");
   rnd_seed <= masterSeed;
   rnd_reset <= 0;
   m2a_reset <= 1;
   k1_rst <= 1;
   k2_rst <= 1;
   if (rnd_reset == 0) state <= state + 1;
   i <= 0;
   mem_wren <= 0;
   mem_addr <= 0;
end
1:begin/////Initialse PseduoMapping f - Generate A&B
   //if (rnd_done) begin
      //rnd_next <= 1;
		f[i] <= rnd_data1;
		f[i+1] <= rnd_data2;
		//m2a_GFactor <= rnd_data1;
		//m2a_PFactor <= rnd_data2;
		i <= i + 2;
		state <= state + 1;
		m2a_reset <= 0;	//Start mul2Adder
		m2a_GFactor <= rnd_data1;
		m2a_PFactor <= rnd_data2;
   //end
end
2:begin/////Find A*G+B*P
	//rnd_next <= 0;
	if (m2a_done == 1) begin
		f[i] <= m2a_rx;
		f[i+1] <= m2a_ry;
		i <= i + 2;
		m2a_reset <= 1;
		if (i == (2**`RandomWalkBranchBits)*4-2)
			state <= state + 1; ///ALl generated
		else 
			state <= 1;
	end
end
3:begin/////PseduoMapping f done. Now Start all Kangaroos and Monitor them
	//First initialise RndGen to sessionSeed
   $display("f initialised");
   $display("Start Kangaroos...");
	rnd_seed <= sessionSeed;
	rnd_reset <= 1;
	//rnd_next <= 0;
	k1_rst <= 1;
	k2_rst <= 1;
	state <= state + 1;
end
4:begin ///Initialse all Kangaroos
	//rnd_next <= 0;
	k1_rst <= 0;
	k1_initDone <= 0;
	k2_rst <= 0;
	k2_initDone <= 0;
	rnd_reset <= 0;
	mem_wren <= 1'b0;
	
	k1_resume <= 0;
	k2_resume <= 0;
	if (k1_done&!k1_resume) begin
		$display("ZOO_K1: %h %h %32h %32h", k1_ra,k1_rb,k1_rx,k1_ry);
		/// Write to Memory
		mem_data <= {k1_ra, k1_rb};
		mem_wren <= 1'b1;
		mem_addr <= mem_addr + 1'b1;
		k1_resume <= 1;
	end else if (k2_done&!k2_resume) begin
		$display("ZOO_K2: %h %h %32h %32h", k2_ra,k2_rb,k2_rx,k2_ry);
		/// Write to Memory
		mem_data <= {k2_ra, k2_rb};
		mem_wren <= 1'b1;
		mem_addr <= mem_addr + 1'b1;
		k2_resume <= 1;
	end	/////////MEMORY WRITE DONE, CHECK Kn Initialisation request. 
	else if (k1_requestInit|k2_requestInit) begin
		m2a_reset <= 0;
		m2a_GFactor <= (`SIMULATION == 1) ? 110'b1 : rnd_data1;
		m2a_PFactor <= (`SIMULATION == 1) ? 110'b0 : rnd_data2;
		$display("Initialisation factor:%h %h", rnd_data1, rnd_data2);
		state <= 6;
	end
end
6:begin //Initialisation 
	//rnd_next <= 1;
	if (k1_requestInit&m2a_done) begin
		k1_initDone <= 1;
		m2a_reset <= 1;
		$display("Initialising Kangaroo 1.");
	end else if (k2_requestInit&m2a_done) begin
		k2_initDone <= 1;
		m2a_reset <= 1;
		$display("Initialising Kangaroo 2.");
	end
	if(k1_initDone|k2_initDone)//Delay initDone signal for 1 clk edge
		state <= 4;
end
7:begin
end
endcase
end

endmodule
*/			 