    module hex2leds(
        input [3:0] hexval,
        output [6:0] ledcode
        );

        reg [6:0] lc;
        assign ledcode = ~lc;

        always @(*)
            case (hexval)
		4'h0: lc <= 7'b0111111;
        4'h1: lc <= 7'b0000110;
 		4'h2: lc <= 7'b1011011;
		4'h3: lc <= 7'b1001111;
		4'h4: lc <= 7'b1100110;
		4'h5: lc <= 7'b1101101;
		4'h6: lc <= 7'b1111101;
		4'h7: lc <= 7'b0000111;
		4'h8: lc <= 7'b1111111;
		4'h9: lc <= 7'b1101111;
		4'ha: lc <= 7'b1110111;
		4'hb: lc <= 7'b1111100;
		4'hc: lc <= 7'b0111001;
		4'hd: lc <= 7'b1011110;
		4'he: lc <= 7'b1111001;
		4'hf: lc <= 7'b1110001;
		//...
	    endcase
    endmodule
    