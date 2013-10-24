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
