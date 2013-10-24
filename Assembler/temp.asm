	8'd0: instr_code <= 8'b00100xxx; //ldo r4, r?			(p_i, ?)
	8'd0: instr_code <= 8'b01100xxx; //tstz r?
	8'd0: instr_code <= 8'b10001010; //jz @p_ni
	8'd0: instr_code <= 8'b00101xxx; //ldo r5, r?			(q_i, ?)
	8'd0: instr_code <= 8'b01100xxx; //tstz r?
	8'd0: instr_code <= 8'b11000110; //jnz @p_i_q_i
	8'd0: instr_code <= 8'b00010xxx; //ldo r2, r?			(q_x, ?)
	8'd0: instr_code <= 8'b01011000; //mov r0
	8'd0: instr_code <= 8'b00011xxx; //ldo r3, r?			(q_y, ?)
	8'd0: instr_code <= 8'b01011001; //mov r1
	8'd0: instr_code <= 8'b11000000; //return true
	8'd0: instr_code <= 8'b10000000; //@p_i_q_i// return false
	8'd0: instr_code <= 8'b00101xxx; //p_ni//ldo r5, r?	(q_i, ?)
	8'd0: instr_code <= 8'b01100xxx; //tstz r?
	8'd0: instr_code <= 8'b10000001; //jz p_ni_q_ni
	8'd0: instr_code <= 8'b11000000; //return true
	8'd0: instr_code <= 8'b00000010; //p_ni_q_ni//ldo r0, r2	(p_x, q_x)
	8'd0: instr_code <= 8'b01000100; //sub r4 				(r4 = p_x - q_x)
	8'd0: instr_code <= 8'b00100xxx; //ldo r4, r? 
	8'd0: instr_code <= 8'b01100xxx; //tstz r?
	8'd0: instr_code <= 8'b10001000; //jz @DOUBLE
	8'd0: instr_code <= 8'b00100xxx; //ldo r4,r?
	8'd0: instr_code <= 8'b01010100; //inv r4 				r4 = (p_x-q_x)^(-1)
	8'd0: instr_code <= 8'b00001011; //ldo r1, r3 			(p_y, q_y)
	8'd0: instr_code <= 8'b01000101; //sub r5 				r5 = p_y - q_y
	8'd0: instr_code <= 8'b00100101; //ldo r4, r5 			[(p_x-q_x)^(-1), p_y - q_y]
	8'd0: instr_code <= 8'b01001100; //mul r4 				r4 = s = (p_y - q_y)/(p_x-q_x)
	8'd0: instr_code <= 8'b11010111; //jnz COMMON
	8'd0: instr_code <= 8'b00001011; //@DOUBLE// ldo r1, r3 	(p_y, q_y)
	8'd0: instr_code <= 8'b01000100; //sub r4 				r4 = p_y - q_y
	8'd0: instr_code <= 8'b00100xxx; //ldo r4, r?
	8'd0: instr_code <= 8'b01100xxx; //tstz r?
	8'd0: instr_code <= 8'b10000010; //jz DOUBLE_1
	8'd0: instr_code <= 8'b10000000; //return false
	8'd0: instr_code <= 8'b00111011; //DOUBLE_1//ldo r7, r3	(0, q_y)
	8'd0: instr_code <= 8'b01000100; //sub r4				r4 = -q_y
	8'd0: instr_code <= 8'b00011100; //ldo r3, r4			(q_y, -q_y)
	8'd0: instr_code <= 8'b01000011; //sub r3				r3 = q_y - (-q_y) = 2y
	8'd0: instr_code <= 8'b01010011; //inv r3				r3 = (2y)^(-1)
	8'd0: instr_code <= 8'b00010010; //ldo r2, r2			(q_x, q_x)
	8'd0: instr_code <= 8'b01001101; //mul r5				r5 =q_x^2
	8'd0: instr_code <= 8'b00111101; //ldo r7, r5			(0, q_x^2)
	8'd0: instr_code <= 8'b01000100; //sub r4				r4 = -x^2
	8'd0: instr_code <= 8'b00101100; //ldo r5, r4			(x^2, -x^2)
	8'd0: instr_code <= 8'b01000101; //sub r5				r5 = 2*x^2
	8'd0: instr_code <= 8'b01000101; //sub r5				r5 = 3*x^2
	8'd0: instr_code <= 8'b00101110; //ldo r5, r6			(3x^2, -A)
	8'd0: instr_code <= 8'b01000100; //sub r4				r4 = 3x^2+A
	8'd0: instr_code <= 8'b00011100; //ldo r3, r4			(2y^-1, 3x^2+A)
	8'd0: instr_code <= 8'b01001100; //mul r4				r4 = s = (3x^2+A)/2y;
	8'd0: instr_code <= 8'b00100100; //COMMON//ldo r4, r4	(s, s)
	8'd0: instr_code <= 8'b01001101; //mul r5				r5 = s*s
	8'd0: instr_code <= 8'b00101000; //ldo r5, r0			s^2, p_x
	8'd0: instr_code <= 8'b01000101; //sub r5				r5 = s^2-px
	8'd0: instr_code <= 8'b00101010; //ldo r5, r2			s^2-px, qx
	8'd0: instr_code <= 8'b01000010; //sub r2				r2 = s^2-px-qx <== X
	8'd0: instr_code <= 8'b00010000; //ldo r2, r0			X, px
	8'd0: instr_code <= 8'b01000011; //sub r3				r3 = X-px
	8'd0: instr_code <= 8'b00011100; //ldo r3, r4			X-px, s
	8'd0: instr_code <= 8'b01001011; //mul r3				r3 = s*(X-px)
	8'd0: instr_code <= 8'b00111001; //ldo r7, r1			0, py 
	8'd0: instr_code <= 8'b01000001; //sub r1				r1 = -py
	8'd0: instr_code <= 8'b00001011; //ldo r1, r3			-py, s*(X-px)
	8'd0: instr_code <= 8'b01000001; //sub r1				r1 = -[py+s*(X-px)] <<Y
	8'd0: instr_code <= 8'b00010xxx; //ldo r2, r?			X, ?
	8'd0: instr_code <= 8'b01011000; //mov r0
	8'd0: instr_code <= 8'b11000000; //return true
