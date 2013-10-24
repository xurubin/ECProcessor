	8'd0: instr_code <= 8'bxxxxxxxx; //tstz r4				pi
	8'd0: instr_code <= 8'bxx001001; //jz @p_ni
	8'd0: instr_code <= 8'bxxxxxxxx; //tstz r5				qi
	8'd0: instr_code <= 8'bxx000110; //jnz @p_i_q_i
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r2, r?			(q_x, ?)
	8'd0: instr_code <= 8'bxxxxxxxx; //mov r0
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r3, r?			(q_y, ?)
	8'd0: instr_code <= 8'bxxxxxxxx; //mov r1
	8'd0: instr_code <= 8'bxxxxxxxx; //return true
	8'd0: instr_code <= 8'bxxxxxxxx; //@p_i_q_i// return false
	8'd0: instr_code <= 8'bxxxxxxxx; //p_ni//tstz r5		(q_i, ?)
	8'd0: instr_code <= 8'bxx000010; //jz p_ni_q_ni
	8'd0: instr_code <= 8'bxxxxxxxx; //return true
	8'd0: instr_code <= 8'bxxxxxxxx; //p_ni_q_ni//ldo r0, r2	(p_x, q_x)
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r4 				(r4 = p_x - q_x)
	8'd0: instr_code <= 8'bxxxxxxxx; //tstz r4				p_x-q_x
	8'd0: instr_code <= 8'bxx001000; //jz @DOUBLE
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r4,r?
	8'd0: instr_code <= 8'bxxxxxxxx; //inv r4 				r4 = (p_x-q_x)^(-1)
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r1, r3 			(p_y, q_y)
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r5 				r5 = p_y - q_y
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r4, r5 			[(p_x-q_x)^(-1), p_y - q_y]
	8'd0: instr_code <= 8'bxxxxxxxx; //mul r4 				r4 = s = (p_y - q_y)/(p_x-q_x)
	8'd0: instr_code <= 8'bxx010110; //jnz COMMON
	8'd0: instr_code <= 8'bxxxxxxxx; //@DOUBLE// ldo r1, r3 	(p_y, q_y)
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r4 				r4 = p_y - q_y
	8'd0: instr_code <= 8'bxxxxxxxx; //tstz r4				p_y - q_y
	8'd0: instr_code <= 8'bxx000010; //jz DOUBLE_1
	8'd0: instr_code <= 8'bxxxxxxxx; //return false
	8'd0: instr_code <= 8'bxxxxxxxx; //DOUBLE_1//ldo r7, r3	(0, q_y)
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r4				r4 = -q_y
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r3, r4			(q_y, -q_y)
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r3				r3 = q_y - (-q_y) = 2y
	8'd0: instr_code <= 8'bxxxxxxxx; //inv r3				r3 = (2y)^(-1)
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r2, r2			(q_x, q_x)
	8'd0: instr_code <= 8'bxxxxxxxx; //mul r5				r5 =q_x^2
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r7, r5			(0, q_x^2)
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r4				r4 = -x^2
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r5, r4			(x^2, -x^2)
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r5				r5 = 2*x^2
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r5				r5 = 3*x^2
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r5, r6			(3x^2, -A)
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r4				r4 = 3x^2+A
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r3, r4			(2y^-1, 3x^2+A)
	8'd0: instr_code <= 8'bxxxxxxxx; //mul r4				r4 = s = (3x^2+A)/2y;
	8'd0: instr_code <= 8'bxxxxxxxx; //COMMON//ldo r4, r4	(s, s)
	8'd0: instr_code <= 8'bxxxxxxxx; //mul r5				r5 = s*s
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r5, r0			s^2, p_x
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r5				r5 = s^2-px
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r5, r2			s^2-px, qx
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r2				r2 = s^2-px-qx <== X
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r2, r0			X, px
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r3				r3 = X-px
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r3, r4			X-px, s
	8'd0: instr_code <= 8'bxxxxxxxx; //mul r3				r3 = s*(X-px)
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r7, r1			0, py 
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r1				r1 = -py
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r1, r3			-py, s*(X-px)
	8'd0: instr_code <= 8'bxxxxxxxx; //sub r1				r1 = -[py+s*(X-px)] <<Y
	8'd0: instr_code <= 8'bxxxxxxxx; //ldo r2, r?			X, ?
	8'd0: instr_code <= 8'bxxxxxxxx; //mov r0
	8'd0: instr_code <= 8'bxxxxxxxx; //return true
