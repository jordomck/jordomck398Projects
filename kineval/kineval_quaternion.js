//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
	kineval.quaternion_from_axisangle = function quaternion_from_axisangle(axis, angle){
		var q = [];
		q.a = Math.cos(angle/2);
		q.b = Math.sin(angle/2) * axis[0];
		q.c = Math.sin(angle/2) * axis[1];
		q.d = Math.sin(angle/2) * axis[2];
		return q;
	}
    //   quaternion_normalize
	kineval.quaternion_normalize = function quaternion_normalize(q){
		magnitude = Math.sqrt(q.a*q.a + q.b*q.b + q.c*q.c + q.d*q.d);
		var qnew = [];
		qnew.a = q.a/magnitude;
		qnew.b = q.b/magnitude;
		qnew.c = q.c/magnitude;
		qnew.d = q.d/magnitude;
		return qnew;
	}
    //   quaternion_to_rotation_matrix
	kineval.quaternion_to_rotation_matrix = function quaternion_to_rotation_matrix(q){
		var mat = [];
		mat[0] = [(q.a*q.a + q.b*q.b - q.c*q.c - q.d*q.d), 2*(q.b*q.c - q.a*q.d), 2*(q.a*q.c + q.b*q.d), 0]; //triple checked
		mat[1] = [2*(q.b*q.c + q.a*q.d), q.a*q.a - q.b*q.b + q.c*q.c - q.d*q.d, 2*(q.c*q.d - q.a*q.b), 0]; //triple checked
		mat[2] = [2*(q.b*q.d - q.a*q.c), 2*(q.a*q.b + q.c*q.d), q.a*q.a - q.b*q.b - q.c*q.c + q.d*q.d, 0];  //double checked
		mat[3] = [0,0,0,1];
		return mat;
	}
	
	
    //   quaternion_multiply
	kineval.quaternion_multiply = function quaternion_multiply(u,v){
		var end = [];
		end.a = u.a*v.a - u.b*v.b - u.c*v.c - u.d*v.d;
		end.b = u.a*v.b + u.b*v.a - u.c*v.d + u.d*v.c;
		end.c = u.a*v.c + u.b*v.d + u.c*v.a - u.d*v.b;
		end.d = u.a*v.d - u.b*v.c + u.c*v.b + u.d*v.a;
		return end;
	}

