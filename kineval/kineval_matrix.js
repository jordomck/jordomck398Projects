//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
function matrix_multiply(m1, m2){
	var mat = [];
	var i, j, k;
	//printMatrix(m1); //this is making it in just fine
	//printMatrix(m2); //also fine
	if(m2.length != m1[0].length) {
		//Console.log("THIS CANNOT MULTIPLY BECAUSE OF DIMENSION MISMATCH");
		return 0;	
	}
	for(i=0; i < m1.length; i++){
		//we must generate a new submatrix of the output to place results in
		mat[i] = [];
		for(j=0; j < m1.length; j++) { //for COLUMN of m2
		
		//now we take a dot product
			mat[i].push(0);
			
			for(k = 0; k < m1[0].length; k++) {
				//console.log(m1[i][k]);				
				mat[i][j] += m1[i][k] * m2[k][j];	
			}

		}		
	}
	//console.log(mat.length +  mat[0].length);
	////console.log("matrix multiplied and ready to print!");
	//printMatrix(mat);
	return mat;
}
    //   matrix_transpose
function matrix_transpose(m1){
	var mat = [];
	var i, j;
	for(i = 0; i < m1.length; i++){
		mat[i] = [];
		for(j = 0; j < m1[0].length; j++){
			mat[i][j] = m1[j][i];
		}
	}
	return mat;
}
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
function vector_normalize(v){
	output = [];
	totalSqrMag = 0;
	for(i = 0; i < v.length; i++){
	totalSqrMag += v[i]*v[i];
	}
	for(i = 0; i < v.length; i++){
		output[i] = v[i] / Math.sqrt(totalSqrMag); 
	}
	return output;
}
    //   vector_cross
function vector_cross(v1, v2){
	output = [];
	output[0] = v1[1]*v2[2] - v1[2]*v2[1];
	output[2] = v1[1]*v2[0] - v1[1]*v2[0];
	output[1] = v1[2]*v2[0] - v1[0]*v2[2];
	
	return output;
}

function printMatrix(m){
	console.log(m[0][0] + " " + m[0][1] + " " + m[0][2] + " " + m[0][3]);
	console.log(m[1][0] + " " + m[1][1] + " " + m[1][2] + " " + m[1][3]);
	console.log(m[2][0] + " " + m[2][1] + " " + m[2][2] + " " + m[2][3]);
	console.log(m[3][0] + " " + m[3][1] + " " + m[3][2] + " " + m[3][3]);
}
    //   generate_identity
function generate_identity(){
	//this is a 4d matrix
	return [[1, 0, 0, 0],
			[0, 1, 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]];
}
    //   generate_translation_matrix
function generate_translation_matrix(xpos, ypos, zpos){
	var mat = [
	[1, 0, 0, xpos],
	[0, 1, 0, ypos],
	[0, 0, 1, zpos],
	[0, 0, 0, 1]
	];
	return mat;
}
    //   generate_rotation_matrix_X
function generate_rotation_matrix_X(xangle){
	var mat = [
	[1, 0, 0, 0], //this row is unmodified because it's the x row.
	[0, Math.cos(xangle), -Math.sin(xangle),0],
	[0, Math.sin(xangle), Math.cos(xangle),0],
	[0, 0, 0, 1],
	];
	return mat;
	
}
    //   generate_rotation_matrix_Y
function generate_rotation_matrix_Y(yangle){
	var mat = [
		[Math.cos(yangle), 0, Math.sin(yangle), 0],
		[0, 1, 0, 0],
		[0, -Math.sin(yangle), Math.cos(yangle), 0],
		[0, 0, 0, 1]
	];
	return mat;
}
    //   generate_rotation_matrix_Z
function generate_rotation_matrix_Z(zangle){
	var mat = [
		[Math.cos(zangle), -Math.sin(zangle), 0, 0],
		[Math.sin(zangle), Math.cos(zangle), 0, 0],
		[0,0,1,0],
		[0,0,0,1]
	 ];
	return mat;
}

