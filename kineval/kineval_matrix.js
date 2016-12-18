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
		console.log("THIS CANNOT MULTIPLY BECAUSE OF DIMENSION MISMATCH");
		return 0;	
	}
	for(i=0; i < m1.length; i++){
		//we must generate a new submatrix of the output to place results in
		mat[i] = [];
		for(j=0; j < m2[0].length; j++) { //for COLUMN of m2
		
		//now we take a dot product
			mat[i][j] = 0;
			
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

function fancy_vector_cross(v1, v2){ //v2 is 2d, v1 is 1d.....
	//console.log("The fancy cross product was called");
	output = [];
	output[0] = v1[1]*v2[2][0] - v1[2]*v2[1][0];	
	output[1] = v1[2]*v2[0][0] - v1[0]*v2[2][0];
	output[2] = v1[0]*v2[1][0] - v1[1]*v2[0][0];
	
	return output;
}

    //   matrix_transpose
function matrix_transpose(m1){
	var mat = [];
	var i, j;
	for(i = 0; i < m1[0].length; i++){
		mat[i] = [];
		for(j = 0; j < m1.length; j++){
			mat[i][j] = m1[j][i];
		}
	}
	return mat;
}
    //   matrix_pseudoinverse
function matrix_pseudoinverse(m){

    if (m[0].length == m.length) {  
        return numeric.inv(m);
    }
    else if (m[0].length < m.length) { 
        var transpose = matrix_transpose(m);
        var squareVersion = matrix_multiply(transpose,m);  
        return matrix_multiply(numeric.inv(squareVersion), transpose);
    }
    else { 
        var transpose = matrix_transpose(m);
        var squareVersion = matrix_multiply(m, transpose);  
        return matrix_multiply(transpose, numeric.inv(squareVersion));
    }
}
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
	console.log("The cross product was called");
	output = [];
	output[0] = v1[1]*v2[2] - v1[2]*v2[1];	
	output[1] = v1[2]*v2[0] - v1[0]*v2[2];
	output[2] = v1[0]*v2[1] - v1[1]*v2[0];
	
	return output;
}

function printMatrix(m){
	console.log("printing matrix- outer size is " + m.length + ", inner size is " + m[0].length);
	for(var outer = 0; outer < m.length; outer++){
		myStr= "";
		for(var inner = 0; inner < m[0].length; inner++){
			myStr = myStr + " " + m[outer][inner];
		}
		console.log(myStr);
	}
}

function printVector(v){
	console.log("printing vector:");
	var myStr = "";
	for(var counter = 0; counter < v.length; counter++){
		myStr += " " + v[counter];
	}
	console.log(myStr);
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
		[-Math.sin(yangle), 0, Math.cos(yangle), 0],
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

