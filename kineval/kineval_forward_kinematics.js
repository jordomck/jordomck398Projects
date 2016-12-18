
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms(); DONE (with just this line)
	kineval.buildFKTransforms(); //just call the function

}

kineval.buildFKTransforms = function buildFKTransforms() {
	//put the base in world coordinates
	//first multiply the rotations of Z * Y
	var z = matrix_copy(generate_rotation_matrix_Z(robot.origin.rpy[2]));
	zz = matrix_multiply(z,z);
	var y = generate_rotation_matrix_Y(robot.origin.rpy[1])
	var zy = matrix_multiply(z, y);
	//now multiply this matrix by rotx
	var zyx = matrix_multiply(zy, generate_rotation_matrix_X(robot.origin.rpy[0]));
	//printMatrix(zyx);
	//finally combine with translation
	robot.origin.xform = matrix_multiply(generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1],robot.origin.xyz[2]), zyx);
	
	//compute heading
	forward_heading_local = [[0], [0], [1], [1]]; //these are in double brackets so that it is the transformed, 2d version, ready for multiplication. This = forward on z only. Sounds good.
	//set heading world
	heading_world = matrix_multiply(robot.origin.xform, forward_heading_local)
	robot_heading = heading_world;
	console.log(robot_heading[0][0] + " " +  robot_heading[1][0] + " " + robot_heading[2][0] + " " +  robot_heading[3][0]);
	//**robot_heading = heading_world
	
	//compute strafe headings with the lateral in the same manner
	lateral_heading_local = [[1],[0],[0],[1]];
    latToWorld = matrix_multiply(robot.origin.xform,lateral_heading_local);
    robot_lateral = latToWorld;
	
 if (robot.links_geom_imported) {
        var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
        kineval.traverseFKLink(robot.links[robot.base], matrix_multiply(robot.origin.xform,offset_xform));
    }
    else {
		//printMatrix(robot.origin.xform);
        kineval.traverseFKLink(robot.origin.xform, robot.links[robot.base]);
    }
	//the format is STARTED FROM, and the actual link.
	//this should call itself on all its joint children.
	
	
}


kineval.traverseFKLink = function traverseFKLink(beginningLoc, myLink){
	
	var copiedBeginning = matrix_copy(beginningLoc); //make sure we're not overwriting anything...
	
	//we need to visit EVERY child joint of the link, so we have to use recursion.
	//first check if there are any children at all...
	myLink.xform = matrix_copy(copiedBeginning);
	if(typeof myLink.children === 'undefined'){
		//console.log("reached the end of the hierarchy.");
		return;
	} 
	if(typeof myLink.children !== 'undefined' && myLink.children.length > 0){
		
		 //links are at the same transform as their parent joint...
		for(var uniqua = 0; uniqua < myLink.children.length; uniqua++){
			//console.log("traversing child number " + (uniqua+1) + " out of " + myLink.children.length);
			kineval.traverseFKJoint(copiedBeginning, robot.joints[myLink.children[uniqua]]);
			//console.log("done traversing the " + (uniqua+1) + "st joint of " + myLink.name);
		}
	}
	
}

kineval.traverseFKJoint = function traverseFKJoint(beginningLoc, incjoint){
	var myJoint = incjoint;
	//console.log("moving through a joint");
	var copiedBeginning = matrix_copy(beginningLoc);
	var jointTransformZy = matrix_multiply(generate_rotation_matrix_Z(myJoint.origin.rpy[2]), generate_rotation_matrix_Y(myJoint.origin.rpy[1]));
	var jointTransformZyx = matrix_multiply(jointTransformZy, generate_rotation_matrix_X(myJoint.origin.rpy[0]));
	var jointTransformInGeneral = generate_translation_matrix(myJoint.origin.xyz[0], myJoint.origin.xyz[1], myJoint.origin.xyz[2]);
	var localTransform = matrix_multiply(jointTransformInGeneral, jointTransformZyx);

	var localXform = generate_identity();
	if(typeof myJoint.type === 'undefined'){
		//first we go grab a quaternion from the kineval functionality
		//console.log(myJoint.angle);
		//console.log(myJoint.axis);
		var theQuaternionVersion = kineval.quaternion_normalize(kineval.quaternion_from_axisangle([myJoint.axis[0],myJoint.axis[1],myJoint.axis[2]],myJoint.angle));
		//now we use it to make a rotation matrix!
        localXform = matrix_copy(kineval.quaternion_to_rotation_matrix(theQuaternionVersion));
		//printMatrix(localXform);
	}else if(myJoint.type === 'revolute' || myJoint.type === 'continuous'){
		console.log("REVOLVING JOINT FOUND!");
		//first we go grab a quaternion from the kineval functionality
		theQuaternionVersion = kineval.quaternion_normalize(kineval.quaternion_from_axisangle([myJoint.axis[0],myJoint.axis[1],myJoint.axis[2]],myJoint.angle));
		//now we use it to make a rotation matrix!
        localXform = matrix_copy(kineval.quaternion_to_rotation_matrix(theQuaternionVersion));
		
	}else if(myJoint.type === 'prismatic'){
		localXform = matrix_copy(generate_translation_matrix(
			myJoint.axis[0]*myJoint.angle,
			myJoint.axis[1]*myJoint.angle,
			myJoint.axis[2]*myJoint.angle
			));
	} 
	
	else {
		localXform = generate_identity();
	}
	var xformcalcpart1 = matrix_multiply(copiedBeginning, localTransform);
	myJoint.xform = matrix_copy(matrix_multiply(xformcalcpart1, localXform)); 
	//printMatrix(myJoint.xform);
	//now we have to get all recursive with it...
	//console.log("about to move through a link called " + robot.links[myJoint.child].name);
	kineval.traverseFKLink(matrix_copy(myJoint.xform), robot.links[myJoint.child])
	//console.log("done traversing a link called " + robot.links[myJoint.child].name);
	
}
    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    //   if (robot.links_geom_imported) {
    //       var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));

