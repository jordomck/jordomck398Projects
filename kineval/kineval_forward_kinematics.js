
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
	z = matrix_copy(generate_rotation_matrix_Z(robot.origin.rpy[2]));
	zy = matrix_multiply(generate_rotation_matrix_Z(robot.origin.rpy[2]), generate_rotation_matrix_Y(robot.origin.rpy[1]));
	//now multiply this matrix by rotx
	zyx = matrix_multiply(zy, generate_rotation_matrix_X(robot.origin.rpy[0]));
	//finally combine with translation
	robot.origin.xform = matrix_multiply(zyx, generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1],robot.origin.xyz[2]));
	
	//compute heading
	forward_heading_local = [[0], [0], [1], [1]]; //these are in double brackets so that it is the transformed, 2d version, ready for multiplication. This = forward on z only. Sounds good.
	//set heading world
	headingToWorld = matrix_multiply(robot.origin.xform, forward_heading_local)
	robot_heading = headingToWorld;
	//**robot_heading = heading_world
	
	//compute strafe headings with the lateral in the same manner
	lateral_heading_local = [[1],[0],[0],[1]];
    latToWorld = matrix_multiply(robot.origin.xform,lateral_heading_local);
    robot_lateral = latToWorld;
	
	//no coordinate conversion will be needed in this case, since I don't plan on using the ROS coordinates for Fetch
	
	//initiate the link travel! we're going to start by traversing the base link. This should set off a chain reaction of xform calcs.
	kineval.traverseFKLink(robot.origin.xform, robot.links[robot.base]);
	//the format is STARTED FROM, and the actual link.
	//this should call itself on all its joint children.
	
	
}


kineval.traverseFKLink = function traverseFKLink(beginningLoc, myLink){
	console.log("moving through a link named " + myLink.name);
	copiedBeginning = matrix_copy(beginningLoc); //make sure we're not overwriting anything...
	
	//we need to visit EVERY child joint of the link, so we have to use recursion.
	//first check if there are any children at all...
	myLink.xform = copiedBeginning;
	if(typeof myLink.children !== 'undefined'){
		
		console.log("children data type is " + typeof myLink.children);
		 //links are at the same transform as their parent joint...
		for(i = 0; i < myLink.children.length; i++){
			kineval.traverseFKJoint(copiedBeginning, robot.joints[myLink.children[i]]);
		}
	}
	
}

kineval.traverseFKJoint = function traverseFKJoint(beginningLoc, myJoint){
	console.log("moving through a joint");
	copiedBeginning = matrix_copy(beginningLoc);
	jointTransformZy = matrix_multiply(generate_rotation_matrix_Z(myJoint.origin.rpy[2]), generate_rotation_matrix_Y(myJoint.origin.rpy[1]));
	jointTransformInGeneral = generate_translation_matrix(myJoint.origin.xyz[0], myJoint.origin.xyz[1], myJoint.origin.xyz[2]);
	localTransform = matrix_multiply(jointTransformZy, jointTransformInGeneral);

	localXform = generate_identity();
	
	if(typeof myJoint.type === 'undefined'){
		//first we go grab a quaternion from the kineval functionality
		theQuaternionVersion = kineval.quaternion_normalize(kineval.quaternion_from_axisangle([myJoint.axis[0],myJoint.axis[1],myJoint.axis[2]],myJoint.angle));
		//now we use it to make a rotation matrix!
        localXform = kineval.quaternion_to_rotation_matrix(theQuaternionVersion);
	}else if(myJoint.type == 'prismatic'){
		localXform = generate_translation_matrix(
			myJoint.axis[0]*myJoint.angle,
			myJoint.axis[1]*myJoint.angle,
			myJoint.axis[2]*myJoint.angle
			)
	} 
	else if(myJoint.type == 'revolute' || myJoint.type == 'continuous'){
		//first we go grab a quaternion from the kineval functionality
		theQuaternionVersion = kineval.quaternion_normalize(kineval.quaternion_from_axisangle([joint.axis[0],joint.axis[1],joint.axis[2]],joint.angle));
		//now we use it to make a rotation matrix!
        localXform = kineval.quaternion_to_rotation_matrix(theQuaternionVersion);
		
	}
	else {
		localXform = generate_identity();
	}
	xformcalcpart1 = matrix_multiply(beginningLoc, localXform);
	myJoint.xform = matrix_multiply(xformcalcpart1, localTransform); 
	//now we have to get all recursive with it...
	kineval.traverseFKLink(matrix_copy(myJoint.xform), robot.links[myJoint.child])
	
	
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

