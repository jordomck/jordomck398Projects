3
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | collision detection

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

// KE: merge collision test into FK
// KE: make FK for a configuration and independent of current robot state

kineval.robotIsCollision = function robot_iscollision() {
    // test whether geometry of current configuration of robot is in collision with planning world 

    // form configuration from base location and joint angles
    var q_robot_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_robot_config.length;
        q_robot_config = q_robot_config.concat(robot.joints[x].angle);
    }

    // test for collision and change base color based on the result
    collision_result = kineval.poseIsCollision(q_robot_config);

    robot.collision = collision_result;
}


kineval.poseIsCollision = function robot_collision_test(q) {
    // perform collision test of robot geometry against planning world 

    // test base origin (not extents) against world boundary extents
    if ((q[0]<robot_boundary[0][0])||(q[0]>robot_boundary[1][0])||(q[2]<robot_boundary[0][2])||(q[2]>robot_boundary[1][2]))
        return robot.base;

    // traverse robot kinematics to test each body for collision
    // STENCIL: implement forward kinematics for collision detection
    return robot_collision_forward_kinematics(q); //this function returns the colliding link.

}

function robot_collision_forward_kinematics(q){
	//we need to redo all of the forward kinematics, since this is for hypothetical poses too...
	var myXform = matrix_multiply(generate_translation_matrix(q[0],q[1],q[2]),
		matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(q[5]),
		generate_rotation_matrix_Y(q[4])),generate_rotation_matrix_X(q[3])));
	//now we can recurse upwards
	return traverse_collision_forward_kinematics_link(robot.links[robot.base], myXform, q);
}

function traverse_collision_forward_kinematics_joint(joint, xform, q){

	// first we grab the position of the joint in comparison to its parent link. this formula is (TRANSLATION((ZY)X))
	var jointPosInLinkFrame = matrix_multiply(generate_translation_matrix(joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2]),
	matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(joint.origin.rpy[2]),generate_rotation_matrix_Y(joint.origin.rpy[1])),generate_rotation_matrix_X(joint.origin.rpy[0])));
	//*phew*
	
	// push local transform at top of matrix stack ** THIS PART IS ADAPTING FOR THE LENGTH OF THE PARENT LINK
	var xformLocalTop = matrix_multiply(xform, jointPosInLinkFrame);
	//At this point, the joint should be at its proper origin. However, the joint still needs to rotate or do its prismatic thing.
	//we can copy this code in from the original implementation of FK
	var localXform = generate_identity();
	if(typeof joint.type === 'undefined'){
		//first we go grab a quaternion from the kineval functionality
		var theQuaternionVersion = kineval.quaternion_normalize(kineval.quaternion_from_axisangle([joint.axis[0],joint.axis[1],joint.axis[2]],joint.angle));
		//now we use it to make a rotation matrix!
        localXform = matrix_copy(kineval.quaternion_to_rotation_matrix(theQuaternionVersion));
	}else if(joint.type === 'revolute' || joint.type === 'continuous'){
		//first we go grab a quaternion from the kineval functionality
		theQuaternionVersion = kineval.quaternion_normalize(kineval.quaternion_from_axisangle([joint.axis[0],joint.axis[1],joint.axis[2]],joint.angle));
		//now we use it to make a rotation matrix!
        localXform = matrix_copy(kineval.quaternion_to_rotation_matrix(theQuaternionVersion));
		
	}else if(joint.type === 'prismatic'){
		localXform = matrix_copy(generate_translation_matrix(
			joint.axis[0]*joint.angle,
			joint.axis[1]*joint.angle,
			joint.axis[2]*joint.angle
			));
	} 


	
	//push local transform of JOINT at top of matrix stack ** COULD BE PRISMATIC OR REVOLUTE, IDC
	var xformDone = matrix_multiply(xformLocalTop, localXform);
	
	return traverse_collision_forward_kinematics_link(robot.links[joint.child], xformDone, q);
}


function traverse_collision_forward_kinematics_link(link,mstack,q) {

    // test collision by transforming obstacles in world to link space

    mstack_inv = numeric.inv(mstack);

    var i;
    var j;

    // test each obstacle against link bbox geometry by transforming obstacle into link frame and testing against axis aligned bounding box
    //for (j=0;j<robot_obstacles.length;j++) { 
    for (j in robot_obstacles) { 

        var obstacle_local = matrix_multiply(mstack_inv,robot_obstacles[j].location);

        // assume link is in collision as default
        var in_collision = true; 

        // if obstacle lies outside the link extents along any dimension, no collision is detected
        if (
            (obstacle_local[0][0]<(link.bbox.min.x-robot_obstacles[j].radius))
            ||
            (obstacle_local[0][0]>(link.bbox.max.x+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[1][0]<(link.bbox.min.y-robot_obstacles[j].radius))
            ||
            (obstacle_local[1][0]>(link.bbox.max.y+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[2][0]<(link.bbox.min.z-robot_obstacles[j].radius)) 
            ||
            (obstacle_local[2][0]>(link.bbox.max.z+robot_obstacles[j].radius))
        )
                in_collision = false;

        // if obstacle lies within link extents along all dimensions, a collision is detected and return true
        if (in_collision)
            return link.name;
    }

    // recurse child joints for collisions, returning true if child returns collision
    if (typeof link.children !== 'undefined') { // return if there are no children
        var local_collision;
        //for (i=0;i<link.children.length;i++) {
        for (i in link.children) {
            local_collision = traverse_collision_forward_kinematics_joint(robot.joints[link.children[i]],mstack,q)
            if (local_collision)
                return local_collision;
        }
    }

    // return false, when no collision detected for this link and children 
    return false;
}



