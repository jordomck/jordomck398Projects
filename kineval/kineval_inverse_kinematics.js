
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {
	//console.log("invoked IK function!");
    // compute joint angle controls to move location on specified link to Cartesian location
	
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime() - kineval.params.trial_ik_random.start.getTime();
	// get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
            Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
            + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
            + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.04) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
    // STENCIL: see instructor for random time trial code
	
}

kineval.invertArray = function invertArray(incoming){
	var out = [];
	var j = 0;
	for(var i = incoming.length - 1; i >= 0; i--){
		out[j] = incoming[i];
		j++;
	}
	return out;
}


kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

	//copy world target just to be safe
	var targetCartesian = [[endeffector_target_world.position[0][0]],[endeffector_target_world.position[1][0]],[endeffector_target_world.position[2][0]]];
	printMatrix(targetCartesian);
    // STENCIL: implement inverse kinematics iteration
	//first we need to build a new backwards array for IK
	var currJoint = endeffector_joint;
	var kinematicChain = [];
	var keeper = false;
	
	while(!keeper){
		kinematicChain.push(currJoint);
		if(robot.joints[currJoint].parent !== robot.base){
			currJoint = robot.links[robot.joints[currJoint].parent].parent; //must do double parent because there's a link between
		} else keeper = true;
	}
	
	var chain = kineval.invertArray(kinematicChain);
	
	var endeffectorCartesian = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
	// init jacobian holder matrix. THREE cols because we only care about position, not rotation.
	var jacob = [[],[],[]]; 
	
	for(var i = 0; i < chain.length; i++){
		//console.log("WE'RE ON THE CHAIN LOOP");
		
		// jacobian time!
		// STEP 1: transform current joint origin into world frame by stripping the rotational data from the xform.
		var jointBaseInWorldFrame = matrix_multiply(robot.joints[chain[i]].xform,[[0],[0],[0],[1]]);  // at joint i
		// STEP 2: current joint axis in joint frame
		var axisInJointFrame = [
            [ robot.joints[chain[i]].axis[0] ], //x of axis
            [ robot.joints[chain[i]].axis[1] ], //y of axis
            [ robot.joints[chain[i]].axis[2] ], //z of axis
            [ 0 ]
        ];
		// STEP 3: rotate joint axis in world frame, remains centered on joint origin, becomes K in the diagram?
		var diagramK = matrix_multiply(robot.joints[chain[i]].xform,axisInJointFrame); //this is the vector out of the joint towards the camera
		// STEP 4: compute vector from joint origin to endeffector location, becomes R in the diagram
		var diagramR = [
            [ endeffector_world[0][0] - jointBaseInWorldFrame[0][0] ],
            [ endeffector_world[1][0] - jointBaseInWorldFrame[1][0] ],
            [ endeffector_world[2][0] - jointBaseInWorldFrame[2][0] ]
            ]; //this is the 'pseudoarm' all the way to the end effector
		// STEP 5: compute cross product for linear component of Jacobian column
		var linearComponent = fancy_vector_cross(diagramR, diagramK);
		// STEP 6: set jacobian vals for this joint Jacobian rows: Cartesian dofs  cols: joints
		//ex: jacob[0][i] = TRANSLATIONAL_X
		jacob[0][i] = linearComponent[0];
		jacob[1][i] = linearComponent[1];
		jacob[2][i] = linearComponent[2];
		// STEP 7: if the joint was prismatic, set the first three values to the joint axis (cause that contains movement, not an axis. this is an overwrite of what would otherwise be inaccurate fillings)
		    if (typeof robot.joints[chain[i]].type !== 'undefined') {
				if ((robot.joints[chain[i]].type === 'prismatic')) {
					jacob[0][i] = diagramK[0][0]; // linear: joint axis
					jacob[1][i] = diagramK[1][0]; 
					jacob[2][i] = diagramK[2][0]; 
				}
			}
	}

	
	// get delta vector from current to target endeffector pose **
	var delta = [
            [ targetCartesian[0][0] - endeffectorCartesian[0][0] ],
            [ targetCartesian[1][0] - endeffectorCartesian[1][0] ],
            [ targetCartesian[2][0] - endeffectorCartesian[2][0] ]
        ]; 
	if(!kineval.params.ik_pseudoinverse)
		finalCommands = matrix_multiply(matrix_transpose(jacob),delta); //one is 3x3, other is 3x1
	else {

		finalCommands = matrix_multiply(matrix_pseudoinverse(jacob), delta);
		
	}
		

	
	// update controls by add dq to joint angles **
	desiredDistance = kineval.params.ik_steplength;
	
	    for (i = 0;i < chain.length; i++) {
			robot.joints[chain[i]].control -= desiredDistance*finalCommands[i];
		}
}



