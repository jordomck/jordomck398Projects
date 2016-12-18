
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | update robot state from controls

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.applyControls = function robot_apply_controls() {
    // apply robot controls to robot kinematics transforms and joint angles, then zero controls
    // includes update of camera position based on base movement

    // update robot configuration from controls
    for (x in robot.joints) {
        if (isNaN(robot.joints[x].control))
            console.warn("kineval: control value for " + x +" is a nan"); //+robot.joints[x].control);
        // update joint angles
        robot.joints[x].angle += robot.joints[x].control; 
		//console.log(robot.joints[x].control);
    // STENCIL: enforce joint limits for prismatic and revolute joints
	//joints have their limits as marked in their field .limits.lower and .limits.upper
	//they apply regardless of whether it's revolute or prismatic on the 'angle' attribute.
	
		//double-check that the joint does exist, so we don't read fields from an empty object
		if(typeof robot.joints[x] !== 'undefined'){
			//now we break if the joint was continuous, since continuous joints don't have rotation limits
			if(typeof robot.joints[x] === 'revolute' || typeof robot.joints[x] === 'prismatic'){
				//now limit!
				if(robot.joints[x].angle < robot.joints[x].limit.lower){ //constrain lower bound
					robot.joints[x].angle = robot.joints[x].limit.lower;
				}
				else if(robot.joints[x].angle > robot.joints[x].limit.upper){ //constrain upper bound
					robot.joints[x].angle = robot.joints[x].limit.upper;
				}

			}
		}


        // clear controls back to zero for next timestep
        robot.joints[x].control = 0;
    }

    // base motion
    robot.origin.xyz[0] += robot.control.xyz[0];
    robot.origin.xyz[1] += robot.control.xyz[1];
    robot.origin.xyz[2] += robot.control.xyz[2];
    robot.origin.rpy[0] += robot.control.rpy[0];
    robot.origin.rpy[1] += robot.control.rpy[1];
    robot.origin.rpy[2] += robot.control.rpy[2];

    // move camera with robot base
    camera_controls.object.position.x += robot.control.xyz[0];
    camera_controls.object.position.y += robot.control.xyz[1];
    camera_controls.object.position.z += robot.control.xyz[2];

    // zero controls now that they have been applied to robot
    robot.control = {xyz: [0,0,0], rpy:[0,0,0]}; 
}

