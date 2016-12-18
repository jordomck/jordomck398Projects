
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/


kineval.setpointDanceSequence = function execute_setpoints() {
	
    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return;
    // STENCIL: implement FSM to cycle through dance pose setpoints
	var curdate = new Date();
	if(typeof storedSeconds === 'undefined')
		storedSeconds = curdate.getSeconds();
	if(typeof poseToAssume === 'undefined')
		poseToAssume = 0;
	if(curdate.getSeconds() > storedSeconds)
	{
		console.log("pose: " + poseToAssume);
		storedSeconds = curdate.getSeconds();
		
		//choose a pose!
		poseToAssume = poseToAssume + 1;
		poseToAssume = poseToAssume % 3;
		
		//order up the pose!
		if(poseToAssume == 0){
		//first pose: all joints to neutral
			for(joint in robot.joints){
				var beginning = -1.6
				kineval.params.setpoint_target[joint] = beginning;
				beginning = beginning * 1.5;
			}
		} else if (poseToAssume == 1) 	{
			var beginning = 1.6;
			for(joint in robot.joints){
				kineval.params.setpoint_target[joint] = beginning;
				beginning = beginning * .7; 
			}
		} else if (poseToAssume == 2)	{
			for(joint in robot.joints){
				kineval.params.setpoint_target[joint] = 1;
			}
		}
	}
	
}



kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
	var myJoint;
	for(myJoint in robot.joints){
		robot.joints[myJoint].servo.p_desired = kineval.params.setpoint_target[myJoint];
		//we may need to add or subtract 2*PI from this value in order to make it not freak outerHTML
		//SCENARIO 1: desired is really far in the negative direction and we need to add 2PI
		if(robot.joints[myJoint].angle > 1.6 * Math.PI && robot.joints[myJoint].servo.p_desired < .4*Math.PI){
			robot.joints[myJoint].servo.p_desired += 2*Math.PI; //this causes the robot to push forward through the loop. once it passes zero, it can reach its destination the traditional way.
		}
		//SCENARIO 2: desired is really far in the positive direction and we could just subtract 2PI to get there faster
		if(robot.joints[myJoint].angle < .4 * Math.PI && robot.joints[myJoint].servo.p_desired > 1.6*Math.PI){
			robot.joints[myJoint].servo.p_desired += 2*Math.PI; //this causes the robot to push backwards through the loop. once it passes zero, it can reach its destination the traditional way.
		}
		robot.joints[myJoint].control = (kineval.params.setpoint_target[myJoint] - robot.joints[myJoint].angle) * robot.joints[myJoint].servo.p_gain; //P term!
		//console.log(robot.joints[myJoint].servo.control);
	}
}


