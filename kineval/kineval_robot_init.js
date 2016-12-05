/*

     KinEval
     Implementation of robot kinematics, control, decision making, and dynamics 
     in HTML5/JavaScript and threejs
     
     @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

*/

kineval.initRobot = function initRobot() {
        
    // ASSUME: robot kinematics are described separate js file (eg., "robot_urdf_example.js")

    // initialize and create threejs mesh objects for robot links
    kineval.initRobotLinks();

    // initialize robot joints and create threejs mesh objects for robot joints and form kinematic hiearchy
    kineval.initRobotJoints();

    // initialize robot collision state
    robot.collision = false;

}

kineval.initRobotLinks = function initRobotLinks() {

    for (x in robot.links) {
        robot.links[x].name = x;
    }

    // initialize controls for robot base link
    robot.control = {xyz: [0,0,0], rpy:[0,0,0]}; 
}

kineval.initRobotJoints = function initRobotJoints() {
    // build kinematic hierarchy by looping over each joint in the robot
    //   (object fields can be index through array-style indices, object[field] = property)
    //   and insert threejs scene graph (each joint and link are directly connect to scene root)
    // NOTE: kinematic hierarchy is maintained independently by this code, not threejs

    var x,tempmat;

    for (x in robot.joints) {

        // give the joint its name as an id
        robot.joints[x].name = x;

        // initialize joint angle value and control input value
        robot.joints[x].angle = 0;
        robot.joints[x].control = 0;
        robot.joints[x].servo = {};
		// STENCIL: set appropriate servo gains for arm setpoint control
        robot.joints[x].servo.p_gain = .2; 
        robot.joints[x].servo.p_desired = 0;
        robot.joints[x].servo.d_gain = .05; 

    // STENCIL: complete kinematic hierarchy of robot for convenience. DONE
    //   robot description only specifies parent and child links for joints.
    //   additionally specify parent and child joints for each link
	//	 the .child feature returns a link, since the children of joints are always links.
		robot.links[robot.joints[x].child].parent = x;
		//now we need to hook up the link as a child of its parent joint
		//if no children yet, add an empty array
		if(typeof robot.links[robot.joints[x].parent].children === 'undefined'){
			robot.links[robot.joints[x].parent].children = [];
		}
		//now that we can guarantee an array exists, append the new link to the joint's children
		robot.links[robot.joints[x].parent].children.push(x);
		
    }

}





