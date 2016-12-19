
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        //keep things within joint limits here
		if(typeof robot.joints[x].type !== 'undefined'){
			if((robot.joints[x].type === 'revolute' || robot.joints[x].type === 'prismatic')){
				q_start_config.push(normalize_joint_state(robot.joints[x].limit.upper, robot.joints[x].limit.lower, robot.joints[x].angle));
			} else {
				q_start_config.push(robot.joints[x].angle % (2*Math.PI));
			}
		} else {
			q_start_config.push(robot.joints[x].angle % (2*Math.PI));
		}
    }
	
	lookingAtStartTree = true;
    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;
	
	//set up the starting tree
	startTree = tree_init(q_start_config);
	treeA = startTree;
	
	
	//set up the ending tree
	endTree = tree_init(q_goal_config);
	treeB = endTree;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}



function robot_rrt_planner_iterate() {
    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)
	var randomConfig = random_config(treeA.vertices[0].vertex.length);
	//console.log(startTree);
    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();
		var extendStatus = rrt_extend(randomConfig, treeA, .5);
		console.log(extendStatus);
		if(extendStatus != "collided"){
			var connectStatus = rrt_connect(treeB, treeA.vertices[treeA.newest].vertex);
			if(connectStatus == "connected"){
				console.log("we made it, in theory!");
				rrt_iterate = false; //ALL DONE!
				pathFromA = find_path(treeA, treeA.vertices[0], treeA.vertices[treeA.newest]);
				pathFromB = find_path(treeB, treeB.vertices[0], treeB.vertices[treeB.newest]);
				
				//now we need to set up the array of states required to get to that destination (for playback)
				if(lookingAtStartTree){ //treeA is startTree
					startTree = treeA;
					endTree = treeB;
					pathFromStart = pathFromA;
					pathFromEnd = pathFromB;
				}
				else {
					startTree = treeB;
					endTree = treeA;
					pathFromStart = pathFromB;
					pathFromEnd = pathFromA;
				}
				
				var idx = 0;
				
                for (i=0; i < pathFromEnd.length; i++) {
                        kineval.motion_plan[idx] = pathFromEnd[i];
                        idx += 1;
                }
				for (i = pathFromStart.length-1; i >= 0; i--) {
							console.log(pathFromStart[i]);
                            kineval.motion_plan[idx] = pathFromStart[i];
                            idx += 1;
				}
				
				
				
				return "reached";
			}
			
		}
		//swap trees!
		var temp = treeA;
		treeA = treeB;
		treeB = temp;
		lookingAtStartTree = !lookingAtStartTree;
		
		rrt_iter_count++;
		
		
    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
    }

}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {

    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
		//this is the step that reaches from the tree towards our generated point.
	function rrt_extend(config, tree, stepSize){
		var nearest = nearest_neighbor(config, tree);
		var nearestIdx = nearest[1];
		console.log(nearestIdx);
		var nearestVert = nearest[0];
		
		var created = new_config(config, nearestVert, stepSize);
		//now we need to check if the created node is in collision or not
		if(!kineval.poseIsCollision(created)){
			//since it's a valid node, we can add it to the tree.
			//make sure to add edges!
			tree_add_vertex(tree, created);
			tree_add_edge(tree, tree.vertices.length-1, nearestIdx);
			
			//did we make it there?
			distanceToGoal = get_config_distance(created, config);
			//console.log("REMAINING DISTANCE: " + distanceToGoal );
			//console.log(distanceToGoal);

			if(distanceToGoal < stepSize){
				//we made it!
				return "connected";
			} else return "continuing";
		} else return "collided";
	}
	
	

    //   rrt_connect
	//this is the one that basically calls rrt_extend until it gets the go-ahead to be done
	function rrt_connect(tree, config){
		var STEP_MAGNITUDE = .5;
		//init while
		var extendStatus = "continuing";
		var counter = 0;
		while(extendStatus == "continuing"){
			extendStatus = rrt_extend(config, tree, STEP_MAGNITUDE);
			counter++;
		}
		
		return extendStatus;
	}
    //   random_config
	function random_config(howManyDofs){
		SCALE_FACTOR = 900;
		var q = [];
		//we need to go all the way through q, because i can't be bothered to count all the dofs
		for(var i = 0; i < howManyDofs; i++){
			
			if((i == 1) || (i == 3) || (i == 5)){ //don't move out of xz plane. no y movement, also don't rotate about anything but yaxis
				q[i] = 0;
			}
			else if(i == 0){ //randomize an x-position
				var upperLimit = robot_boundary[1][0];
				var lowerLimit = robot_boundary[0][0];
				var mag = upperLimit - lowerLimit;
				var randomValue = Math.random() * mag;
				q[i] = randomValue + lowerLimit;
				
			} else if (i == 2) { //randomize a z-position
				var upperLimit = robot_boundary[1][2];
				
				//console.log("upper limit: " + upperLimit);
				var lowerLimit = robot_boundary[0][2];
				var mag = Math.abs(upperLimit - lowerLimit);
				
				//console.log("lower limit: " + lowerLimit);
				var randomValue = Math.random() * mag;
				q[i] = randomValue + lowerLimit;
				//console.log(q[i]);
			} else if (i == 4) { //randomize base rotation
				q[i] = Math.random() * 2 * Math.PI;
				
			} //we've done all the randomization we need to do for the base. Now we must use the q_index to find out the configuration data for the joint in question
			
			else if (typeof robot.joints[q_index[i]].type === 'undefined'){
				//if it's undefined, we assume it's continuous and we can just assign a random angle to it.
				q[i] = Math.random() * 2 * Math.PI;
			} else {
				//we know the joint has a type. If it has limits, we need to know that and assign within them. If it does not, we can assign arbitrarily
				//revolute and prismatic both have limits
				if(robot.joints[q_index[i]].type === 'revolute' || robot.joints[q_index[i]].type === 'prismatic'){
					//first calculate the acceptable range
					var upperLimit = robot.joints[q_index[i]].limit.upper ;
					var lowerLimit = robot.joints[q_index[i]].limit.lower;
					var magnitudeOfRange = upperLimit - lowerLimit;
					var randomValue = Math.random() * magnitudeOfRange;
					q[i] = lowerLimit + magnitudeOfRange;
				} else {
					q[i] = Math.random() * 2 * Math.PI; //it's continuous and we can just assign whatever we want.
				}
			}
		}
		return q;
	}
    //   new_config
	//this function should linearly interpolate between the goal and the new config. Except not really. either get there, or step one step closer.
	function new_config(goal, beginning, stepMagnitude){
		var diff = []
		var mag = 0;
		//first we have to find the vector between the two configurations
		for(var i = 0; i < beginning.vertex.length; i++){
			diff[i] = goal[i] - beginning.vertex[i];
			mag += diff[i]*diff[i];
		}
		mag = Math.sqrt(mag);
		
		
		var finalVersion = [];
		for(var i = 0; i < beginning.vertex.length; i++){
			finalVersion[i] = beginning.vertex[i] + stepMagnitude * (diff[i]/mag);
		}
		return finalVersion;
		
	}
	// get_config_distance
	function get_config_distance(q1, q2){
		//this is just pythagorean distance in all DOFs
		var distance = 0;
		//console.log(q1);
		//console.log(q2);
		for(var i = 0; i < q2.length; i++){
			distance += (q1[i]-q2[i]) * (q1[i]-q2[i]);
		}
		return Math.sqrt(distance);
	}
    //   nearest_neighbor
	function nearest_neighbor(q, tree){
		var bestDistance = get_config_distance(q, tree.vertices[0].vertex); //initially the best one will be the zeroth index of the tree
		var bestIdx = 0;
		console.log(tree.vertices.length);
		//iterate through entire tree one step at a time
		for(var i = 0; i < tree.vertices.length; i++){
			var currDistance = get_config_distance(tree.vertices[i].vertex, q);
			if(currDistance < bestDistance){
				bestDistance = currDistance;
				bestIdx = i;
			}
		}
		
		//now we'd really like to return both the index and the config, so we'll do it as a tuple
		var returnable = [];
		returnable[0] = tree.vertices[bestIdx];
		returnable[1] = bestIdx;
		return returnable;
	}
    //   normalize_joint_state
	//I'm pretty sure this is meant to be a function that enforces joint boundaries
	function normalize_joint_state(upper, lower, current){
		if(current > upper)
			return upper;
		if(current < lower)
			return lower;
		return current;
	}
    //   find_path
	//this is the function that should call path_dfs, I think.
	function find_path(currentTree, goal, startNode){
		//first reset all 'searched' on all nodes in the tree
		for(var i = 0; i < currentTree.vertices.length; i++){
			currentTree.vertices[i].searched = false;
		}
		
		path = path_dfs(goal, startNode);
		if(path != "failure"){
			//color the path on the mapping
			for(var i = 0; i < path.length; i++){
				//each index of path is a vertex
				path[i].geom.material.color = {r:1, g:0, b:0};
				
			}
			//thank goodness we don't need to force the robot to execute this path!
		}
		return path; //returns "failure" if it didn't work, I believe.
	}
	
    //   path_dfs
	//this is a recursive tree traversal that hopes to find the goal config
	function path_dfs(goalConfig, vertex){
		if(vertex == goalConfig){
			return [vertex]; //this has to be an array so we can push other parts of the path to it...
		}
		vertex.searched = true;
		
		//now we need to search all nonsearched edges adjacent to this vertex
		for(var i = 0; i < vertex.edges.length; i++){
			if(vertex.edges[i].searched) continue; //don't get caught bouncing back and forth across the tree...
			//recursive call goes here
			var result = path_dfs(goalConfig, vertex.edges[i]);
			
			//now we need to work backwards if one of the iteration stacks DOES come up with something
			//if a stack returns anything but "failure" we know we're in good shape and we can add another node to the list
			if(result != "failure"){
				result.push(vertex);
				return result;
			}
		}
		
		return "failure";
	}










