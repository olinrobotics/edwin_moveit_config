/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

moveit_msgs::CollisionObject spawnObject(moveit::planning_interface::PlanningSceneInterface& p){
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.is_diff = true;
	planning_scene.robot_state.is_diff = true;

	std::vector<moveit_msgs::CollisionObject> v;

	// add table
	moveit_msgs::CollisionObject table;
	table.header.frame_id = "base_link";
	table.id = "table";
	shape_msgs::SolidPrimitive table_geom;
	table_geom.type = table_geom.BOX;
	table_geom.dimensions.push_back(2.032);
	table_geom.dimensions.push_back(0.6096);
	table_geom.dimensions.push_back(0.05);
	geometry_msgs::Pose table_pose;
	table_pose.orientation.w = 1;
	table_pose.position.x = 0.55;
	table_pose.position.y = 0;
	table_pose.position.z = -0.05;
	table.primitives.push_back(table_geom);
	table.primitive_poses.push_back(table_pose);
	table.operation = table.ADD;
	v.push_back(table);

	moveit_msgs::CollisionObject object;

	// add "OBJECT"
	object.header.frame_id = "odom";
	object.id = "object";

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.CYLINDER;
	primitive.dimensions.push_back(0.2);
	primitive.dimensions.push_back(0.04);

	geometry_msgs::Pose pose;
	pose.orientation.w = 1;
	pose.position.x = 0.3; // 30 cm forwards from frame
	pose.position.y = 0.0;
	pose.position.z = primitive.dimensions[0]/2 + 0.0;

	object.primitives.push_back(primitive);
	object.primitive_poses.push_back(pose);

	// add object to scene
	object.operation = object.ADD;
	v.push_back(object);



	p.addCollisionObjects(v);
	return object;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "edwin_move_group_interface");
	ros::NodeHandle node_handle;  
	ros::AsyncSpinner spinner(1);
	spinner.start();


	/* This sleep is ONLY to allow Rviz to come up */
	// BEGIN_TUTORIAL
	// 
	// Setup
	// ^^^^^
	// 
	// The :move_group_interface:`MoveGroup` class can be easily 
	// setup using just the name
	// of the group you would like to control and plan for.
	moveit::planning_interface::MoveGroup group("arm_group");

	// We will use the :planning_scene_interface:`PlanningSceneInterface`
	// class to deal directly with the world.
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
	moveit_msgs::CollisionObject obj = spawnObject(planning_scene_interface);

	// (Optional) Create a publisher for visualizing plans in Rviz.
	//ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	//moveit_msgs::DisplayTrajectory display_trajectory;

	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame (Plan): %s", group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO("Reference frame (End Effector): %s", group.getEndEffectorLink().c_str());

	// Planning to a Pose goal
	// ^^^^^^^^^^^^^^^^^^^^^^^
	// We can plan a motion for this group to a desired pose for the 
	// end-effector.
	geometry_msgs::Pose target_pose1;

	tf::Quaternion q1(tf::Vector3(0,1,0), 0);
	tf::Quaternion q2(tf::Vector3(0,0,1), 0);

	tf::Quaternion q = q2*q1;

	tf::quaternionTFToMsg(q, target_pose1.orientation);

	target_pose1.position.x = 0.0;
	target_pose1.position.y = 0.02;
	target_pose1.position.z = 1.1474;

	group.setPlanningTime(5.0);
	//group.setPoseReferenceFrame("base_link");
	group.setPoseTarget(target_pose1);

	// Now, we call the planner to compute the plan
	// and visualize it.
	// Note that we are just planning, not asking move_group 
	// to actually move the robot.
	moveit::planning_interface::MoveGroup::Plan my_plan;

	bool success = group.plan(my_plan);

	ROS_INFO("Plan 1 %s",success?"SUCCESS":"FAILED");    

	if(success)
		group.move();

	// Visualizing plans
	// ^^^^^^^^^^^^^^^^^
	// Now that we have a plan we can visualize it in Rviz.  This is not
	// necessary because the group.plan() call we made above did this
	// automatically.  But explicitly publishing plans is useful in cases that we
	// want to visualize a previously created plan.
	//if (1)
	//{
	//  ROS_INFO("Visualizing plan 1 (again)");    
	//  display_trajectory.trajectory_start = my_plan.start_state_;
	//  display_trajectory.trajectory.push_back(my_plan.trajectory_);
	//  display_publisher.publish(display_trajectory);
	//  /* Sleep to give Rviz time to visualize the plan. */
	//  sleep(5.0);
	//}

	// Moving to a pose goal
	// ^^^^^^^^^^^^^^^^^^^^^
	//
	// Moving to a pose goal is similar to the step above
	// except we now use the move() function. Note that
	// the pose goal we had set earlier is still active 
	// and so the robot will try to move to that goal. We will
	// not use that function in this tutorial since it is 
	// a blocking function and requires a controller to be active
	// and report success on execution of a trajectory.

	/* Uncomment below line when working with a real robot*/
	/* group.move() */

	// Planning to a joint-space goal 
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Let's set a joint space goal and move towards it.  This will replace the
	// pose target we set above.
	//
	// First get the current set of joint values for the group.
	std::vector<double> group_variable_values;
	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	// Now, let's modify one of the joints, plan to the new joint
	// space goal and visualize the plan.
	for(std::vector<double>::iterator it = group_variable_values.begin(); it != group_variable_values.end(); ++it){
		*it = 0.0; // set home
	}

	group.setJointValueTarget(group_variable_values);
	success = group.plan(my_plan);
	ROS_INFO("Plan 2 (joint space goal) %s",success?"":"FAILED");

	if(success)
		group.move();

	//sleep(3.0); // sleep for 10 sec.

	//moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	//moveit_simple_grasps::SimpleGrasps simple_grasps;

	//std::vector<moveit_msgs::Grasp> grasps;
	//moveit_msgs::Grasp g;
	//group.setPlanningTime(10.0);
	//group.setSupportSurfaceName("table");

	//moveit::planning_interface::MoveItErrorCode err = group.planGraspsAndPick(obj);
	//std::cerr << err << std::endl;

	//group.pick(obj.id, grasps);
	//sleep(10.0);

	/* Sleep to give Rviz time to visualize the plan. */
	// Planning with Path Constraints
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// Path constraints can easily be specified for a link on the robot.
	// Let's specify a path constraint and a pose goal for our group.
	// First define the path constraint.
	//moveit_msgs::OrientationConstraint ocm;  
	//ocm.link_name = "r_wrist_roll_link";  
	//ocm.header.frame_id = "base_link";
	//ocm.orientation.w = 1.0;
	//ocm.absolute_x_axis_tolerance = 0.1;
	//ocm.absolute_y_axis_tolerance = 0.1;
	//ocm.absolute_z_axis_tolerance = 0.1;
	//ocm.weight = 1.0;
	//
	//// Now, set it as the path constraint for the group.
	//moveit_msgs::Constraints test_constraints;
	//test_constraints.orientation_constraints.push_back(ocm);  
	//group.setPathConstraints(test_constraints);

	//// We will reuse the old goal that we had and plan to it.
	//// Note that this will only work if the current state already 
	//// satisfies the path constraints. So, we need to set the start
	//// state to a new pose. 
	//robot_state::RobotState start_state(*group.getCurrentState());
	//geometry_msgs::Pose start_pose2;
	//start_pose2.orientation.w = 1.0;
	//start_pose2.position.x = 0.55;
	//start_pose2.position.y = -0.05;
	//start_pose2.position.z = 0.8;
	//const robot_state::JointModelGroup *joint_model_group =
	//                start_state.getJointModelGroup(group.getName());
	//start_state.setFromIK(joint_model_group, start_pose2);
	//group.setStartState(start_state);
	//
	//// Now we will plan to the earlier pose target from the new 
	//// start state that we have just created.
	//group.setPoseTarget(target_pose1);
	//success = group.plan(my_plan);

	//ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
	///* Sleep to give Rviz time to visualize the plan. */
	//sleep(10.0);

	//// When done with the path constraint be sure to clear it.
	//group.clearPathConstraints();

	// Cartesian Paths
	// ^^^^^^^^^^^^^^^
	// You can plan a cartesian path directly by specifying a list of waypoints 
	// for the end-effector to go through. Note that we are starting 
	// from the new start state above.  The initial pose (start state) does not
	// need to be added to the waypoint list.
	//std::vector<geometry_msgs::Pose> waypoints;

	//geometry_msgs::Pose target_pose3 = start_pose2;
	//target_pose3.position.x += 0.2;
	//target_pose3.position.z += 0.2;
	//waypoints.push_back(target_pose3);  // up and out

	//target_pose3.position.y -= 0.2;
	//waypoints.push_back(target_pose3);  // left

	//target_pose3.position.z -= 0.2;
	//target_pose3.position.y += 0.2;
	//target_pose3.position.x -= 0.2;
	//waypoints.push_back(target_pose3);  // down and right (back to start)

	//// We want the cartesian path to be interpolated at a resolution of 1 cm
	//// which is why we will specify 0.01 as the max step in cartesian
	//// translation.  We will specify the jump threshold as 0.0, effectively
	//// disabling it.
	//moveit_msgs::RobotTrajectory trajectory;
	//double fraction = group.computeCartesianPath(waypoints,
	//                                             0.01,  // eef_step
	//                                             0.0,   // jump_threshold
	//                                             trajectory);

	//ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
	//      fraction * 100.0);    
	///* Sleep to give Rviz time to visualize the plan. */
	//sleep(15.0);


	// Adding/Removing Objects and Attaching/Detaching Objects
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// First, we will define the collision object message.
	//moveit_msgs::CollisionObject collision_object;
	//collision_object.header.frame_id = group.getPlanningFrame();

	///* The id of the object is used to identify it. */
	//collision_object.id = "box1";

	///* Define a box to add to the world. */
	//shape_msgs::SolidPrimitive primitive;
	//primitive.type = primitive.BOX;
	//primitive.dimensions.resize(3);
	//primitive.dimensions[0] = 0.4;
	//primitive.dimensions[1] = 0.1;
	//primitive.dimensions[2] = 0.4;

	///* A pose for the box (specified relative to frame_id) */
	//geometry_msgs::Pose box_pose;
	//box_pose.orientation.w = 1.0;
	//box_pose.position.x =  0.6;
	//box_pose.position.y = -0.4;
	//box_pose.position.z =  1.2;

	//collision_object.primitives.push_back(primitive);
	//collision_object.primitive_poses.push_back(box_pose);
	//collision_object.operation = collision_object.ADD;

	//std::vector<moveit_msgs::CollisionObject> collision_objects;  
	//collision_objects.push_back(collision_object);  

	//// Now, let's add the collision object into the world
	//ROS_INFO("Add an object into the world");  
	//planning_scene_interface.addCollisionObjects(collision_objects);
	//
	///* Sleep so we have time to see the object in RViz */
	//sleep(2.0);

	//// Planning with collision detection can be slow.  Lets set the planning time
	//// to be sure the planner has enough time to plan around the box.  10 seconds
	//// should be plenty.
	//group.setPlanningTime(10.0);


	//// Now when we plan a trajectory it will avoid the obstacle
	//group.setStartState(*group.getCurrentState());
	//group.setPoseTarget(target_pose1);
	//success = group.plan(my_plan);

	//ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
	//  success?"":"FAILED");
	///* Sleep to give Rviz time to visualize the plan. */
	//sleep(10.0);
	//

	//// Now, let's attach the collision object to the robot.
	//ROS_INFO("Attach the object to the robot");  
	//group.attachObject(collision_object.id);  
	///* Sleep to give Rviz time to show the object attached (different color). */
	//sleep(4.0);


	//// Now, let's detach the collision object from the robot.
	//ROS_INFO("Detach the object from the robot");  
	//group.detachObject(collision_object.id);  
	///* Sleep to give Rviz time to show the object detached. */
	//sleep(4.0);


	//// Now, let's remove the collision object from the world.
	//ROS_INFO("Remove the object from the world");  
	//std::vector<std::string> object_ids;
	//object_ids.push_back(collision_object.id);  
	//planning_scene_interface.removeCollisionObjects(object_ids);
	///* Sleep to give Rviz time to show the object is no longer there. */
	//sleep(4.0);

	ros::shutdown();  
	return 0;
}
