#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_base");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* This sleep is ONLY to allow Rviz to come up */
  sleep(10.0);
  moveit::planning_interface::MoveGroup group("base");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  while(collision_object_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }

  //Agregar objeto que colisiona
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();
  collision_object.id = "box1";

  /* A default pose */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  -1.0;
  box_pose.position.y = -1.0;
  box_pose.position.z =  0.5;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.5;
  primitive.dimensions[1] = 4;
  primitive.dimensions[2] = 1;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  
  /* Publish and sleep (to view the visualized results) */
  collision_object_publisher.publish(collision_object);
  ros::WallDuration sleep_time(1.0);
  sleep_time.sleep();

  /* CHECK IF A STATE IS VALID */
  /* PUT THE OBJECT IN THE ENVIRONMENT */
  
  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);
  sleep(10.0);


  // First get the current set of joint values for the group.
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  
  // Now, let's modify one of the joints, plan to the new joint
  // space goal and visualize the plan.
  group_variable_values[0] = -3.0; 
  group_variable_values[1] = -3.0;
  group_variable_values[2] = 1.5;
  group.setJointValueTarget(group_variable_values);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  group.setPlanningTime(10.0);
  bool success = group.plan(my_plan);
  std::vector<std::string> obj = planning_scene_interface.getKnownObjectNames();
  
if (obj.size() > 0)
  {
    std::cout << std::endl << "-- KNOWN COLLISION OBJECTS --" << std::endl;
    for (int i = 0; i < obj.size(); ++i)
      std::cout << obj[i] << std::endl;
  }
  ros::spin();
  return 0;
}
