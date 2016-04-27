#include <ros/ros.h>
#include <geometric_shapes/shape_operations.h>

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
  moveit_msgs::CollisionObject co;
  co.header.frame_id = group.getPlanningFrame();
  co.id= "muros";

  shapes::Mesh* m = shapes::createMeshFromResource("package://pr2_moveit_ltl/models/ENV_6.dae");
  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;
  shapes::constructMsgFromShape(m,co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
  co.meshes.resize(1);
  co.meshes[0] = co_mesh;
  co.mesh_poses.resize(1);
  co.mesh_poses[0].position.x = -5.0;
  co.mesh_poses[0].position.y = -5.0;
  co.mesh_poses[0].position.z = 0.0;
  co.mesh_poses[0].orientation.w= 0.7071;
  co.mesh_poses[0].orientation.x= 0.7071 ;
  co.mesh_poses[0].orientation.y= 0.0;
  co.mesh_poses[0].orientation.z= 0.0;

  co.meshes.push_back(co_mesh);
  co.mesh_poses.push_back(co.mesh_poses[0]);
  co.operation = co.ADD;
  
  /* Publish and sleep (to view the visualized results) */
  collision_object_publisher.publish(co);
  ros::WallDuration sleep_time(10.0);
  sleep_time.sleep();

  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(co);
  planning_scene_interface.addCollisionObjects(collision_objects);
  sleep(10.0);

  
  // First get the current set of joint values for the group.
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  
  // Now, let's modify one of the joints, plan to the new joint
  // space goal and visualize the plan.
  group_variable_values[0] = -4.0; 
  group_variable_values[1] = -4.0;
  group_variable_values[2] = 1.5;
  group.setJointValueTarget(group_variable_values);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  group.setPlannerId("RRTstarkConfigDefault");
  group.setPlanningTime(10.0);

  bool success = group.plan(my_plan);



  if (success){
    group.execute(my_plan);
  }
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
