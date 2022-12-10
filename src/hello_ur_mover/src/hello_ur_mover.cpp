#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("hello_ur_mover")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Pose>("pose_msg", 1, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Bool>("searchBool", 1);
      pub_timer_ = this->create_publisher<std_msgs::msg::Bool>("timer", 1);
    
    }

  private:
    void topic_callback(const geometry_msgs::msg::Pose::SharedPtr msg) const
    {

      auto const node = std::make_shared<rclcpp::Node>(
          "hello_ur_mover",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );

      // Create pub messege type
      auto searchBool = std_msgs::msg::Bool();
      auto timer = std_msgs::msg::Bool();

      // Create a ROS logger
      auto const logger = rclcpp::get_logger("hello_ur_mover");

      // Create the MoveIt MoveGroup Interface
      using moveit::planning_interface::MoveGroupInterface;
      auto move_group_interface = MoveGroupInterface(node, "ur_manipulator"); //change
      
      // move_group_interface.setPoseReferenceFrame("base");


      const float quan_w = msg->orientation.w;
      const float quan_x = msg->orientation.x;
      const float quan_y = msg->orientation.y;
      const float quan_z = msg->orientation.z;
      const float post_x = msg->position.x;
      const float post_y = msg->position.y;
      const float post_z = msg->position.z;



      // // Joint Paths
      // auto const target_pose = [x,y,z]{
      //   geometry_msgs::msg::Pose set_msg;
      //   set_msg.orientation.w = 0;
      //   set_msg.orientation.x = 1;
      //   set_msg.orientation.y = 0;
      //   set_msg.orientation.z = 0;
      //   set_msg.position.x = x;
      //   set_msg.position.y = y;
      //   set_msg.position.z = z;
      //   return set_msg;
      // }();

      // move_group_interface.setPoseTarget(target_pose);
      // // move_group_interface.setPositionTarget(x,y,z);
      // // move_group_interface.setRPYTarget(0,0,0);



      // get current pose

      //auto const current_pose = move_group_interface.getCurrentPose();

      // Create collision object for the robot to avoid
      auto const collision_object_1 = [frame_id =
                                      move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object_1;
        collision_object_1.header.frame_id = frame_id;
        collision_object_1.id = "box1";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 3;
        primitive.dimensions[primitive.BOX_Y] = 3;
        primitive.dimensions[primitive.BOX_Z] = 0.1;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0;
        box_pose.position.y = 0;
        box_pose.position.z = -0.055;

        collision_object_1.primitives.push_back(primitive);
        collision_object_1.primitive_poses.push_back(box_pose);
        collision_object_1.operation = collision_object_1.ADD;

        return collision_object_1;
      }();

            
      // Add the collision object to the scene
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      planning_scene_interface.applyCollisionObject(collision_object_1);

      //------------------------------------------------------------

      // // Create collision object for the robot to avoid
      // auto const collision_object_2 = [frame_id =
      //                                 move_group_interface.getPlanningFrame()] {
      //   moveit_msgs::msg::CollisionObject collision_object_2;
      //   collision_object_2.header.frame_id = frame_id;
      //   collision_object_2.id = "box2";
      //   shape_msgs::msg::SolidPrimitive primitive;

      //   // Define the size of the box in meters
      //   primitive.type = primitive.BOX;
      //   primitive.dimensions.resize(3);
      //   primitive.dimensions[primitive.BOX_X] = 3;
      //   primitive.dimensions[primitive.BOX_Y] = 0.1;
      //   primitive.dimensions[primitive.BOX_Z] = 3;

      //   // Define the pose of the box (relative to the frame_id)
      //   geometry_msgs::msg::Pose box_pose;
      //   box_pose.orientation.w = 0.3826834;
      //   box_pose.orientation.x = 0;
      //   box_pose.orientation.y = 0;
      //   box_pose.orientation.z = 0.9238795;
      //   box_pose.position.x = -0.3;
      //   box_pose.position.y = -0.3;
      //   box_pose.position.z = 0;

      //   collision_object_2.primitives.push_back(primitive);
      //   collision_object_2.primitive_poses.push_back(box_pose);
      //   collision_object_2.operation = collision_object_2.ADD;

      //   return collision_object_2;
      // }();

            
      // // Add the collision object to the scene
      // // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      // planning_scene_interface.applyCollisionObject(collision_object_2);


      //--------------------------------
      
      // Create collision object for the robot to avoid
      auto const collision_object_3 = [frame_id =
                                      move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object_3;
        collision_object_3.header.frame_id = frame_id;
        collision_object_3.id = "box3";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.1;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.1;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1;
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.position.x = 0.7;
        box_pose.position.y = 0.7;
        box_pose.position.z = -0.05;

        collision_object_3.primitives.push_back(primitive);
        collision_object_3.primitive_poses.push_back(box_pose);
        collision_object_3.operation = collision_object_3.ADD;

        return collision_object_3;
      }();

            
      // Add the collision object to the scene
      // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      planning_scene_interface.applyCollisionObject(collision_object_3);




      // Cartesian Paths
      // ^^^^^^^^^^^^^^^
      // You can plan a Cartesian path directly by specifying a list of waypoints
      // for the end-effector to go through. Note that we are starting
      // from the new start state above.  The initial pose (start state) does not
      // need to be added to the waypoint list but adding it can help with visualizations
      std::vector<geometry_msgs::msg::Pose> waypoints;


      geometry_msgs::msg::Pose target_pose1;

      target_pose1.orientation.w = quan_w;
      target_pose1.orientation.x = quan_x;
      target_pose1.orientation.y = quan_y;
      target_pose1.orientation.z = quan_z;
      target_pose1.position.z = post_z;
      target_pose1.position.y = post_y;
      target_pose1.position.x = post_x;
      waypoints.push_back(target_pose1); 

      std::cout <<"post x: " << post_x << std::endl;
      std::cout <<"post y: " << post_y << std::endl;
      std::cout <<"post z: " << post_z << std::endl;

      std::cout <<"quan w: " << quan_w << std::endl;
      std::cout <<"quan x: " << quan_x << std::endl;
      std::cout <<"quan y: " << quan_y << std::endl;
      std::cout <<"quan z: " << quan_z << std::endl;

      // We want the Cartesian path to be interpolated at a resolution of 1 cm
      // which is why we will specify 0.01 as the max step in Cartesian
      // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
      // Warning - disabling the jump threshold while operating real hardware can cause
      // large unpredictable motions of redundant joints and could be a safety issue
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      RCLCPP_INFO(logger, "Visualizing plan 1 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

      if (fraction >0.75){
      move_group_interface.execute(trajectory);

      std::cout << "frame name: " <<move_group_interface.getPoseReferenceFrame() << std::endl;

      timer.data = false;
      pub_timer_->publish(timer);


      // std::string inputString;
      // std::cout << "Give input" << std::endl;
      // std::getline(std::cin, inputString);
        
      
      
      //-----------------
      sleep(2);

      std::vector<geometry_msgs::msg::Pose> waypoints2;

      geometry_msgs::msg::Pose target_pose2;

      target_pose2.orientation.w = 0;
      target_pose2.orientation.x = -1;
      target_pose2.position.x = 0.5;
      target_pose2.position.y = 0.5;
      target_pose2.position.z = 0.65;
      waypoints2.push_back(target_pose2); 


      // We want the Cartesian path to be interpolated at a resolution of 1 cm
      // which is why we will specify 0.01 as the max step in Cartesian
      // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
      // Warning - disabling the jump threshold while operating real hardware can cause
      // large unpredictable motions of redundant joints and could be a safety issue
      moveit_msgs::msg::RobotTrajectory trajectory2;


      fraction = move_group_interface.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);
      RCLCPP_INFO(logger, "Visualizing plan 2 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

      move_group_interface.execute(trajectory2); 
      //------------

      sleep(2);
      };
      
      searchBool.data = true;
      publisher_->publish(searchBool);
      // sleep(1);
      // publisher_->publish(searchBool);
      // sleep(1);
      // publisher_->publish(searchBool);


      // // Create a plan to that target pose
      // auto const [success, plan] = [&move_group_interface]{
      //   moveit::planning_interface::MoveGroupInterface::Plan msg;
      //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      //   return std::make_pair(ok, msg);
      // }();

      // // Execute the plan
      // if(success) {
      //   move_group_interface.execute(plan);
      // } else {
      //   RCLCPP_ERROR(logger, "Planing failed!");
      // }
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

