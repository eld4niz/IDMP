import rospy
import numpy as np
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler

class ChompPlanner:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('chomp_planner', anonymous=True)

        # MoveIt setup
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "right_arm"
        self.move_group = MoveGroupCommander(self.group_name)
        
        # Set the planner to CHOMP
        self.move_group.set_planner_id("CHOMP")

        # Planning parameters
        self.planning_time = 5.0
        self.num_planning_attempts = 10
        
        # Set velocity and acceleration scaling
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.2)

        # Print current robot state information for debugging
        rospy.loginfo("CHOMP Planner initialized")
        rospy.loginfo(f"Available joints: {self.move_group.get_active_joints()}")
        rospy.loginfo(f"Current end effector pose: {self.move_group.get_current_pose().pose}")
        rospy.loginfo(f"Current joint values: {self.move_group.get_current_joint_values()}")
        rospy.loginfo(f"Robot reference frame: {self.move_group.get_planning_frame()}")
        rospy.loginfo(f"End effector link: {self.move_group.get_end_effector_link()}")

    def plan_and_execute_joint_target(self, joint_positions):
        """Plan and execute motion using CHOMP with joint targets"""
        rospy.loginfo("Starting motion planning with CHOMP (joint space)...")
        rospy.loginfo(f"Target joint values: {joint_positions}")

        # Set planning parameters
        self.move_group.set_planning_time(self.planning_time)
        self.move_group.set_num_planning_attempts(self.num_planning_attempts)

        # Set joint-space goal
        try:
            self.move_group.set_joint_value_target(joint_positions)
        except Exception as e:
            rospy.logerr(f"Failed to set joint target: {e}")
            return False
        
        # Plan the motion
        plan = self.move_group.plan()
        
        # Handle different MoveIt versions
        if isinstance(plan, tuple):
            success = plan[0]
            trajectory = plan[1]
        else:
            success = len(plan.joint_trajectory.points) > 0
            trajectory = plan
            
        if success:
            rospy.loginfo("Planning succeeded, executing motion...")
            self.move_group.execute(trajectory, wait=True)
            rospy.loginfo("Motion execution complete")
            return True
        else:
            rospy.logerr("Motion planning failed")
            return False

    def plan_to_named_target(self, target_name):
        """Plan to a named target configuration"""
        rospy.loginfo(f"Planning to named target: {target_name}")
        
        try:
            self.move_group.set_named_target(target_name)
            plan = self.move_group.plan()
            
            # Handle different MoveIt versions
            if isinstance(plan, tuple):
                success = plan[0]
                trajectory = plan[1]
            else:
                success = len(plan.joint_trajectory.points) > 0
                trajectory = plan
                
            if success:
                rospy.loginfo("Planning succeeded, executing motion...")
                self.move_group.execute(trajectory, wait=True)
                rospy.loginfo("Motion execution complete")
                return True
            else:
                rospy.logerr("Motion planning failed")
                return False
        except Exception as e:
            rospy.logerr(f"Error planning to named target: {e}")
            return False

    def move_relative_joint(self, joint_index, delta):
        """Move a single joint by a relative amount"""
        current_joints = self.move_group.get_current_joint_values()
        if joint_index < 0 or joint_index >= len(current_joints):
            rospy.logerr(f"Invalid joint index: {joint_index}")
            return False
            
        target_joints = current_joints.copy()
        target_joints[joint_index] += delta
        
        return self.plan_and_execute_joint_target(target_joints)

    def move_to_joint_positions(self, positions_list):
        """Move through a sequence of joint positions"""
        success = True
        for i, joint_positions in enumerate(positions_list):
            rospy.loginfo(f"Moving to joint position {i+1}/{len(positions_list)}")
            if not self.plan_and_execute_joint_target(joint_positions):
                success = False
                break
        return success

    def sample_reachable_joint_positions(self, num_samples=5):
        """Generate sample joint positions that should be reachable"""
        current_joints = self.move_group.get_current_joint_values()
        samples = [current_joints]
        
        # Add some random variations to current position
        for _ in range(num_samples-1):
            random_delta = [(np.random.random() - 0.5) * 0.2 for _ in current_joints]  # +/- 0.1 radians
            new_sample = [current + delta for current, delta in zip(current_joints, random_delta)]
            samples.append(new_sample)
            
        return samples

def main():
    try:
        planner = ChompPlanner()
        
        # Get the current joint values
        current_joints = planner.move_group.get_current_joint_values()
        
        # Strategy 1: Try to move a single joint a small amount
        joint_indices = range(len(current_joints))
        joint_names = planner.move_group.get_active_joints()
        
        rospy.loginfo("Moving each joint incrementally:")
        for idx, name in zip(joint_indices, joint_names):
            rospy.loginfo(f"Moving joint {name} (index {idx})")
            planner.move_relative_joint(idx, 0.1)  # Move 0.1 radians
            rospy.sleep(1)
            planner.move_relative_joint(idx, -0.1)  # Move back
            rospy.sleep(1)
        
        # Generate a simple sequence of reachable joint positions
        rospy.loginfo("Moving through sequence of reachable joint positions")
        positions_sequence = planner.sample_reachable_joint_positions(3)
        planner.move_to_joint_positions(positions_sequence)
        
        # Clean up
        planner.move_group.clear_pose_targets()
        rospy.loginfo("Motion sequence complete")
        rospy.sleep(2)

    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
