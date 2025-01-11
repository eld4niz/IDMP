import rospy
import numpy as np
from moveit_commander import MoveGroupCommander
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from idmp_ros.srv import GetDistanceGradient
import matplotlib.pyplot as plt
from matplotlib import cm

class ChompIDMPPlanner:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('chomp_idmp_planner', anonymous=True)
        
        # MoveIt setup
        self.group_name = "right_arm"
        self.move_group = MoveGroupCommander(self.group_name)
        
        # Wait for IDMP service
        rospy.wait_for_service('query_dist_field')
        self.query_distance_field = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)
        
        # Publishers for visualization
        self.marker_pub = rospy.Publisher('gradient_visualization', MarkerArray, queue_size=10)
        
        # Planning parameters
        self.planning_time = 5.0
        self.num_planning_attempts = 10
        self.collision_threshold = 0.1
        
        rospy.loginfo("CHOMP-IDMP Planner initialized")


    def create_gradient_marker(self, position, gradient, marker_id, color):
        """Create visualization marker for gradients"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set arrow properties
        marker.scale.x = 0.02  # shaft diameter
        marker.scale.y = 0.04  # head diameter
        marker.scale.z = 0.0   # head length
        
        # Set color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # Set arrow points
        start_point = Point(*position)
        end_point = Point(*(position + gradient * 0.15))  # Scale gradient for visualization
        marker.points = [start_point, end_point]
        
        return marker


    def query_idmp(self, positions):
        """Query IDMP for distances and gradients"""
        try:
            # Flatten positions array for service call
            positions_flat = positions.reshape(-1).tolist()
            response = self.query_distance_field(positions_flat)
            
            # Process response
            distances = np.array(response.distances)
            gradients = np.array(response.gradients).reshape(-1, 3)
            
            return distances, gradients
        except rospy.ServiceException as e:
            rospy.logerr(f"IDMP service call failed: {e}")
            return None, None


    def visualize_gradients(self, positions, distances, gradients):
        """Visualize distance field gradients"""
        marker_array = MarkerArray()
        
        # Normalize distances for coloring
        normalized_distances = (distances - distances.min()) / (distances.max() - distances.min())
        colors = plt.cm.gist_rainbow(normalized_distances)
        
        # Create markers for each position
        for i, (pos, grad, color) in enumerate(zip(positions, gradients, colors)):
            if i % 2 == 0:  # Skip every other point to reduce visual clutter
                marker = self.create_gradient_marker(pos, grad, i, color)
                marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)


    def plan_and_execute(self, joint_target):
        """Plan and execute motion using IDMP gradients"""
        rospy.loginfo("Starting motion planning with IDMP gradients...")
        
        # Set planning parameters
        self.move_group.set_planning_time(self.planning_time)
        self.move_group.set_num_planning_attempts(self.num_planning_attempts)
        
        # Get current robot pose
        start_pose = self.move_group.get_current_pose().pose
        
        waypoints = [start_pose]
        target_pose = Pose()
        target_pose.position.x = start_pose.position.x + 0.2
        target_pose.position.y = start_pose.position.y - 0.3
        target_pose.position.z = start_pose.position.z
        target_pose.orientation = start_pose.orientation
        waypoints.append(target_pose)
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,      # waypoints to follow
            0.01,           # eef_step
            0.0,            # jump_threshold
            avoid_collisions=True
        )
        
        # Extract positions for IDMP
        positions = np.array([[wp.position.x, wp.position.y, wp.position.z] for wp in waypoints])
        
        # Query IDMP for distances and gradients
        distances, gradients = self.query_idmp(positions)
        
        if distances is not None:
            # Apply gradients to refine trajectory
            updated_positions = positions - 0.1 * gradients  # Gradient descent adjustment
            
            # Visualize the distance field and updated gradients
            self.visualize_gradients(updated_positions, distances, gradients)
            
            # Update waypoints with adjusted positions
            for wp, new_pos in zip(waypoints, updated_positions):
                wp.position.x, wp.position.y, wp.position.z = new_pos
            
            # Plan final trajectory
            (final_plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints, 0.01, 0.0, avoid_collisions=True
            )
            
            if final_plan.joint_trajectory.points:
                # Execute the plan
                self.move_group.execute(final_plan, wait=True)
                rospy.loginfo("Motion execution complete")
                return True
            else:
                rospy.logerr("Motion planning failed")
                return False
        else:
            rospy.logerr("Failed to get distance field from IDMP")
            return False


def main():
    try:
        planner = ChompIDMPPlanner()
        
        # Get current joint values
        current_joint_values = planner.move_group.get_current_joint_values()
        rospy.loginfo("Current Joint Values: %s", current_joint_values)
        
        # Define joint target (example values)
        joint_target = current_joint_values[:]
        joint_target[0] += 0.2  # Increment joint 1
        joint_target[1] -= 0.3  # Decrement joint 2
        
        # Plan and execute
        success = planner.plan_and_execute(joint_target)
        
        # Clean up
        planner.move_group.clear_pose_targets()
        rospy.loginfo("Shutting down")
        rospy.sleep(2)
        
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
