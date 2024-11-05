import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from idmp_ros.srv import GetDistanceGradient
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib

matplotlib.use("Qt5Agg")

def pose_to_numpy(msg):
    return np.dot(
        transformations.translation_matrix(np.array([msg.position.x, msg.position.y, msg.position.z])),
        transformations.quaternion_matrix(np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))
    )

def numpy_to_point(arr):
    if arr.shape[-1] == 4:
        arr = arr[...,:-1] / arr[...,-1]

    if len(arr.shape) == 1:
        return Point(*arr)
    else:
        return np.apply_along_axis(lambda v: Point(*v), axis=-1, arr=arr)

def create_arrow(scale, start, end, idnum, color=None):
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = 'base_link'
    m.header.stamp = rospy.Time.now()
    m.id = idnum
    m.type = Marker.ARROW
    m.pose.orientation.w = 1
    m.scale.x = scale * 0.5
    m.scale.y = scale
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = color[3]
    m.points = [numpy_to_point(start), numpy_to_point(end)]
    return m

def get_distance_gradient(x, y, z):
    # Initialize ROS node if it hasn't been initialized
    if not rospy.get_node_uri():
        rospy.init_node("distance_gradient_query")

    # Wait for the service to be available
    rospy.wait_for_service('query_dist_field')
    try:
        query_service = rospy.ServiceProxy('query_dist_field', GetDistanceGradient)
        
        # Send the point as a flat array with homogeneous coordinates
        point = np.array([x, y, z, 1])
        res = query_service(point[:3].tolist())
        
        distances = np.array(res.distances)
        gradients = np.array(res.gradients).reshape(-1, 3)
        
        # Check if distances and gradients are within bounds (modify as per requirement)
        in_bounds = (distances < 5).all()

        return distances, gradients, in_bounds

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return None, None, False

def visualize_point_gradients(x, y, z, distances, gradients):
    plt.ion()
    marker_pub = rospy.Publisher("grad_array", MarkerArray, queue_size=0)
    
    # Scale and color map for distance and gradients visualization
    scale = 0.04
    color_map = plt.cm.gist_rainbow((distances - distances.min()) / distances.ptp())
    
    # Create visualization markers
    marker_array = MarkerArray()
    for idx, (gradient, dist, color) in enumerate(zip(gradients, distances, color_map)):
        start = np.array([x, y, z])
        end = start + gradient * 0.15  # Adjust scaling as needed
        arrow_marker = create_arrow(scale, start, end, idx, color=color[:4])
        marker_array.markers.append(arrow_marker)
    
    # Publish markers for visualization in RViz or other ROS visualization tools
    marker_pub.publish(marker_array)
    rospy.Rate(1).sleep()  # Adjust rate if necessary

if __name__ == "__main__":
    # Input for the 3D point
    x, y, z = map(float, input("Enter x, y, z coordinates: ").split())

    # Fetch distances, gradients, and bounds for the specified point
    distances, gradients, in_bounds = get_distance_gradient(x, y, z)
    
    if distances is not None and in_bounds:
        print(f"Gradients: {gradients}\nDistances: {distances}\nIn Bounds: {in_bounds}")
        visualize_point_gradients(x, y, z, distances, gradients)
    else:
        print("Point is out of bounds or service call failed.")
