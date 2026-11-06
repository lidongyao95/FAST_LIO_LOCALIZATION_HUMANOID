"""Publish five translated copies of the same view in an isolated ROS domain."""
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs_py.point_cloud2 import create_cloud
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

rclpy.init()
node = rclpy.create_node('hang_a02_filter_review')
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
cloud_pub = node.create_publisher(PointCloud2, '/comparison_cloud', qos)
markers_pub = node.create_publisher(MarkerArray, '/comparison_markers', qos)
header = Header(frame_id='map')
data = np.load('grids.npz')
chunks = []
markers = MarkerArray()
views = [
    (0, 'Baseline', 0., 12.), (3, 'Height 0.45-1.20 m', 20., 12.),
    (6, 'Drop within 1.50 m', 40., 12.), (8, 'One scan per second', 0., -10.),
    (12, 'IMU P95 <= 0.60 m/s2', 20., -10.),
]
for i, (key, label, dx, dy) in enumerate(views):
    xyz = np.load(f'cloud-{key}.npy')
    keep = ((xyz[:,0] >= -7) & (xyz[:,0] < 11) & (xyz[:,1] >= -8) & (xyz[:,1] < 10))
    xyz = xyz[keep] + np.array([dx, dy, 0], dtype=np.float32)
    chunks.append(xyz)
    text = Marker(header=header, id=2*i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
    text.pose.position = Point(x=dx+2, y=dy+11, z=3.)
    text.pose.orientation.w = 1.
    text.scale.z = .82
    text.color.r = text.color.g = text.color.b = text.color.a = 1.
    text.text = label
    line = Marker(header=header, id=2*i+1, type=Marker.LINE_STRIP, action=Marker.ADD)
    line.pose.orientation.w = 1.
    line.scale.x = .047
    line.color.g, line.color.b, line.color.a = .85, 1., 1.
    line.points = [Point(x=float(x+dx), y=float(y+dy), z=2.55) for x,y,_ in data['body']]
    markers.markers.extend([text, line])
fields = [PointField(name=axis, offset=i*4, datatype=PointField.FLOAT32, count=1)
          for i, axis in enumerate('xyz')]
cloud_pub.publish(create_cloud(header, fields, np.concatenate(chunks)))
markers_pub.publish(markers)
rclpy.spin(node)
