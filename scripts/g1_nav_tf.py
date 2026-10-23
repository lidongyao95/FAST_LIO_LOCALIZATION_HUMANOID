#!/usr/bin/env python3
"""Expose local FAST-LIO odometry as planar odom -> base_footprint TF."""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class NavTF(Node):
    def __init__(self):
        super().__init__('g1_nav_tf')
        self.declare_parameter('foot_height',.793)
        self.height=self.get_parameter('foot_height').value
        self.pub=self.create_publisher(Odometry,'/odom',20)
        self.tf=TransformBroadcaster(self)
        self.prev=None
        self.create_subscription(Odometry,'/Odometry_loc',self.on_odom,20)
    def on_odom(self,msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        yaw=yaw_of(q)
        stamp=msg.header.stamp
        t=stamp.sec+stamp.nanosec*1e-9
        v=(0.,0.,0.)
        if self.prev is not None:
            dt=t-self.prev[0]
            if .001<dt<.5:
                dx=(p.x-self.prev[1])/dt;dy=(p.y-self.prev[2])/dt
                v=(math.cos(yaw)*dx+math.sin(yaw)*dy,
                   -math.sin(yaw)*dx+math.cos(yaw)*dy,
                   math.atan2(math.sin(yaw-self.prev[3]),math.cos(yaw-self.prev[3]))/dt)
        self.prev=(t,p.x,p.y,yaw)
        half=yaw/2
        planar=[0.,0.,math.sin(half),math.cos(half)]
        foot=TransformStamped();foot.header.stamp=stamp;foot.header.frame_id='odom';foot.child_frame_id='base_footprint'
        foot.transform.translation.x=p.x;foot.transform.translation.y=p.y;foot.transform.translation.z=-self.height
        foot.transform.rotation.x,foot.transform.rotation.y,foot.transform.rotation.z,foot.transform.rotation.w=planar
        self.tf.sendTransform(foot)
        odom=Odometry();odom.header=foot.header;odom.child_frame_id='base_footprint'
        odom.pose.pose.position.x=p.x;odom.pose.pose.position.y=p.y;odom.pose.pose.position.z=-self.height
        odom.pose.pose.orientation=foot.transform.rotation
        odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z=v
        self.pub.publish(odom)
        # Full pelvis attitude and vertical motion remain visible in the robot model.
        base=TransformStamped();base.header.stamp=stamp;base.header.frame_id='base_footprint';base.child_frame_id='base_link'
        base.transform.translation.z=p.z+self.height
        sy,cy=math.sin(-half),math.cos(-half)
        base.transform.rotation.x=cy*q.x-sy*q.y
        base.transform.rotation.y=cy*q.y+sy*q.x
        base.transform.rotation.z=cy*q.z+sy*q.w
        base.transform.rotation.w=cy*q.w-sy*q.z
        self.tf.sendTransform(base)

def main():
    rclpy.init();node=NavTF()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
