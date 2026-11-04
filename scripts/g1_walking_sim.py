#!/usr/bin/env python3
"""Single MuJoCo owner; official low-level DDS + independent ROS sensors."""
import argparse
import ctypes
import math
import time
import numpy as np
import mujoco
import rclpy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, SetBool
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from g1_mujoco_sensors import Sensors, generate, ROOT

class WalkingSim(Sensors):
    def __init__(self, domain, start_x=0., start_y=0., start_yaw=0.,
                 obstacle_x=2.2, obstacle_y=.4, model_path=None):
        super().__init__(model_path=model_path)
        if model_path is not None:
            self.groups[3] = 1  # Scan collision voxels are sensor-visible.
        self.model.qpos0[0:2]=[start_x,start_y]
        self.model.qpos0[3:7]=[math.cos(start_yaw/2),0.,0.,math.sin(start_yaw/2)]
        support=self.model.body('support')
        support.pos[0:2]=[start_x,start_y]
        support.quat[:]=self.model.qpos0[3:7]
        mujoco.mj_resetData(self.model,self.data);mujoco.mj_forward(self.model,self.data)
        self.epoch_ns=0
        self.dds=ctypes.CDLL(str(ROOT/'.deps/controller-build/libg1_dds_bridge.so'))
        ptr=ctypes.POINTER(ctypes.c_double)
        self.dds.bridge_init.argtypes=[ctypes.c_int]
        self.dds.bridge_step.argtypes=[ptr]*6+[ctypes.c_int]
        self.dds.bridge_init(domain)
        self.ptr=ptr
        self.obstacle_xy=[float(obstacle_x),float(obstacle_y)]
        self.obstacle_active=False
        obstacle_body=self.model.body('nav_dynamic_obstacle').id
        self.obstacle_mocap_id=int(self.model.body_mocapid[obstacle_body])
        self.set_obstacle(False)
        self.fresh=False;self.reset_wall=False
        self.clock_pub=self.create_publisher(Clock,'/clock',10)
        self.joint_pub=self.create_publisher(JointState,'/joint_states',10)
        self.released_pub=self.create_publisher(Bool,'/g1/released',10)
        self.obstacle_pub=self.create_publisher(Bool,'/g1/obstacle_active',10)
        self.obstacle_pose_pub=self.create_publisher(PoseStamped,'/g1/obstacle_pose',10)
        self.create_service(Trigger,'/g1/release',self.release)
        self.create_service(Trigger,'/g1/reset',self.reset)
        self.create_service(SetBool,'/g1/obstacle',self.obstacle)
        self.get_logger().info('DDS mode: base held until /g1/release, no scripted translation or internal PD')
    def release(self,req,res):
        res.success=self.fresh
        if self.fresh:self.data.eq_active[0]=False
        res.message='Base released' if self.fresh else 'No fresh LowCmd; keeping support'
        return res
    def reset(self,req,res):
        mujoco.mj_resetData(self.model,self.data);mujoco.mj_forward(self.model,self.data)
        self.set_obstacle(False)
        self.epoch_ns=0;self.next_scan=0.;self.reset_wall=True
        res.success=True;res.message='Reset; restart localization after controller is ready'
        return res
    def set_obstacle(self,active):
        self.obstacle_active=bool(active)
        z=.75 if self.obstacle_active else -5.
        self.data.mocap_pos[self.obstacle_mocap_id]=[self.obstacle_xy[0],self.obstacle_xy[1],z]
    def obstacle(self,req,res):
        self.set_obstacle(req.data)
        res.success=True
        res.message=('Dynamic obstacle placed' if req.data else 'Dynamic obstacle removed')
        self.get_logger().warn(res.message+f' at ({self.obstacle_xy[0]:.2f}, {self.obstacle_xy[1]:.2f})')
        return res
    def apply_control(self):
        arrays=[np.ascontiguousarray(x,dtype=np.float64) for x in [self.data.qpos[7:],self.data.qvel[6:],self.data.sensor('imu_quat').data,self.data.sensor('imu_gyro').data,self.data.sensor('imu_acc').data]]
        torque=np.zeros(29)
        self.fresh=bool(self.dds.bridge_step(*[x.ctypes.data_as(self.ptr) for x in arrays],torque.ctypes.data_as(self.ptr),round(self.data.time*1000)))
        self.data.ctrl[:]=np.clip(torque,self.model.actuator_ctrlrange[:,0],self.model.actuator_ctrlrange[:,1])
    def step(self):
        super().step()
        clock=Clock();clock.clock=rclpy.time.Time(nanoseconds=round(self.data.time*1e9)).to_msg();self.clock_pub.publish(clock)
        if int(self.data.time*500)%10==0:
            msg=JointState();msg.header.stamp=rclpy.time.Time(nanoseconds=round(self.data.time*1e9)).to_msg()
            msg.name=[self.model.joint(i).name for i in range(1,self.model.njnt)]
            msg.position=self.data.qpos[7:].tolist();msg.velocity=self.data.qvel[6:].tolist();self.joint_pub.publish(msg)
        if int(self.data.time*500)%25==0:
            released=Bool();released.data=not bool(self.data.eq_active[0]);self.released_pub.publish(released)
            obstacle=Bool();obstacle.data=self.obstacle_active;self.obstacle_pub.publish(obstacle)
            pose=PoseStamped()
            pose.header.stamp=rclpy.time.Time(nanoseconds=round(self.data.time*1e9)).to_msg()
            pose.header.frame_id='map'
            pose.pose.position.x,pose.pose.position.y=self.obstacle_xy
            pose.pose.position.z=.75 if self.obstacle_active else -5.
            pose.pose.orientation.w=1.
            self.obstacle_pose_pub.publish(pose)

def main():
    p=argparse.ArgumentParser();p.add_argument('--headless',action='store_true');p.add_argument('--domain',type=int,default=72)
    p.add_argument('--start-x',type=float,default=0.);p.add_argument('--start-y',type=float,default=0.)
    p.add_argument('--start-yaw',type=float,default=0.)
    p.add_argument('--obstacle-x',type=float,default=2.2)
    p.add_argument('--obstacle-y',type=float,default=.4)
    p.add_argument('--model-path', help='Existing MJCF scene; skip default room generation')
    args=p.parse_args()
    if args.model_path is None: generate()
    rclpy.init()
    node=WalkingSim(args.domain,args.start_x,args.start_y,args.start_yaw,
                    args.obstacle_x,args.obstacle_y,args.model_path)
    viewer=None
    if not args.headless:
        import mujoco.viewer
        viewer=mujoco.viewer.launch_passive(node.model,node.data);viewer.cam.distance=7;viewer.cam.lookat[:]=[0,0,1]
        if args.model_path: viewer.opt.geomgroup[3]=0
    start=time.monotonic();next_view=start
    try:
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            node.step();rclpy.spin_once(node,timeout_sec=0.)
            if node.reset_wall:start=time.monotonic()-node.data.time;node.reset_wall=False
            if viewer and time.monotonic()>=next_view:viewer.sync();next_view=time.monotonic()+1/30
            time.sleep(max(0.,start+node.data.time-time.monotonic()))
    except KeyboardInterrupt:pass
    finally:
        if viewer:viewer.close()
        node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
