import io,sys,time,numpy as np,rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer,TransformListener
rclpy.init();n=rclpy.create_node('alignment_capture');b=Buffer();l=TransformListener(b,n);frames=[];last=0
frame=sys.argv[1]
def cb(m):
 global last
 now=time.monotonic()
 if now-last<.4:return
 try:t=b.lookup_transform(frame,m.header.frame_id,Time()).transform
 except Exception:return
 a=np.ndarray((m.width*m.height,3),dtype='<f4',buffer=bytes(m.data),strides=(m.point_step,4))
 q=t.rotation;u=np.array([q.x,q.y,q.z]); p=a+2*np.cross(u,np.cross(u,a)+q.w*a)+[t.translation.x,t.translation.y,t.translation.z]
 frames.append(np.column_stack((np.full(len(p),len(frames)),p)));last=now
n.create_subscription(PointCloud2,'/cloud_registered_body_1',cb,qos_profile_sensor_data)
end=time.monotonic()+16
while len(frames)<12 and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
assert len(frames)>=10
out=io.BytesIO();np.save(out,np.concatenate(frames));sys.stdout.buffer.write(out.getvalue())
