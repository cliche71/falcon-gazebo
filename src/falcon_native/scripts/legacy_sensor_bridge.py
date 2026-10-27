#!/usr/bin/python3
"""Measured Gazebo input; selectable official depth semantics, no map changes."""
import copy, json, math, threading, time
from collections import deque
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_math import camera_transform, quat_matrix, quat_product, interpolate
from tf.transformations import quaternion_from_matrix
from depth_semantics import MODES, convert_depth

class Bridge:
    def __init__(self):
        self.lock=threading.RLock(); self.samples=deque(maxlen=500); self.pending=None
        self.pairs=0; self.info=None; self.stats={}; self.last_pair=0; self.category_totals={}
        self.mode=rospy.get_param('~depth_mode','legacy'); self.raw_metadata=None
        if self.mode not in MODES: raise ValueError('Unsupported depth mode: '+self.mode)
        camera_mode=rospy.get_param('/native/camera_depth_mode','legacy')
        if camera_mode!=self.mode: raise ValueError('Camera and sensor depth modes differ')
        self.input_topic=('/native/observations/depth_raw' if self.mode=='official_compatible'
            else '/hummingbird/vi_sensor/camera_depth/depth/disparity')
        self.tf=camera_transform(rospy.get_param('/hummingbird/robot_description'),
            'hummingbird/odometry_sensor1_link','hummingbird/fuel_official_camera_optical_link')
        self.q=quaternion_from_matrix(self.tf)
        self.odom_pub=rospy.Publisher('/native/odom_world',Odometry,queue_size=50)
        self.pose_pub=rospy.Publisher('/native/sensor_pose',PoseStamped,queue_size=5)
        self.depth_pub=rospy.Publisher('/native/depth',Image,queue_size=2)
        self.status=rospy.Publisher('/native/sensor_status',String,queue_size=1,latch=True)
        rospy.Subscriber('/hummingbird/odometry_sensor1/odometry',Odometry,self.odom,queue_size=100)
        rospy.Subscriber('/hummingbird/vi_sensor/camera_depth/depth/camera_info',CameraInfo,self.camera,queue_size=1)
        if self.mode=='official_compatible':
            rospy.Subscriber('/native/observations/depth_metadata',String,self.metadata,queue_size=1)
            # OpenNI's depth CameraInfo alone does not keep its sensor active.
            # The same camera's standard CameraInfo uses ImageConnect; this
            # retains acquisition without a second depth subscription or input.
            self.camera_keepalive=rospy.Subscriber(
                '/hummingbird/vi_sensor/camera_depth/camera/camera_info',CameraInfo,
                lambda _:None,queue_size=1)
        rospy.Subscriber(self.input_topic,Image,self.depth,queue_size=1,buff_size=4000000)
        rospy.Timer(rospy.Duration(.5),self.report)
    def metadata(self,m):
        with self.lock:
            data=json.loads(m.data)
            if (data.get('near')!=0.35 or data.get('far')!=500.0 or
                data.get('source')!='DepthCamera::ConnectNewDepthFrame' or
                data.get('frame_id')!='hummingbird/fuel_official_camera_optical_link'):
                rospy.signal_shutdown('Unsupported actual raw camera metadata'); return
            self.raw_metadata=data
    def camera(self,m):
        with self.lock:
            if self.info: return
            if m.K[0]<=0 or m.K[4]<=0 or any(abs(d)>1e-9 for d in m.D):
                rospy.logerr('Unsupported CameraInfo'); return
            self.info={'width':m.width,'height':m.height,'K':list(m.K),'D':list(m.D),'frame':m.header.frame_id}
            fx,fy,cx,cy=m.K[0],m.K[4],m.K[2],m.K[5]
            for k,v in dict(fx=fx,fy=fy,cx=cx,cy=cy).items():
                rospy.set_param('/uav_model/sensing_parameters/camera_intrinsics/'+k,float(v))
            rospy.set_param('/uav_model/sensing_parameters/image_width',m.width)
            rospy.set_param('/uav_model/sensing_parameters/image_height',m.height)
            left,right=math.atan(cx/fx),math.atan((m.width-cx)/fx)
            top=min(math.atan(cy/fy),math.atan((m.height-cy)/fy))
            for k,v in dict(left_angle=left,right_angle=right,top_angle=top).items(): rospy.set_param('/perception_utils/'+k,v)
            rospy.set_param('/uav_model/sensing_parameters/fov/horizontal',left+right)
            rospy.set_param('/uav_model/sensing_parameters/fov/vertical',2*top)
            rospy.set_param('/native/calibrated',True)
    def odom(self,m):
        with self.lock:
            p,q=m.pose.pose.position,m.pose.pose.orientation; a=np.array([q.x,q.y,q.z,q.w]); r=quat_matrix(a)
            self.samples.append((m.header.stamp.to_sec(),np.array([p.x,p.y,p.z]),a))
            out=copy.deepcopy(m); v=m.twist.twist.linear
            v=r@np.array([v.x,v.y,v.z]); out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,v)
            # FALCON consumes world-frame velocity, while the RotorS controller uses raw body-frame odometry.
            out.child_frame_id='world'; self.odom_pub.publish(out); self.pair()
    def depth(self,m):
        with self.lock:
            if m.encoding!='32FC1': rospy.logerr_throttle(2,'Expected meter 32FC1, received '+m.encoding); return
            if self.mode=='official_compatible':
                if self.raw_metadata is None: return
                if (m.header.frame_id!=self.raw_metadata['frame_id'] or
                    m.width!=self.raw_metadata['width'] or m.height!=self.raw_metadata['height']):
                    rospy.logerr('Raw image does not match camera metadata'); return
            d=np.ndarray((m.height,m.width),dtype='>f4' if m.is_bigendian else '<f4',buffer=m.data,strides=(m.step,4))
            valid=np.isfinite(d)&(d>0)
            self.stats={'encoding':m.encoding,'unit':'meters','stamp':m.header.stamp.to_sec(),'valid':int(valid.sum()),
                'nan':int(np.isnan(d).sum()),'infinity':int(np.isinf(d).sum()),'zero':int((d==0).sum()),
                'min':float(d[valid].min()) if valid.any() else None,'max':float(d[valid].max()) if valid.any() else None}
            values,categories=convert_depth(d,self.mode,
                near=self.raw_metadata['near'] if self.raw_metadata else .35,
                far=self.raw_metadata['far'] if self.raw_metadata else 500.)
            self.stats['categories']=categories
            for key,count in categories.items(): self.category_totals[key]=self.category_totals.get(key,0)+count
            out=copy.copy(m)
            out.is_bigendian=False; out.step=m.width*4; out.data=values.astype('<f4').tobytes()
            self.pending=out; self.pair()
    def pair(self):
        if self.pending is None or self.info is None: return
        m=self.pending; measured=interpolate(self.samples,m.header.stamp.to_sec(),max_gap=.04)
        if measured is None: return
        p,q=measured; p=p+quat_matrix(q)@self.tf[:3,3]; q=quat_product(q,self.q)
        pose=PoseStamped(); pose.header=copy.copy(m.header); pose.header.frame_id='world'
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,p)
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=map(float,q)
        self.pose_pub.publish(pose); self.depth_pub.publish(m); self.pending=None; self.pairs+=1; self.last_pair=time.monotonic()
    def report(self,_):
        with self.lock:
            self.status.publish(String(data=json.dumps({'pairs':self.pairs,'pair_age_wall':time.monotonic()-self.last_pair,
                'depth_mode':self.mode,'input_topic':self.input_topic,'raw_metadata':self.raw_metadata,
                'category_totals':self.category_totals,
                'calibration':self.info,'T_odom_optical':self.tf.tolist(),'depth':self.stats})))
if __name__=='__main__':
    rospy.init_node('native_sensor'); rospy.set_param('/native/calibrated',False); Bridge(); rospy.spin()
