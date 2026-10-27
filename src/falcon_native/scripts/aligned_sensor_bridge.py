"""Official-compatible input: independent measured transforms, no per-image pairing."""
import copy,json,threading,time,xmlrpc.client
from collections import Counter
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from sensor_math import camera_transform,quat_matrix,quat_product
from tf.transformations import quaternion_from_matrix
from depth_semantics import convert_depth

class AlignedBridge:
    def __init__(self):
        if rospy.get_param('/native/camera_depth_mode')!='official_compatible':raise ValueError('Camera/bridge mode mismatch')
        self.lock=threading.RLock();self.latest=None;self.info=None;self.metadata=None;self.consumer_connected=False
        self.anchor_ns=None;self.last_pose_ns=None;self.calibrated=False;self.counts=Counter();self.skips=Counter();self.categories=Counter();self.streams={};self.last_depth={}
        self.tf=camera_transform(rospy.get_param('/hummingbird/robot_description'),'hummingbird/odometry_sensor1_link','hummingbird/fuel_official_camera_optical_link');self.q=quaternion_from_matrix(self.tf)
        self.odom_pub=rospy.Publisher('/native/odom_world',Odometry,queue_size=50)
        self.pose_pub=rospy.Publisher('/native/sensor_pose',TransformStamped,queue_size=100)
        self.depth_pub=rospy.Publisher('/native/depth',Image,queue_size=8)
        self.status=rospy.Publisher('/native/sensor_status',String,queue_size=1,latch=True)
        self.subs=[rospy.Subscriber('/hummingbird/odometry_sensor1/odometry',Odometry,self.odom,queue_size=200),
            rospy.Subscriber('/hummingbird/vi_sensor/camera_depth/depth/camera_info',CameraInfo,self.camera,queue_size=2),
            rospy.Subscriber('/hummingbird/vi_sensor/camera_depth/camera/camera_info',CameraInfo,lambda _:None,queue_size=1),
            rospy.Subscriber('/native/observations/depth_metadata',String,self.meta,queue_size=1),
            rospy.Subscriber('/native/observations/depth_raw',Image,self.depth,queue_size=8,buff_size=16000000)]
        self.timers=[rospy.Timer(rospy.Duration(1./30),self.pose_tick),rospy.Timer(rospy.Duration(.1),self.connection_tick),rospy.Timer(rospy.Duration(.5),self.report)]
    def track(self,name,m):
        s=self.streams.setdefault(name,{'count':0,'stamps':set(),'duplicates':0,'last_seq':None,'sequence_gaps':0,'out_of_order_seq':0})
        stamp=m.header.stamp.to_nsec();s['count']+=1
        if stamp in s['stamps']:s['duplicates']+=1
        s['stamps'].add(stamp)
        if s['last_seq'] is not None:
            delta=int(m.header.seq)-s['last_seq']
            if delta>1:s['sequence_gaps']+=delta-1
            elif delta<0:s['out_of_order_seq']+=1
        s['last_seq']=int(m.header.seq)
    def ready(self):
        if not self.calibrated and self.info is not None and self.metadata is not None and self.latest is not None:
            rospy.set_param('/native/calibrated',True);self.calibrated=True
    def camera(self,m):
        with self.lock:
            expected=np.array([320,0,320,0,320,240,0,0,1],float)
            if m.width!=640 or m.height!=480 or not np.allclose(m.K,expected,atol=1e-8,rtol=0) or any(d!=0 for d in m.D):
                rospy.signal_shutdown('Actual CameraInfo does not match fixed official camera');return
            if not np.allclose(m.P,[320,0,320,0,0,320,240,0,0,0,1,0],atol=1e-8,rtol=0):
                rospy.signal_shutdown('Actual CameraInfo P is inconsistent');return
            # Check loaded targets; NEVER overwrite official planner calibration/FOV.
            for k,index in [('fx',0),('fy',4),('cx',2),('cy',5)]:
                if abs(rospy.get_param('/uav_model/sensing_parameters/camera_intrinsics/'+k)-m.K[index])>1e-8:raise ValueError('Planner camera mismatch '+k)
            self.info={'width':m.width,'height':m.height,'K':list(m.K),'P':list(m.P),'D':list(m.D),'frame':m.header.frame_id}
            self.ready()
    def meta(self,m):
        with self.lock:
            a=json.loads(m.data)
            if a.get('near')!=.1 or a.get('far')!=500 or a.get('source')!='DepthCamera::ConnectNewDepthFrame' or a.get('frame_id')!='hummingbird/fuel_official_camera_optical_link':
                rospy.signal_shutdown('Actual raw camera metadata mismatch');return
            self.metadata=a;self.ready()
    def odom(self,m):
        with self.lock:
            self.track('odom_in',m);self.latest=m
            out=copy.deepcopy(m);q=m.pose.pose.orientation;v=m.twist.twist.linear
            world_v=quat_matrix([q.x,q.y,q.z,q.w])@np.array([v.x,v.y,v.z])
            out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,world_v)
            out.child_frame_id='world';self.odom_pub.publish(out);self.counts['odom_output']+=1
            self.ready()
    def connection_tick(self,_):
        if self.consumer_connected:return
        try:
            bus=xmlrpc.client.ServerProxy(rospy.get_node_uri()).getBusInfo('/native_sensor')[2]
            # Check the actual planner TCP connection, not merely a recorder.
            self.consumer_connected=any(x[1]=='/exploration_node' and x[2]=='o' and x[4]=='/native/sensor_pose' and x[5] for x in bus)
        except Exception:self.counts['startup_bus_query_error']+=1
    def pose_tick(self,_):
        with self.lock:
            self.counts['pose_timer_callbacks']+=1
            if self.latest is None:self.counts['pose_without_odom']+=1;return
            m=self.latest;p=m.pose.pose.position;q=m.pose.pose.orientation;qa=np.array([q.x,q.y,q.z,q.w]);position=np.array([p.x,p.y,p.z])+quat_matrix(qa)@self.tf[:3,3];orientation=quat_product(qa,self.q)
            out=TransformStamped();out.header=copy.copy(m.header);out.header.frame_id='world';out.child_frame_id='hummingbird/fuel_official_camera_optical_link'
            out.transform.translation.x,out.transform.translation.y,out.transform.translation.z=map(float,position)
            out.transform.rotation.x,out.transform.rotation.y,out.transform.rotation.z,out.transform.rotation.w=map(float,orientation)
            self.pose_pub.publish(out);self.counts['transform_output']+=1
            ns=m.header.stamp.to_nsec()
            if self.last_pose_ns==ns:self.counts['transform_reused_measurement']+=1
            self.last_pose_ns=ns
            if self.consumer_connected and self.anchor_ns is None:self.anchor_ns=ns
    def depth(self,m):
        with self.lock:
            self.track('raw_in',m)
            if self.info is None or self.metadata is None:self.skips['startup_calibration_or_metadata']+=1;return
            if self.anchor_ns is None:self.skips['startup_no_planner_transform_connection']+=1;return
            if m.header.stamp.to_nsec()<self.anchor_ns:self.skips['startup_before_first_delivered_transform']+=1;return
            if m.encoding!='32FC1' or m.width!=640 or m.height!=480 or m.header.frame_id!=self.metadata['frame_id']:
                self.skips['unsupported_image_contract']+=1;return
            d=np.ndarray((m.height,m.width),dtype='>f4' if m.is_bigendian else '<f4',buffer=m.data,strides=(m.step,4))
            values,categories=convert_depth(d,'official_compatible',near=self.metadata['near'],far=self.metadata['far']);self.categories.update(categories)
            out=copy.copy(m);out.is_bigendian=False;out.step=m.width*4;out.data=values.astype('<f4').tobytes()
            # No pose interpolation, pending image, span threshold or ongoing gate.
            self.depth_pub.publish(out);self.counts['depth_output']+=1
            self.last_depth={'stamp_ns':m.header.stamp.to_nsec(),'categories':categories}
    def report(self,_):
        with self.lock:
            streams={k:dict({n:v for n,v in x.items() if n!='stamps'},unique_stamps=len(x['stamps'])) for k,x in self.streams.items()}
            self.status.publish(String(data=json.dumps({'depth_mode':'official_compatible','input_topic':'/native/observations/depth_raw','pose_type':'geometry_msgs/TransformStamped','pose_nominal_hz':30,'counts':dict(self.counts),'startup_skips':dict(self.skips),'streams':streams,'queue_sizes':{'raw_subscriber':8,'depth_publisher':8,'odom_subscriber':200,'transform_publisher':100},'queue_drop_observation':'source sequence gaps are recorded; internal rospy/transport drops are not independently distinguishable','first_planner_transform_stamp_ns':self.anchor_ns,'planner_transform_connected':self.consumer_connected,'calibration':self.info,'raw_metadata':self.metadata,'category_totals':dict(self.categories),'last_depth':self.last_depth,'T_odom_optical':self.tf.tolist()})))
