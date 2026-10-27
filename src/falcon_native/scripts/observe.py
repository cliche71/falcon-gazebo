#!/usr/bin/python3
"""Independent passive evidence recorder; never sends flight commands."""
import json, threading, time, sys, re
from pathlib import Path
import numpy as np
import rospy, rosbag
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import String, Float32, Int32
from rosgraph_msgs.msg import Log, Clock
from trajectory.msg import Bspline
from quadrotor_msgs.msg import PositionCommand
from trajectory_msgs.msg import MultiDOFJointTrajectory
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

class Observer:
    def __init__(self,path):
        self.path=Path(path); self.lock=threading.RLock();self.files={};self.last={};self.latest={};self.counts={};self.state='UNKNOWN'
        self.cloud_subs={}
        self.bag=rosbag.Bag(str(self.path/'evidence.bag'),'w',compression='lz4')
        specs=[('/hummingbird/odometry_sensor1/odometry',Odometry),('/native/odom_world',Odometry),('/planning/pos_cmd',PositionCommand),
            ('/hummingbird/command/trajectory',MultiDOFJointTrajectory),('/planning/bspline',Bspline),('/planning/replan',Int32),
            ('/rosout_agg',Log),('/native/sensor_status',String),('/native/bridge_status',String),('/native/trigger',PoseStamped),
            ('/voxel_mapping/map_coverage',Float32),('/native/diagnostics/contacts',String),('/native/diagnostics/collision_registry',String),
            ('/clock',Clock),('/hummingbird/vi_sensor/camera_depth/depth/camera_info',CameraInfo)]
        for topic,typ in specs:rospy.Subscriber(topic,typ,self.receive,callback_args=topic,queue_size=500,buff_size=4000000)
        for topic in ['/planning_vis/frontier_pcl','/planning_vis/dormant_frontier_pcl','/planning_vis/tiny_frontier_pcl','/voxel_mapping/occupancy_grid_free','/voxel_mapping/occupancy_grid_unknown','/voxel_mapping/occupancy_grid_occupied','/voxel_mapping/tsdf','/voxel_mapping/esdf']:
            self.cloud_subs[topic]=rospy.Subscriber(topic,PointCloud2,self.cloud,callback_args=topic,queue_size=1,buff_size=10000000)
        rospy.Subscriber('/planning_vis/frontier',Marker,self.marker,queue_size=100)
        rospy.Subscriber('/planning_vis/hgrid',Marker,self.marker,queue_size=100)
        rospy.Subscriber('/hummingbird/vi_sensor/camera_depth/depth/disparity',Image,self.depth,queue_size=1,buff_size=4000000)
        rospy.Timer(rospy.Duration(1),self.status);rospy.on_shutdown(self.close)
    def line(self,key,data):
        if key not in self.files:self.files[key]=(self.path/(key+'.jsonl')).open('a',buffering=1)
        self.files[key].write(json.dumps(data,allow_nan=True)+'\n')
    def receive(self,m,t):
        with self.lock:
            now=rospy.Time.now(); wall=time.time(); self.counts[t]=self.counts.get(t,0)+1
            # Calibration repeats at camera rate; retain the first and at one minute intervals.
            if isinstance(m,CameraInfo) and wall-self.last.get(t,0)<60:return
            self.last[t]=wall;self.bag.write(t,m,now)
            if isinstance(m,Log):
                self.line('rosout',{'sim':m.header.stamp.to_sec(),'wall':wall,'node':m.name,'level':m.level,'text':m.msg})
                s=re.search(r'Transit state from (\w+) to (\w+) by (\w+)',m.msg)
                if s:
                    self.state=s[2];self.line('fsm',{'sim':m.header.stamp.to_sec(),'wall':wall,'from':s[1],'state':s[2],'caller':s[3]})
            if isinstance(m,String):
                try:d=json.loads(m.data)
                except ValueError:d={'text':m.data}
                self.latest[t]=d;self.line(t.strip('/').replace('/','_'),{'sim':now.to_sec(),'wall':wall,'data':d})
            if isinstance(m,Float32):self.latest[t]=m.data
            if isinstance(m,Bspline):self.line('routes',{'sim':now.to_sec(),'wall':wall,'id':m.traj_id,'start':m.start_time.to_sec(),'knots':list(m.knots),'position_points':[[p.x,p.y,p.z] for p in m.pos_pts],'yaw_pts':list(m.yaw_pts),'yaw_dt':m.yaw_dt})
    def cloud(self,m,t):
        with self.lock:
            wall=time.time()
            if wall-self.last.get(t,0)<1:return
            self.last[t]=wall; n=m.width*m.height
            d={'sim':rospy.Time.now().to_sec(),'wall':wall,'points':n,'fields':[x.name for x in m.fields]}
            self.latest[t]=d;self.line('cloud_counts',dict(topic=t,**d))
            # Snapshot each cloud every 10 simulation seconds for reproducible map/cell inspection.
            now=rospy.Time.now()
            if now.to_sec()-self.last.get(t+'_bag',-100)>10:
                self.last[t+'_bag']=now.to_sec();self.bag.write(t,m,now)
            # Always replace a single latest copy (online map only, no scene truth).
            (self.path/(t.strip('/').replace('/','_')+'.latest.bin')).write_bytes(m.data)
            (self.path/(t.strip('/').replace('/','_')+'.latest.json')).write_text(json.dumps(dict(d,point_step=m.point_step,width=m.width,height=m.height,fields=[{'name':x.name,'offset':x.offset,'datatype':x.datatype} for x in m.fields])))
            if t in ['/voxel_mapping/esdf','/voxel_mapping/occupancy_grid_unknown']:
                self.cloud_subs[t].unregister()  # One initial snapshot; avoid recurring full-volume visualization cost.
    def marker(self,m):
        if m.action==m.DELETEALL:return
        # Official marker names and IDs are kept as supplemental route/frontier evidence.
        with self.lock:
            self.line('markers',{'sim':rospy.Time.now().to_sec(),'ns':m.ns,'id':m.id,'action':m.action,'points':len(m.points),'text':m.text})
    def depth(self,m):
        with self.lock:
            now=rospy.Time.now().to_sec()
            if now-self.last.get('depth_sample',-100)<30:return
            self.last['depth_sample']=now
            a=np.ndarray((m.height,m.width),dtype='>f4' if m.is_bigendian else '<f4',buffer=m.data,strides=(m.step,4))
            np.savez_compressed(self.path/('depth_%.3f.npz'%m.header.stamp.to_sec()),depth=a,stamp=m.header.stamp.to_sec(),frame=m.header.frame_id)
    def status(self,_):
        with self.lock:
            s={'sim':rospy.Time.now().to_sec(),'wall':time.time(),'state_from_official_rosout':self.state,'counts':self.counts,'latest':self.latest}
            (self.path/'live_status.json').write_text(json.dumps(s,indent=2));self.line('status',s)
    def close(self):
        with self.lock:
            self.bag.close()
            for f in self.files.values():f.close()
if __name__=='__main__':
    rospy.init_node('native_observer'); Observer(sys.argv[1]); rospy.spin()
