#!/usr/bin/python3
"""Immediate native PositionCommand -> RotorS conversion; no shaping or limits."""
import copy, json, math, threading
import rospy
from geometry_msgs.msg import Transform, Twist, PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

class Bridge:
    def __init__(self):
        self.lock=threading.RLock();self.mode='WAIT_PREPARE';self.odom=None;self.target=[0.,0.,1.];self.yaw=0.;self.count=0
        self.out=rospy.Publisher('/hummingbird/command/trajectory',MultiDOFJointTrajectory,queue_size=50)
        self.trigger=rospy.Publisher('/native/trigger',PoseStamped,queue_size=1)
        self.status=rospy.Publisher('/native/bridge_status',String,queue_size=1,latch=True)
        rospy.Subscriber('/hummingbird/odometry_sensor1/odometry',Odometry,self.on_odom,queue_size=20)
        rospy.Subscriber('/planning/pos_cmd',PositionCommand,self.command,queue_size=100)
        rospy.Service('/native/prepare',Trigger,self.prepare);rospy.Service('/native/start',Trigger,self.start);rospy.Service('/native/stop',Trigger,self.stop)
        rospy.Timer(rospy.Duration(.01),self.tick);rospy.Timer(rospy.Duration(.2),self.report)
    def on_odom(self,m): self.odom=m
    def send(self,p,v,a,yaw,yawdot,stamp):
        if not all(math.isfinite(float(x)) for x in [*p,*v,*a,yaw,yawdot]):
            rospy.logerr('Non-finite native command; ignored');return
        msg=MultiDOFJointTrajectory();msg.header.stamp=stamp;msg.header.frame_id='world';msg.joint_names=['base_link']
        pt=MultiDOFJointTrajectoryPoint();t=Transform();vel=Twist();acc=Twist()
        t.translation.x,t.translation.y,t.translation.z=p;t.rotation.z=math.sin(yaw/2);t.rotation.w=math.cos(yaw/2)
        vel.linear.x,vel.linear.y,vel.linear.z=v;vel.angular.z=yawdot;acc.linear.x,acc.linear.y,acc.linear.z=a
        pt.transforms=[t];pt.velocities=[vel];pt.accelerations=[acc];pt.time_from_start=rospy.Duration(0);msg.points=[pt];self.out.publish(msg)
    def command(self,m):
        with self.lock:
            if self.mode!='RUNNING':return
            xyz=lambda q:[q.x,q.y,q.z]
            self.send(xyz(m.position),xyz(m.velocity),xyz(m.acceleration),m.yaw,m.yaw_dot,m.header.stamp);self.count+=1
    def prepare(self,_):
        with self.lock:
            if self.odom is None:return TriggerResponse(False,'No measured odometry')
            if self.mode!='WAIT_PREPARE':return TriggerResponse(False,self.mode)
            self.mode='PREPARING';return TriggerResponse(True,'Takeoff to world (0,0,1); separate from exploration')
    def start(self,_):
        with self.lock:
            if self.mode!='READY':return TriggerResponse(False,'Prepare and settle first: '+self.mode)
            if self.trigger.get_num_connections()<1:return TriggerResponse(False,'Planner trigger subscriber missing')
            self.mode='RUNNING';m=PoseStamped();m.header.stamp=rospy.Time.now();m.header.frame_id='world';m.pose=copy.deepcopy(self.odom.pose.pose);self.trigger.publish(m)
            return TriggerResponse(True,'Single explicit trigger sent; acceptance must be read from official FSM log')
    def stop(self,_):
        with self.lock:
            if self.odom:
                p=self.odom.pose.pose.position;self.target=[p.x,p.y,p.z];q=self.odom.pose.pose.orientation
                self.yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.mode='STOPPED';return TriggerResponse(True,'Explicit measured-pose hover; this is not planner FINISH')
    def tick(self,_):
        with self.lock:
            if self.mode in ('PREPARING','READY','STOPPED') or (self.mode=='RUNNING' and self.count==0):
                self.send(self.target,[0.]*3,[0.]*3,self.yaw,0.,rospy.Time.now())
            if self.mode=='PREPARING' and self.odom:
                p=self.odom.pose.pose.position;v=self.odom.twist.twist.linear
                if math.dist([p.x,p.y,p.z],self.target)<.1 and math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)<.1:self.mode='READY'
    def report(self,_):self.status.publish(String(data=json.dumps({'mode':self.mode,'converted_commands':self.count})))
if __name__=='__main__':
    rospy.init_node('native_command');Bridge();rospy.spin()
