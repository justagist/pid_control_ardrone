from std_msgs.msg import Bool
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from Drone import Drone
from ardrone_joy.msg import AutoPilotCmd
import math
import time
import tf
import csv

class pidcontroller:
    


    def __init__(self,kp=(0.5,0.5,0.5,0.5),kd=(0.1,0.1,0.1,0.0025),ki=(0.0,0.0,0.0,0.0)):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.drone = Drone()
        self.breaker = True
        self.vel = [0.0,0.0,0.0,0.0]
        self.prev_time = [0.0,0.0,0.0,0.0]
        self.prev_error = [0.0,0.0,0.0,0.0]
        self.e_i = [0.0,0.0,0.0,0.0]
        self.speed = 5
        self.repeat_pilot = False
        self.single_point_error = 0.01
        self.multiple_point_error = 0.1
        self.ang_error = 0.02
        self.length = 1
        
    def start_autopilot(self,autopilot_data):
        self.breaker = autopilot_data.autopilot_breaker_state
        self.count_loop = 0
        self.reach_pose = False
        listener = tf.TransformListener()
        arrA = []
        arrB = []
        t = time.time()
        rate = rospy.Rate(50)
        listener.waitForTransform("nav", "base_stabilized", rospy.Time(), rospy.Duration(1.0))
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "nav"
        goal_pose.pose.position.x = autopilot_data.newpos[0]
        goal_pose.pose.position.y = autopilot_data.newpos[1]
        goal_pose.pose.position.z = autopilot_data.newpos[2]
        target_orient = autopilot_data.neworientation[:3]
        if autopilot_data.no_of_points > 1:
            self.repeat_pilot = True
            self.error_tol = self.multiple_point_error
            self.new_autopilot_newpos = autopilot_data.newpos[3:]
            self.new_autopilot_neworient = autopilot_data.neworientation[3:]
            self.points_left = len(self.new_autopilot_newpos)/3
            if autopilot_data.no_of_points > self.length:
                self.list_of_points = autopilot_data.newpos
                self.list_of_orients = autopilot_data.neworientation
                self.length = autopilot_data.no_of_points
        elif autopilot_data.no_of_points == 1:
            self.error_tol = self.single_point_error
            if self.repeat_pilot:
                self.new_autopilot_newpos = self.list_of_points
                self.new_autopilot_neworient = self.list_of_orients
                self.points_left = len(self.list_of_points)/3
                self.error_tol = self.multiple_point_error
        else:
            print 'Invalid no. of points given!'
        print 'point:',self.length - autopilot_data.no_of_points+1,'/',self.length
        print (goal_pose.pose.position.x ,goal_pose.pose.position.y ,goal_pose.pose.position.z),target_orient[2]*180/math.pi
        while not self.breaker:
            try:
                rospy.Subscriber('/autopilot_stop_cmd',AutoPilotCmd,self.checkbreak)
                listener.waitForTransform("nav", "base_stabilized", rospy.Time(), rospy.Duration(1.0))
                (trans,rot) = listener.lookupTransform("nav", "base_stabilized", rospy.Time())
                self.curr_global_pose = trans
                goal_pose_n = goal_pose
                goal_pose_n.header.stamp = listener.getLatestCommonTime("/base_stabilized",goal_pose_n.header.frame_id)
                pose_in_loc_frame = listener.transformPose("base_stabilized", goal_pose_n)
                #print pose_in_loc_frame
                target_point = pose_in_loc_frame.pose.position
                #print rot
                if autopilot_data.autopilot_mode == 'move+turn':
                    self.moveturn(target_point,target_orient,rot)
                elif autopilot_data.autopilot_mode == 'no_turn':
                    self.no_turn(target_point)
                elif autopilot_data.autopilot_mode == 'reach&turn':
                    self.reach_turn(target_point,target_orient,rot)
                elif autopilot_data.autopilot_mode == 'turn&move':
                    self.turn_move(target_point,target_orient,rot)  
                else:
                    print 'Invalid or No Autopilot Mode Specified!'
                t1 = time.time()-t
                p = target_point.x
                arrA.append(t1)
                arrB.append(p)
                if self.reach_pose:
                    break
            except (tf.LookupException, tf.ConnectivityException):
                continue
            rate.sleep()
        if self.repeat_pilot == True and self.reach_pose == True and self.breaker == False:
            self.nextpoint(self.new_autopilot_newpos,self.new_autopilot_neworient,self.points_left,autopilot_data.autopilot_mode)
        else:
            self.breaker = True
            self.repeat_pilot = False
            self.reach_pose = False
            print 'Navigation Done!'
            with open('expt_x_sim.csv','w') as f: 
                writer = csv.writer(f, delimiter='\t')
                writer.writerows(zip(arrA,arrB))
            
    def checkbreak(self,val):
        self.breaker = val.autopilot_breaker_state
                    
    def calc_error(self,error,i):
        curr_time = time.time()
        dt = 0.0
        if self.prev_time[i] != 0.0:
            dt = curr_time - self.prev_time[i]
        de = error - self.prev_error[i]
        
        e_p = self.kp[i] * error
        self.e_i[i] += error * dt
        e_d = 0
        if dt > 0:
            e_d = de/dt
        self.prev_time[i] = curr_time
        self.prev_error[i] = error
        #print i,':',e_p,(self.kd[i]*e_d),(self.ki[i]*self.e_i[i])
        return e_p + (self.ki[i]*self.e_i[i]) + (self.kd[i]*e_d)
        
    def no_turn(self,targ_pos):
        if not self.breaker:
            errors = (targ_pos.x,targ_pos.y,targ_pos.z)
            for i in range(len(errors)):
                self.vel[i] = self.calc_error(errors[i],i)
            if max(errors) > self.error_tol or min(errors) < -self.error_tol:
                self.drone.moveLinear(self.speed*self.vel[0],0,0)
            else:
                self.reach_pose = True
                print 'reached destination coordinates'
                
    def reach_turn(self,targ_pos,targ_orient,curr_orient):
        if not self.breaker:
            yaw = math.atan2(2 * (curr_orient[0] * curr_orient[1] + curr_orient[3] * curr_orient[2]),curr_orient[3] * curr_orient[3] + curr_orient[0] * curr_orient[0] - curr_orient[1] * curr_orient[1] - curr_orient[2] * curr_orient[2])
            #print yaw*180/math.pi
            errors = (targ_pos.x,targ_pos.y,targ_pos.z,targ_orient[2]-yaw)
            for i in range(len(errors)):
                self.vel[i] = self.calc_error(errors[i],i)
            if max(errors[:3]) > self.error_tol or min(errors[:3]) < -self.error_tol:
                self.drone.moveLinear(self.speed*self.vel[0],self.speed*self.vel[1],self.speed*self.vel[2])
            else:
                if abs(errors[3]) > self.ang_error:
                    self.drone.moveAngular(0,0,self.speed*self.vel[3])
                    print 'Reached position. Correcting orientation...'
                else:
                    self.reach_pose = True
                    print 'Reached destination pose'
                    
    def turn_move(self,targ_pos,targ_orient,curr_orient):
        if not self.breaker:
            yaw = math.atan2(2 * (curr_orient[0] * curr_orient[1] + curr_orient[3] * curr_orient[2]),curr_orient[3] * curr_orient[3] + curr_orient[0] * curr_orient[0] - curr_orient[1] * curr_orient[1] - curr_orient[2] * curr_orient[2])
            #print yaw*180/math.pi
            errors = (targ_pos.x,targ_pos.y,targ_pos.z,targ_orient[2]-yaw)
            for i in range(len(errors)):
                self.vel[i] = self.calc_error(errors[i],i)
            if abs(errors[3]) > self.ang_error:
                self.drone.moveAngular(0,0,self.speed*self.vel[3])
            else:
                if max(errors[:3]) > self.error_tol or min(errors[:3]) < -self.error_tol:
                    self.drone.moveLinear(self.speed*self.vel[0],self.speed*self.vel[1],self.speed*self.vel[2])
                    print 'Corrected Orientation. Moving to destination...'
        
    def moveturn(self,targ_pos,targ_orient,curr_orient):
        if not self.breaker:
            yaw = math.atan2(2 * (curr_orient[0] * curr_orient[1] + curr_orient[3] * curr_orient[2]),curr_orient[3] * curr_orient[3] + curr_orient[0] * curr_orient[0] - curr_orient[1] * curr_orient[1] - curr_orient[2] * curr_orient[2])
            #print yaw*180/math.pi
            errors = (targ_pos.x,targ_pos.y,targ_pos.z,targ_orient[2]-yaw)
            for i in range(len(errors)):
                self.vel[i] = self.calc_error(errors[i],i)
            if max(errors[:3]) > self.error_tol or min(errors[:3]) < -self.error_tol:
                if self.count_loop % 2 == 0 or abs(errors[3]) < self.ang_error:
                    self.drone.moveLinear(self.speed*self.vel[0],self.speed*self.vel[1],self.speed*self.vel[2])
                    #print 'error:',errors,max(errors),min(errors),self.error_tol#,'moving to',self.newpoint
                #print 'global coordinates: ',self.curr_global_pose
                #print 'velocities',self.lin_vel
                else:
                    self.drone.moveAngular(0,0,self.speed*self.vel[3])
                self.count_loop+=1
            else:
                self.reach_pose = True
                print 'reached destination pose!'
                
    def nextpoint(self,newpos,neworient,points_left,mode):
        loop_data = AutoPilotCmd(False,newpos,neworient,points_left,mode)
        self.start_autopilot(loop_data)
        print 'going to next point'
            
                
def listener(pd):
    rospy.init_node('mover', anonymous=True)
    rospy.Subscriber('/autopilot_start_cmd',AutoPilotCmd,pd.start_autopilot)
    print '>>Started PID Controller for Simulator...\n\tSinglePose Error tol:',pd.single_point_error,'\n\tMultiPose Error tol:',pd.multiple_point_error
    print '\tSpeed:',pd.speed
    print 'PID terms:',pd.kp,pd.kd,pd.ki
    rospy.spin()
    
if __name__ == '__main__':
    pd = pidcontroller()
    listener(pd)
