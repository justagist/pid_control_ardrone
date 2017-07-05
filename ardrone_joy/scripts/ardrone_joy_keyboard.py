#!/usr/bin/env python

import rospy
import roslib; #roslib.load_manifest('ardrone_python')
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Twist, Vector3
from Drone import Drone
from ardrone_joy.msg import AutoPilotCmd
import math
import pygame
from nav_msgs.msg import Odometry


# Controls:
# Return - Take-off/stop autopilot
# Space - Land/stop autopilot
# W - move front
# S - move back
# A - move left
# D - move right
# LEFT - Turn left
# RIGHT - Turn right
# C - capture current position and store in memory
# V - go to capture point(s)
# X - clear all captured points
# L - print list of captured points
# M - Stop navigation and clear all captured points
# Keypad PLUS - increase keyboard control speed
# Keypad MINUS - decrease keyboard control speed
# Keypad 1 - plot a circular path
# Keypad 2 - plot a figure-8 path
# TAB - change navigation method

def main(drone):
  
    global navigate
    global record
    global point_array
    global orient_array
    global no_of_points
    global speed
    global mode
    pygame.init()
    W, H = 320, 240
    screen = pygame.display.set_mode((W, H))
    clock = pygame.time.Clock()
    running = True
    rate = rospy.Rate(100)
    record = False
    point_array = ()
    orient_array = ()
    no_of_points = 0
    print 'Return - Take-off/stop autopilot\nSpace - Land/stop autopilot\nW - move front\nS - move back\nA - move left\nD - move right\nLEFT - Turn left\nRIGHT - Turn right\nC - capture current position and store in memory\nV - go to capture point(s)\nX - clear all captured points\nL - print list of captured points\nM - Stop navigation and clear all captured points\nKeypad PLUS - increase keyboard control speed\nKeypad MINUS - decrease keyboard control speed\nKeypad 1 - plot a circular path\nKeypad 2 - plot a figure-8 path\nTAB - change navigation method'
    print '>>Manual Controller Started\n\tManual Control Speed:',speed,'\n\tAutopilot Mode:',mode
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                drone.land()
                running = False 
            elif event.type == pygame.KEYUP:
                drone.hover()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    drone.takeOff()
                    if navigate == True:
                        stopper = AutoPilotCmd(True,setpoint,setorient,1,mode)
                        stoppilotpub.publish(stopper)
                        navigate = False
                        print 'manually stopped autopilot'
                elif event.key == pygame.K_SPACE:
                    drone.land()
                    print 'landing'
                    if navigate == True:
                        stopper = AutoPilotCmd(True,setpoint,setorient,1,mode)
                        stoppilotpub.publish(stopper)
                        print 'manually stopped autopilot'
                        navigate = False
                elif event.key == pygame.K_w:
                    drone.moveLinear(0.1*speed,0,0)
                elif event.key == pygame.K_s:
                    drone.moveLinear(-0.1*speed,0,0)
                elif event.key == pygame.K_a:
                    drone.moveLinear(0,0.1*speed,0)
                elif event.key == pygame.K_d:
                    drone.moveLinear(0,-0.1*speed,0)
                elif event.key == pygame.K_UP:
                    drone.moveLinear(0,0,0.1*speed)
                elif event.key == pygame.K_DOWN:
                    drone.moveLinear(0,0,-0.1*speed)
                elif event.key == pygame.K_RIGHT:
                    drone.moveAngular(0,0,-0.2*speed)
                elif event.key == pygame.K_LEFT:
                    drone.moveAngular(0,0,speed*0.2)
                elif event.key == pygame.K_q:
                    if drone.hasTakenOff:
                        if navigate == False:
                            navigate = True
                            autopilot_start_p = AutoPilotCmd(False,(0.0,0.75,1.5),(0.0,0.0,-20*math.pi/180),1,mode)
                            startpilotpub.publish(autopilot_start_p)
                            print 'starting autopilot to coordinates',(0.0,0.75,1.5)
                        else:
                            print 'Cancel current navigation first'
                    else:
                        print 'Spinny has not taken off yet'
                elif event.key == pygame.K_p:
                    if drone.hasTakenOff:
                        if navigate == False:
                            navigate = True
                            autopilot_start_p = AutoPilotCmd(False,setpoint,setorient,1,mode)
                            startpilotpub.publish(autopilot_start_p)
                            print 'starting autopilot to coordinates',setpoint
                        else:
                            print 'Cancel current navigation first'
                    else:
                        print 'Spinny has not taken off yet'
                elif event.key == pygame.K_c:
                    if navigate == False:
                        record = False
                        rospy.Subscriber("odom",Odometry,capture_pos)
                        rospy.Subscriber("/ground_truth/state",Odometry,capture_pos_from_sim)              ####for simulator
                    else:       
                        print 'Cancel current navigation first'
                elif event.key == pygame.K_KP1:
                    if navigate == False:
                        record = False
                        # ----------- Change as required
                        r = 0.8
                        phi = math.pi/6
                        steps = 20
                        # ----------
                        rospy.Subscriber("odom",Odometry,circle_trajectory,(r,phi,steps))
                        rospy.Subscriber("/ground_truth/state",Odometry,circle_trajectory,(r,phi,steps))
                elif event.key == pygame.K_KP2:
                    if navigate == False:
                        record = False
                        steps = 20
                        rospy.Subscriber("odom",Odometry,figure8_trajectory,steps)
                        rospy.Subscriber("/ground_truth/state",Odometry,figure8_trajectory,steps)
                elif event.key == pygame.K_v:
                    if drone.hasTakenOff:
                        if navigate == False:
                            if record == True:
                                navigate = True
                                if no_of_points == 1:
                                    pointarray = (point_array[0],point_array[1]+1,point_array[2])        ##########
                                    autopilot_to_saved_point = AutoPilotCmd(False,pointarray,(0,0,30*math.pi/180),1,'turn&move') ############
                                    startpilotpub.publish(autopilot_to_saved_point)                               
                                    print point_array[1],pointarray[1] 
                                else:
                                    autopilot_to_saved_point = AutoPilotCmd(False,point_array,orient_array,no_of_points,mode)
                                    startpilotpub.publish(autopilot_to_saved_point) 
                                for i in range(1,no_of_points+1):
                                    n = 3*i-1
                            else:
                                print 'No coordinates recorded'
                        else:
                            print 'Cancel current navigation first'
                    else:
                        print 'Spinny has not taken off yet'
                elif event.key == pygame.K_b:
                    print 'navigation state',navigate
                    print 'no of captured points', no_of_points
                    print 'autopilot mode', mode
                elif event.key == pygame.K_l:
                    if no_of_points==0:
                        print 'No captured points'
                    else:
                        for i in range(0,len(point_array),3):
                            print (point_array[i],point_array[i+1],point_array[i+2]),(orient_array[i+2]*180/math.pi)
                elif event.key == pygame.K_x:
                    if record == True:
                        point_array = ()
                        orient_array = ()
                        no_of_points = 0
                        print 'Cleared all recorded coordinates'
                    else:
                        print 'No captured points'
                elif event.key == pygame.K_m:
                    if record == True:
                        point_array = ()
                        orient_array = ()
                        no_of_points = 0
                        stopper = AutoPilotCmd(True,setpoint,setorient,1,mode)
                        stoppilotpub.publish(stopper)
                        navigate = False
                        print 'Cleared all recorded coordinates and stopped navigations'
                elif event.key == pygame.K_KP_PLUS:
                    if speed <=6:
                        speed += 1
                        print 'Speed increased to:',speed
                    else:   
                        print 'At max speed!'
                elif event.key == pygame.K_KP_MINUS:
                    if speed >=1:
                        speed -= 1
                        print 'Speed decreased to:',speed
                    else:
                        print 'At min speed'
                elif event.key == pygame.K_TAB:
                    for i in range(len(mode_list)):
                        if mode == mode_list[i]:
                            if i == len(mode_list)-1:
                                mode = mode_list[0]
                            else:
                                mode = mode_list[i+1]
                            print 'Autopilot Mode:',mode
                            break

        pygame.display.flip()
        clock.tick(50)
        pygame.display.set_caption("FPS: %.2f" % clock.get_fps())
        rate.sleep()   
    stopper = AutoPilotCmd(True,setpoint,setorient,1,mode)
    if navigate == True:
        stoppilotpub.publish(stopper)
        navigate = False
        print 'stopping autopilot'   
    print 'shutting down...'

def circle_trajectory(data,(r,phi,steps)): # get destination points on an inclined circular trajectory with centre at current position of drone
    global record
    global point_array
    global orient_array
    global no_of_points
    if record == False:
        print phi
        point_array = ()
        orient_array = ()
        no_of_points = 0
        c = (data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z)
        # c = (0.7,0.7,0.7)
        print 'Creating circular trajectory with radius %f, centre (%f,%f,%f), and inclined at %f degrees to the horizontal'%(r,c[0],c[1],c[2],phi*180/math.pi)
        for step in range(steps):
            theta = 2*math.pi - (2*step*math.pi/steps) - math.pi
            x = r*math.cos(theta)*math.cos(phi)+c[0]
            y = r*math.sin(theta)*math.cos(phi)+c[1]
            z = r*math.cos(theta)*math.sin(phi)+c[2]
            yaw = theta-(math.pi)/2
            point_array+=((x,y,z))
            orient_array+=((0,0,yaw))
            no_of_points += 1
            print 'Position captured: (%f,%f,%f); Orientation: %f'%(x,y,z,yaw*180/math.pi)
        record = True

def figure8_trajectory(data,steps): # get destination points on a figure 8 trajectory with centre at current position of drone
    global record
    global point_array
    global orient_array
    global no_of_points
    if record == False:
        point_array = ()
        orient_array = ()
        no_of_points = 0
        c = (data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z)
        # c = (0.7,0.7,0.7)
        for i in range(steps):
            theta = i*((3*math.pi/2)+math.pi/2)/(steps)-math.pi
            x = 0.5*math.cos(theta) + c[0]
            y = math.cos(theta)*math.sin(theta) + c[1]
            z = c[2]
            point_array += ((x,y,z))
            orient_array += ((0,0,theta))
            no_of_points+=1
            print 'Position captured: (%f,%f,%f); Orientation: %f'%(x,y,z,theta*180/math.pi)
        record = True


def capture_pos(data):  
    global record
    global point_array
    global orient_array
    global no_of_points
    if record == False:
        print 'here'
        capture_point = (data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z)
        curr_orient = data.pose.pose.orientation
        yaw_angle = math.atan2(2 * ((-curr_orient.x) * (-curr_orient.y) + curr_orient.w * (-curr_orient.z)),curr_orient.w * curr_orient.w + (-curr_orient.x) * (-curr_orient.x) - (-curr_orient.y) * (-curr_orient.y) - (-curr_orient.z) * (-curr_orient.z))
        capture_orient = (0,0,yaw_angle)
        point_array+=(capture_point)
        orient_array+=(capture_orient)
        no_of_points +=1
        print 'Position captured:',capture_point,'orientation',capture_orient[2]*180/math.pi,'no of points captured:',no_of_points
        record = True

def capture_pos_from_sim(data):  
    global record
    global point_array
    global orient_array
    global no_of_points
    if record == False:
        print 'here'
        capture_point = (data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z)
        curr_orient = data.pose.pose.orientation
        yaw_angle = math.atan2(2 * (curr_orient.x * curr_orient.y + curr_orient.w * curr_orient.z),curr_orient.w * curr_orient.w + curr_orient.x * curr_orient.x - curr_orient.y * curr_orient.y - curr_orient.z * curr_orient.z)
        capture_orient = (0,0,yaw_angle)
        point_array+=(capture_point)
        orient_array+=(capture_orient)
        no_of_points +=1
        print 'Position captured:',capture_point,'orientation',capture_orient[2]*180/math.pi,'no of points captured:',no_of_points
        record = True
        
if __name__ == '__main__':
    rospy.init_node('master_mover',anonymous=True)
    startpilotpub = rospy.Publisher('/autopilot_start_cmd',AutoPilotCmd,queue_size=10)
    stoppilotpub = rospy.Publisher('/autopilot_stop_cmd',AutoPilotCmd, queue_size=10)
    drone = Drone()
    speed = 5               
    navigate = False
    setpoint = (1.0,0.5,1.0) 
    setorient = (0.0,0.0,30*math.pi/180)
    mode_list = ('move+turn','no_turn','reach&turn','turn&move')
    mode = mode_list[0]
    print 'Starting Keyboard Controller...'
    main(drone)
