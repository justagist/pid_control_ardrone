import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty

class Drone:


    def __init__(self):
        # publish commands (send to quadrotor)
        self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
        self.pub_reset = rospy.Publisher('/ardrone/reset', Empty, queue_size=10)
        self.pub_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.hasTakenOff = False


    def takeOff(self):
        self.pub_takeoff.publish(Empty())
        self.hasTakenOff = True

    def land(self):
        self.pub_land.publish(Empty())
        self.hasTakenOff = False

    def reset(self):
        self.pub_reset.publish(Empty())

    def moveLinear(self,vx,vy,vz):
        self.pub_velocity.publish(Twist(Vector3(vx,vy,vz),Vector3(0,0,0)))

    def moveAngular(self,wx,wy,wz):
        self.pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(wx,wy,wz)))

    def hover(self):
        self.moveLinear(0,0,0)
