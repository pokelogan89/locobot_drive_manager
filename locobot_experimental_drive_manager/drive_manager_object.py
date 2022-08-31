import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion
from cmath import pi

class Drive_Manager:
    def __init__(self,rt):
        self._rate = rt #Rate in Hz
        self._base_loc = Odometry() #Initializing odometry object for use with odometry readings
        self._bumper = BumperEvent() #Initializing bumper event topic for use with the bumper on the Kobuki
        self._jm_confirmation = String()
        self._sound = Int8()

        rospy.init_node('drive_manager') #Initializing topic for this node to operate on
        self._base_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = self._rate) #Declaring the topic to publish velocity commands (Twists) to
        self._jm_comm = rospy.Publisher('joint_command',String,queue_size = self._rate) #Declaring topic to issue commands to joint manager
        self._res_odom = rospy.Publisher('/mobile_base/commands/reset_odometry',Empty,queue_size = self._rate) #Declaring topic to reset odometry data

        rospy.Subscriber("/odom", Odometry, self.get_base_location) #Subscribing to the odometry topic to use position
        rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,self.get_bumper_state) #Subscribing to the bumper topic to use the bumper event
        rospy.Subscriber("joint_confirmation",String,self.get_jm_conf)

    #Get current odom orientation readings, convert into Euler, convert into degrees, and place into list rather than tuple
    def current_rot_euler(self):
        r_q = self._base_loc.pose.pose.orientation
        r_q = [r_q.x,r_q.y,r_q.z,r_q.w]
        r_er,r_ed = euler_from_quaternion(r_q),[]
        for r in r_er:
            r_ed.append(r*180/pi + 180)

        return r_ed

    #Getting current confirmation status from the joint manager
    def jm_conf(self):
        return self._jm_confirmation.data

    #Issuing velocity commands to the base
    def set_vel_base(self,drive,rotate):
        velo = Twist()
        velo.linear.x = drive
        velo.linear.y = 0
        velo.linear.z = 0
        velo.angular.x = 0
        velo.angular.y = 0
        velo.angular.z = rotate
        self._base_pub.publish(velo)
        rospy.sleep(1/self._rate)

    #Publishing raise or lower events to the joint manager
    def set_jm_command(self,command):
        while self.jm_conf() != "done" and not rospy.is_shutdown():
            self._jm_comm.publish(command)
            rospy.sleep(.1)
        self._jm_comm.publish("")

    def reset_odom(self):
        self._res_odom.publish()
        rospy.sleep(1/self._rate)

    #Getting the current location for the odometry subscriber
    def get_base_location(self,odom):
        self._base_loc = odom

    #Getting the current bumper data for the bumper event subscriber
    def get_bumper_state(self,bump_event):
        self._bumper = bump_event

    #Getting the confirmation from the joint manager to see if it has finishing moving
    def get_jm_conf(self,jm_cf):
        self._jm_confirmation = jm_cf
