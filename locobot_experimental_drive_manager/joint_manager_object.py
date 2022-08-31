import rospy
from sensor_msgs.msg import JointState
from interbotix_xs_sdk.msg import JointGroupCommand
from std_msgs.msg import String
from cmath import pi
from math import sqrt

class Joint_Manager:
    def __init__(self,rt):
        self._rate = rt #Rate in Hz
        self._joint_states = JointState() #Initalizing JointState object for use of measuring current joint positions
        self._joint_commands = JointGroupCommand() #Initializing JointGroupCommand object for use of issuing commands to the set of joints
        self._user_command = String() #Initializing String object for communication from drive manager to the joint manager

        self._sleep = [0, -1.14, 1.6, 0, .4, 0]
        self._clear_bumper = [['',-.6,.9,'','',''],['','','',3.1,2,''],[0,-1.14,1.6,3.1,2,0]]
        self._home_pos = [['',-.6,.9,'','',''],['','','',0,0,''],self._sleep]

        self._com_dict = {
            "": "",
            "raise": self._clear_bumper,
            "lower": self._home_pos
        }

        rospy.init_node('joint_manager') #Opening node for the joint manager to operate on
        self._joint_commands = rospy.Publisher('/mobile_wx250s/commands/joint_group',JointGroupCommand,queue_size = self._rate) #Declaring topic to issue joint commands to
        self._jm_conf = rospy.Publisher('joint_confirmation',String,queue_size = self._rate) #Declaring topic to issue confirmation for the drive manager

        rospy.Subscriber('/mobile_wx250s/joint_states',JointState,self.get_joint_states) #Subscribing to joint states topic to get joint reading
        rospy.Subscriber('joint_command',String,self.get_dm_commands) #Subscribing to command topic from drive manager to get instructions

    #Getting current joint states in radians
    def joints_rad(self):
        js_rt,js_rl = self._joint_states.position[:-2],[]
        for joint in js_rt:
            js_rl.append(joint)

        return js_rl
    
    #Getting current joint states in degree
    def joints_deg(self):
        js_r,js_d = self.joints_rad(),[]

        for joint in js_r:
            js_d.append(joint*180/pi)

        return js_d

    #Get magnitude of difference between current position and programmed position
    def mag_difference(self,pp):
        cp,mag = self.joints_rad(),0

        for i,joint in enumerate(pp):
            mag += (joint - cp[i])**2
        
        mag = sqrt(mag)

        return mag

    #Getting instructions from the drive manager
    def instructions(self):
        int_list = self._com_dict[self._user_command.data]

        return int_list

    #Setting commands to the joint group
    def set_joint_states(self,joint_set):
        j_gc = JointGroupCommand()
        j_gc.name = 'arm'
        j_gc.cmd = joint_set
        self._joint_commands.publish(j_gc)

    #Publishing a done message to the drive manager so that it can continue
    def set_confirmation(self):
        self._jm_conf.publish("done")
        rospy.sleep(2)
        self._jm_conf.publish("")

    #Getting the current joint states for the joint states subscriber
    def get_joint_states(self,js):
        self._joint_states = js

    #Getting the current command from the drive manager
    def get_dm_commands(self,com):
        self._user_command = com
        