import rospy
from locobot_experimental_drive_manager import Drive_Manager

rt = 10 #Hz

#List of instructions for the drive manager to perform
instructions = [["drive",1,.2]
                ]

#Main function for processing the instructions and deploying them to the robot
def drive_manager():
    dm = Drive_Manager(rt)
    rate = rospy.Rate(dm._rate) #10hz
    rate.sleep()

    while not rospy.is_shutdown():
        #Raising the arm out of the way so that collisions d`on't hit it
        dm.set_jm_command("raise")

        for li in instructions:
            dm.reset_odom()

            ###DRIVING###
            if li[0] == "drive":
                d_g,d_v = li[1],li[1]/abs(li[1])*li[2] #m,m/s
                x_i = dm._base_loc.pose.pose.position.x #Getting initial position
                x_c = x_i #Setting current position variable

                print("Currently at %f, goal in %f m" % (x_i,abs(d_g))) #Declaration of goal

                while abs(x_c - x_i) <= abs(d_g) and not rospy.is_shutdown():
                    #Checking if bumper state is activated and quit if so
                    if dm._bumper.state == 1:
                        print("Kobuki has collided with something, shutting down")
                        dm.set_vel_base(0,0)
                        quit()
                    x_c = dm._base_loc.pose.pose.position.x #Updating current position
                    rospy.loginfo_throttle(1,"Goal in %f m" % (abs(d_g) - abs(x_c - x_i)))
                    dm.set_vel_base(d_v,0) #Issuing velocity commands

            ###ROTATION###
            elif li[0] == "rotate":
                r_g,r_v = li[1],li[1]/abs(li[1])*li[2] #deg,m/s
                rz_i = dm.current_rot_euler()[2] #Defining current position
                rz_c,ga = rz_i,0 #Setting current position variable and resetting goal adjustment variable

                if rz_i + r_g > 360 or rz_i + r_g < 0:
                    ga = 360 #Adjusting goal calculation if the movement crosses 0 or 360 degrees

                print("Currently at %f, goal in %f deg" % (rz_i,abs(r_g)))
                cc,gc = abs(rz_c - rz_i),abs(abs(r_g) - ga) #Defining current position check and goal check variables

                while abs(gc - cc) >= 1 and not rospy.is_shutdown():
                    #Checking if bumper state is activated and quit if so
                    if dm._bumper.state == 1:
                        print("Kobuki has collided with something, shutting down")
                        dm.set_vel_base(0,0)
                        quit()
                    rz_c = dm.current_rot_euler()[2] #Getting a list of Euler angles
                    cc = abs(rz_c - rz_i) #Updating current position check
                    rospy.loginfo_throttle(1,"Goal in %f deg" % abs(gc - cc))
                    dm.set_vel_base(0,r_v) #Issuing velocity commands
            
            if li != instructions[-1]:
                print("Goal Reached")

        #Making sure that the robot has stopped moving before lowering the arm, and then lowering the arm
        rospy.sleep(1)
        dm.set_jm_command("lower")
        print("Reached Final Goal, awaiting further instructions")

        while not rospy.is_shutdown():
            rate.sleep()