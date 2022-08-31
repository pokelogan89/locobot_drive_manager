import rospy
from locobot_experimental_drive_manager import Joint_Manager

rt = 10 #Hz

#Main function for processing the instructions and deploying them to the robot
def joint_manager():
    jm = Joint_Manager(rt)
    rate = rospy.Rate(rt)
    rate.sleep()
    
    while not rospy.is_shutdown():
        #Getting the instructions issued by the drive manager
        instructions = jm.instructions()

        #Only continue in the current iteration if there is a non-empty command
        if not instructions:
            rate.sleep()
            continue

        for line in instructions:
            #Creating variables for the current joint position and the programmed joint position, in radians
            j_cr,j_pr = jm.joints_rad(),[]
            
            #Parsing loop to set only non-empty joint positions, and to use current positions otherwise
            for i,p in enumerate(line):
                if p == '':
                    j_pr.append(j_cr[i])
                    continue
                j_pr.append(p)

            #Publishing the joint commands until the magnitude of difference is less than .1
            while not rospy.is_shutdown():
                if jm.mag_difference(j_pr) < .25 or jm.mag_difference(instructions[-1]) < .25:
                    print("Arm finished moving, proceeding")
                    break
                j_cr = jm.joints_rad()
                jm.set_joint_states(j_pr)
                rospy.loginfo_throttle(1,"Moving arm")
                rate.sleep()

        #Telling the drive manager that the current movement is done
        jm.set_confirmation()
