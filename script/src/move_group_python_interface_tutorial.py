#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def moving():
    
    # goal1
    goal1=FollowJointTrajectoryGoal() # create goal
    goal1.trajectory.joint_names = ['joint_a1', 'joint_a2', 'joint_a3','joint_a4', 'joint_a5', 'joint_a6']
    point1 = JointTrajectoryPoint()
    point1.positions = [0 , -1.57 , 1    ,1.57 , 1.57 ,0]
    point1.time_from_start = rospy.Duration(7)
    goal1.trajectory.points.append(point1)

    # goal2
    goal2=FollowJointTrajectoryGoal() # create goal
    goal2.trajectory.joint_names = ['joint_a1', 'joint_a2', 'joint_a3','joint_a4', 'joint_a5', 'joint_a6']
    point2 = JointTrajectoryPoint()
    point2.positions = [0 , -1.57 , 1.57    ,0 , 0 ,0]
    point2.time_from_start = rospy.Duration(7)
    goal2.trajectory.points.append(point2)

    # send goal 1
    rospy.logwarn("Kuka is moving Goal 1")

    action.send_goal_and_wait(goal1)
    rospy.logwarn("Kuka is moving Goal 2")
    action.send_goal_and_wait(goal2)

    """
    # sleeep 2
    rospy.sleep(rospy.Duration(3))
    rospy.logwarn("send Goal 3")
    action.cancel_goal()
    #
    rospy.sleep(rospy.Duration(10))
    rospy.logwarn("sleep 10sec")

    # send goal 2
    rospy.logwarn("send goal2 again")
    action.send_goal_and_wait(goal2)
    """
    rospy.logwarn("Controller Kuka successfully finished")

def main():
    global action
    rospy.init_node('move_group_python_interface')
    action=actionlib.SimpleActionClient('/position_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.logwarn("Wait for server")
    action.wait_for_server()
    rospy.logwarn("MoveGroup Python Interface")

    moving()
    now = rospy.get_rostime()
    rospy.loginfo("Current time %i %i", now.secs, now.nsecs)

if __name__ == '__main__': main()