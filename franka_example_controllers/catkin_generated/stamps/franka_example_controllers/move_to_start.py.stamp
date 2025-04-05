#!/usr/bin/env python
import sys
import rospy as ros

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult

ros.init_node('move_to_start')

action = ros.resolve_name('~follow_joint_trajectory')
client = SimpleActionClient(action, FollowJointTrajectoryAction)
ros.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
client.wait_for_server()

param = ros.resolve_name('~joint_pose')
pose = ros.get_param(param, None)
if pose is None:
    ros.logerr('move_to_start: Could not find required parameter "' + param + '"')
    sys.exit(1)

topic = ros.resolve_name('~joint_states')
ros.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")
joint_state = ros.wait_for_message(topic, JointState)
initial_pose = dict(zip(joint_state.name, joint_state.position))

max_movement = max(abs(pose[joint] - initial_pose[joint]) for joint in pose)

point = JointTrajectoryPoint()
point.time_from_start = ros.Duration.from_sec(
    # Use either the time to move the furthest joint with 'max_dq' or 500ms,
    # whatever is greater
    max(max_movement / ros.get_param('~max_dq', 0.5), 0.5)
)
goal = FollowJointTrajectoryGoal()

goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
point.velocities = [0] * len(pose)
print(point.positions)
goal.trajectory.points.append(point)
goal.goal_time_tolerance = ros.Duration.from_sec(0.5)

print(goal)

ros.loginfo('Sending trajectory Goal to move into initial config')
client.send_goal_and_wait(goal)

result = client.get_result()
if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
    ros.logerr('move_to_start: Movement was not successful: ' + {
        FollowJointTrajectoryResult.INVALID_GOAL:
        """
        The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
        Is the 'joint_pose' reachable?
        """,

        FollowJointTrajectoryResult.INVALID_JOINTS:
        """
        The joint pose you specified is for different joints than the joint trajectory controller
        is claiming. Does you 'joint_pose' include all 7 joints of the robot?
        """,

        FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
        """
        During the motion the robot deviated from the planned path too much. Is something blocking
        the robot?
        """,

        FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
        """
        After the motion the robot deviated from the desired goal pose too much. Probably the robot
        didn't reach the joint_pose properly
        """,
    }[result.error_code])

else:
    ros.loginfo('move_to_start: Successfully moved into start pose')



'''
#!/usr/bin/env python
import time
import sys
import rospy as ros
import numpy as np
from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult

ros.init_node('move_to_start')

file_path = '/home/caleb/ros_relaxed_ik_ws/src/relaxed_ik_ros1/scripts/shy.npz'
data = np.load(file_path)
print(data['q'],data['qdot'])
print(len(data['q']), len(data['qdot']), "C")
q = [[0,0,0,0,0,0,0]]*int(len(data['q']))
qdot = [[0,0,0,0,0,0,0]]*int(len(data['qdot']))

action = ros.resolve_name('~follow_joint_trajectory')
client = SimpleActionClient(action, FollowJointTrajectoryAction)
ros.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
client.wait_for_server()

param = ros.resolve_name('~joint_pose')
pose = ros.get_param(param, None)
#if pose is None:
#    ros.logerr('move_to_start: Could not find required parameter "' + param + '"')
#    sys.exit(1)

#topic = ros.resolve_name('~joint_states')
#ros.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")
#joint_state = ros.wait_for_message(topic, JointState)
#initial_pose = dict(zip(joint_state.name, joint_state.position))

#max_movement = max(abs(pose[joint] - initial_pose[joint]) for joint in pose)
points = []
for index in range(0,len(data['q'])):
    print(len(data['q']))
    point = JointTrajectoryPoint()
    point.time_from_start = ros.Duration.from_sec(
        (index+1)*0.033
    )
    #point.time_from_start = ros.Duration.from_sec(
    #    # Use either the time to move the furthest joint with 'max_dq' or 500ms,
    #    # whatever is greater
    #    max(max_movement / ros.get_param('~max_dq', 0.5), 0.5)
    #)
    #goal = FollowJointTrajectoryGoal()
    #goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
    #point.velocities = [0] * len(pose)
    #print(len(point.positions), type(point.positions),"A")
    #goal.trajectory.points.append(point)
    #goal.goal_time_tolerance = ros.Duration.from_sec(0.5)

    #qdot = [[0,0,0,0,0,0,0]]*int(len(data['qdot']))
    #q = [[0,0,0,0,0,0,0]]*10
    #qdot = [[0,0,0,0,0,0,0]]*10
    #q[0] = [0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397]
    #qdot[0] = [0, 0, 0, 0, 0, 0, 0]
    #for i in range(0,7):
    #    q[index][i] = data['q'][index][i]
    #    qdot[index][i] = data['qdot'][index][i]
    q = data['q']
    qdot = data['qdot']
    #for i in range(1,10):   
    #    qdot[i] = [0+i*0.1, 0+i*0.1, 0+i*0.1, 0+i*0.1, 0+i*0.1, 0+i*0.1, 0+i*0.1]
    #    q[i] = [q[i-1][0]+qdot[i][0], q[i-1][1]+qdot[i][1], q[i-1][2]+qdot[i][2], q[i-1][3]+qdot[i][3], q[i-1][4]+qdot[i][4], q[i-1][5]+qdot[i][5], q[i-1][6]+qdot[i][6]]

    ros.loginfo('Sending trajectory Goal to move into initial config')
    #for joint_angle in range(0, len(q)):
    #index = joint_angle
    point.positions = q[index]
    point.velocities = [0,0,0,0,0,0,0]#qdot[index]
    points.append(point)
    
#print(len(point.positions), type(point.positions),"B")
goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names, a = [list(x) for x in zip(*pose.items())]
print(goal.trajectory.joint_names)
#point.velocities = [0] * 7
#point.positions = [0] * len(pose)
goal.trajectory.points = points
#goal.trajectory.points.append(point)
print(len(goal.trajectory.points),goal.trajectory.points[0],goal.trajectory.points[1],"AAAAAAAAAAAAAAAAAAA")
client.send_goal_and_wait(goal)
result = client.get_result()
if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
    ros.logerr('move_to_start: Movement was not successful: ' + {
        FollowJointTrajectoryResult.INVALID_GOAL:
        """
        The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
        Is the 'joint_pose' reachable?
        """,

        FollowJointTrajectoryResult.INVALID_JOINTS:
        """
        The joint pose you specified is for different joints than the joint trajectory controller
        is claiming. Does you 'joint_pose' include all 7 joints of the robot?
        """,

        FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
        """
        During the motion the robot deviated from the planned path too much. Is something blocking
        the robot?
        """,

        FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
        """
        After the motion the robot deviated from the desired goal pose too much. Probably the robot
        didn't reach the joint_pose properly
        """,
    }[result.error_code])

else:
    ros.loginfo('move_to_start: Successfully moved into start pose')
'''