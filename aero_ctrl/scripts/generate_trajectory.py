#!/usr/bin/env python
import rospy
import sys

from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

def get_ros_param(param_name, default_value=None):
    target_var = rospy.get_param(param_name, default_value);
    rospy.loginfo("[TRAJ] Param %s : %s" % (param_name, target_var))
    return target_var

def main():
    pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size = 1)
    pub_land = rospy.Publisher('land', Empty, queue_size = 1)
    pub_camera = rospy.Publisher('camera_control', Twist, queue_size = 1)
    pub_slam = rospy.Publisher('slam_enable', Int8, queue_size = 1)
    pub_ctrl_enable = rospy.Publisher('ctrl_enable', Bool, queue_size = 1)
    pub_fil_enable = rospy.Publisher('fil_enable', Bool, queue_size = 1)
    pub_setpoint = rospy.Publisher('setpoint', Pose, queue_size = 1)

    empty_msg = Empty()
    camera_msg = Twist()
    slam_msg = Int8()
    ctrl_enable_msg = Bool()
    fil_enable_msg = Bool()
    setpoint_msg = Pose()

    rospy.init_node('trajectory_generator', anonymous = False)

    #abs_yaw_ctrl = get_ros_param("~abs_yaw_ctrl", False)
    #abs_alt_ctrl = get_ros_param("~abs_alt_ctrl", False)
    #do_takeoff = get_ros_param("~takeoff", False)
    #do_lannd = get_ros_param("~land", False)
    plan_list = get_ros_param("~plan")

    if not plan_list or len(plan_list) <= 0:
        rospy.logfatal("[TRAJ] No plan has been specified")
        sys.exit(1)
    else:
        rospy.loginfo("[TRAJ] Plan size: %s" % (len(plan_list)))

    rospy.loginfo("[TRAJ] Starting the trajectory_generator node ...")

    last_transition_time = rospy.Time(0.0)
    current_timeout = rospy.Duration(0.0)
    plan_id = 0
    plan = {}
    rate = rospy.Rate(30)
    action = ''
    while not rospy.is_shutdown():
        try:
            if (rospy.Time.now() - last_transition_time > current_timeout):
                plan = plan_list[plan_id]
                current_timeout = rospy.Duration(plan['duration'])
                plan_id += 1
                last_transition_time = rospy.Time.now()

                action = plan['action']

                rospy.loginfo("Executing plan %s [%s]" % (plan_id, action))
                if (action == 'takeoff'):
                    pub_takeoff.publish(empty_msg)
                elif (action == 'land'):
                    pub_land.publish(empty_msg)
                elif (action == 'setpoint'):
                    target = plan['target']
                    setpoint_msg.position.x = target['px']
                    setpoint_msg.position.y = target['py']
                    setpoint_msg.position.z = target['pz']
                    setpoint_msg.orientation.z = target['yaw']
                    pub_setpoint.publish(setpoint_msg)
                elif (action == 'wait'):
                    pass
                elif (action == 'cam'):
                    target = plan['target']
                    camera_msg.angular.y = target
                    pub_camera.publish(camera_msg)
                elif (action == 'slam'):
                    target = plan['target']
                    slam_msg.data = target
                    pub_slam.publish(slam_msg)
                elif (action == 'ctrl'):
                    target = plan['target']
                    ctrl_enable_msg.data = target
                    pub_ctrl_enable.publish(ctrl_enable_msg)
                elif (action == 'fil'):
                    target = plan['target']
                    fil_enable_msg.data = target
                    pub_fil_enable.publish(fil_enable_msg)
                else:
                    rospy.logerr("[TRAJ] Invalid action")
                    current_timeout = rospy.Duration(0.0)
        except IndexError:
            break
        except KeyError as e:
            rospy.logfatal("[TRAJ] Bad plan: %s" % (e, ))
            pub_land.publish(empty_msg)

        #if action in ['setpoint', 'stop']:
        #    pub_twist.publish(cmdvel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
