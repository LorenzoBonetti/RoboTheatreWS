import actionlib
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
import rospy
from triskarone_msgs.msg import *
from std_msgs.msg import Float32MultiArray


class DummyPublisher:
    def __init__(self):
        rospy.init_node('dummy_publisher', anonymous=True)
        self.action_client = actionlib.SimpleActionClient('cmd_vel_manager', triskarone_msgs.msg.manual_move_baseAction)
        self.action_client.wait_for_server()
        self.run()

    def run(self):
        r = rospy.Rate(1)
        r.sleep()
        data_to_send = Float32MultiArray()
        array = []
        array.append(0)
        array.append(0)
        array.append(1.0)
        array.append(0.75)
        array.append(0.75)
        array.append(0.3)
        data_to_send.data = array
        goal = triskarone_msgs.msg.manual_move_baseGoal(goal=data_to_send)
        self.action_client.send_goal(goal=goal)
        while not self.action_client.get_state() == GoalStatus.SUCCEEDED:
            continue
        return


if __name__ == '__main__':
    try:
        DummyPublisher()
    except rospy.ROSInterruptException:
        pass
