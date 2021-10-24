#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from vitarana_drone.srv import Gripper, GripperResponse, GripperRequest
from std_msgs.msg import String


class edrone_gripper():

    # Constructor
    def __init__(self):

        rospy.init_node('node_service_server_gripper')
        self._attach_srv_a = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self._attach_srv_a.wait_for_service()

        self._attach_srv_d = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self._attach_srv_d.wait_for_service()
        self.model_state_msg = ModelStates()
        self.box_model_name = 'parcel_box'
        self.drone_model_name = 'edrone'
        self.box_index = 0
        self.drone_index = 0
        self.pickable_flag = 'False'

        self.box_coordinates = [0.0, 0.0, 0.0]
        self.drone_coordinates = [0.0, 0.0, 0.0]
        rospy.Subscriber('/gazebo/model_states_throttle', ModelStates, self.model_state_callback)
        self.check_pub = rospy.Publisher('/edrone/gripper_check', String, queue_size=1)
        self.gripper_service = rospy.Service('/edrone/activate_gripper', Gripper, self.callback_service_on_request)

    # Destructor
    def __del__(self):
        rospy.loginfo('\033[94m' + " >>> Gripper Del." + '\033[0m')

    def model_state_callback(self, msg):
        self.model_state_msg.name = msg.name
        self.model_state_msg.pose = msg.pose
        self.model_state_msg.twist = msg.twist

    def callback_service_on_request(self, req):
        rospy.loginfo('\033[94m' + " >>> Gripper Activate: {}".format(req.activate_gripper) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Gripper Flag Pickable: {}".format(self.pickable_flag) + '\033[0m')

        if((req.activate_gripper == True) and (self.pickable_flag == 'True') ):
            self.activate_gripper()
            return GripperResponse(True)
        else:
            self.deactivate_gripper()
            return GripperResponse(False)

    def activate_gripper(self):
        rospy.loginfo("Attach request received")
        req = AttachRequest()
        req.model_name_1 = 'edrone'
        req.link_name_1 = 'base_frame'
        req.model_name_2 = 'parcel_box'
        req.link_name_2 = 'link'
        self._attach_srv_a.call(req)

    def deactivate_gripper(self):
        rospy.loginfo("Detach request received")
        req = AttachRequest()
        req.model_name_1 = 'edrone'
        req.link_name_1 = 'base_frame'
        req.model_name_2 = 'parcel_box'
        req.link_name_2 = 'link'
        self._attach_srv_d.call(req)

    def check(self):
        try:
            self.box_index = self.model_state_msg.name.index(self.box_model_name)
            self.box_coordinates[0] = self.model_state_msg.pose[self.box_index].position.x
            self.box_coordinates[1] = self.model_state_msg.pose[self.box_index].position.y
            self.box_coordinates[2] = self.model_state_msg.pose[self.box_index].position.z
        except Exception as err:
            self.box_index = -1
        try:
            self.drone_index = self.model_state_msg.name.index(self.drone_model_name)
            self.drone_coordinates[0] = self.model_state_msg.pose[self.drone_index].position.x
            self.drone_coordinates[1] = self.model_state_msg.pose[self.drone_index].position.y
            self.drone_coordinates[2] = self.model_state_msg.pose[self.drone_index].position.z
        except Exception as err:
            self.drone_index = -1

        if (self.box_index != -1 and self.drone_index !=-1 ):
            if(abs(self.drone_coordinates[0] - self.box_coordinates[0]) < 0.1 and abs(self.drone_coordinates[1] - self.box_coordinates[1]) < 0.1 and (self.box_coordinates[2]-self.drone_coordinates[2])>0.105 and (self.box_coordinates[2]-self.drone_coordinates[2])>0):
                self.pickable_flag = 'True'
            else:
                self.pickable_flag = 'False'
        else:
            self.pickable_flag = 'False'

        self.check_pub.publish(self.pickable_flag)


def main():
    eDrone_gripper = edrone_gripper()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            eDrone_gripper.check()
            r.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr("Shtdown Req")


if __name__ == "__main__":
    main()
