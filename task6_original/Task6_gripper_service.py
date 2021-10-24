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
        self.box_model_name_list = ['parcel_box_0', 'parcel_box_1', 'parcel_box_2', 'parcel_box_3', 'parcel_box_4', 'parcel_box_5', 'parcel_box_6', 'parcel_box_7', 'parcel_box_8', 'parcel_box_9', 'parcel_box_10', 'parcel_box_11', 'parcel_box_12', 'parcel_box_13', 'parcel_box_14', 'parcel_box_15', 'parcel_box_16', 'parcel_box_17', 'parcel_box_18']
        self.drone_model_name = 'edrone'
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
        pickable, box_name = self.check()
        rospy.loginfo('\033[94m' + " >>> Gripper Activate: {}".format(req.activate_gripper) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Gripper Flag Pickable: {}".format(str(pickable)) + '\033[0m')
        if pickable:
            if(req.activate_gripper is True):
                self.activate_gripper(box_name)
                return GripperResponse(True)
            else:
                self.deactivate_gripper(box_name)
                return GripperResponse(False)
        else:
            return GripperResponse(False)

    def activate_gripper(self, model_name_2):
        rospy.loginfo("Attach request received")
        req = AttachRequest()
        req.model_name_1 = 'edrone'
        req.link_name_1 = 'base_frame'
        req.model_name_2 = model_name_2
        req.link_name_2 = 'link'
        self._attach_srv_a.call(req)

    def deactivate_gripper(self, model_name_2):
        rospy.loginfo("Detach request received")
        req = AttachRequest()
        req.model_name_1 = 'edrone'
        req.link_name_1 = 'base_frame'
        req.model_name_2 = model_name_2
        req.link_name_2 = 'link'
        self._attach_srv_d.call(req)

    def check(self):
        try:
            drone_index = self.model_state_msg.name.index(self.drone_model_name)
            dr_0 = self.model_state_msg.pose[drone_index].position.x
            dr_1 = self.model_state_msg.pose[drone_index].position.y
            dr_2 = self.model_state_msg.pose[drone_index].position.z
        except Exception as err:
            drone_index = -1
        pickable = False
        box_name = "None"
        if drone_index != -1:
            for box_model_name in self.box_model_name_list:
                try:
                    box_index = self.model_state_msg.name.index(box_model_name)
                    bx_0 = self.model_state_msg.pose[box_index].position.x
                    bx_1 = self.model_state_msg.pose[box_index].position.y
                    bx_2 = self.model_state_msg.pose[box_index].position.z
                except Exception as err:
                    box_index = -1
                if(box_index != -1):
                    if(abs(dr_0 - bx_0) < 0.1 and abs(dr_1 - bx_1) < 0.1 and (bx_2 - dr_2) > 0.105):
                        pickable = True
                        box_name = box_model_name
                        break
        return pickable, box_name

    def publish_check(self, pickable_flag):
        self.check_pub.publish(str(pickable_flag))


def main():
    eDrone_gripper = edrone_gripper()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            pickable, _ = eDrone_gripper.check()
            eDrone_gripper.publish_check(pickable)
            r.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr("Shtdown Req")


if __name__ == "__main__":
    main()
