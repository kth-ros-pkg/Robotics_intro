import actionlib
import rospy

class MockActionServer:
    def __init__(self, server_name, action_type):
        self.server_ = actionlib.SimpleActionServer(server_name,
                                                    action_type,
                                                    execute_cb=self.execute_cb,
                                                    auto_start=False)
        self.server_name = server_name
        self.server_.start()

    def execute_cb(self, goal):
        rospy.loginfo("Got goal on Mock {} server".format(self.server_name))
        while not rospy.is_shutdown() and not self.server_.is_preempt_requested():
            rospy.sleep(0.1)
            if self.success_in and rospy.Time.now() > self.success_in:
                rospy.loginfo("Mock {} success".format(self.server_name))
                self.server_.set_succeeded()
                return
            if self.abort_in and rospy.Time.now() > self.abort_in:
                rospy.loginfo("Mock {} abort".format(self.server_name))
                self.server_.set_aborted()
                return
        if self.server_.is_preempt_requested():
            self.server_.set_preempted()

    def reset(self, success_in=None, abort_in=None):
        if success_in:
            self.success_in = rospy.Time.now() + rospy.Duration(success_in)
        else:
            self.success_in = None
        if abort_in:
            self.abort_in = rospy.Time.now() + rospy.Duration(abort_in)
        else:
            self.abort_in = None
