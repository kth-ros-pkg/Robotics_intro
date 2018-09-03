import actionlib
import rospy
import tf_lookup.msg as TLM
import tf_lookup.tf_common as TLC
import tf.msg as TM

class TfLookupClient:
    def __init__(self):
        self.al_client = actionlib.ActionClient("tf_lookup", TLM.TfLookupAction)
        self.cbs = {}
        self.ts = {}
        self.ghs = []

    def query_transform(self, target, source, cb):
        target = TLC.sanitize_frame(target)
        source = TLC.sanitize_frame(source)
        key = TLC.key_from_transform(target, source)
        if key in self.cbs:
            if rospy.get_time() - self.ts[key] < 1.0:
                return
        self.cbs[key] = cb
        self.ts[key] = rospy.get_time()
        goal = TLM.TfLookupGoal(target, source, rospy.Time(0.0))
        try:
            self.ghs.append(self.al_client.send_goal(goal, transition_cb=self._al_cb))
        except rospy.ROSException as e:
            rospy.logfatal("Failed to send goal: %s", e.message)

    def _al_cb(self, gh):
        if gh.get_comm_state() != actionlib.CommState.DONE:
            return
        if gh.get_terminal_state() == actionlib.GoalStatus.SUCCEEDED:
            result = gh.get_result()
            key = "{}@{}".format(result.transform.header.frame_id, result.transform.child_frame_id)
            if key in self.cbs:
                self.cbs[key](result.transform)
        if gh in self.ghs:
            self.ghs.remove(gh)
