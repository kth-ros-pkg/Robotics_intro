import rosbag
import rospy
from dynamic_introspection.msg import IntrospectionMsg
import matplotlib.pyplot as plt

def callback(msg):
    number_vectors = len(msg.vectors)
    index = 1
    for v in msg.vectors:
       plt.subplot(number_vectors, 1, index)
       plt.plot(v.value)
       plt.title(v.name)
       plt.grid(True)
       index += 1
       
    plt.show(block=False) 
     
def listener():


    rospy.init_node('dynamic_introspection_plotting_node', anonymous=True)
    rospy.Subscriber("debug", IntrospectionMsg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()