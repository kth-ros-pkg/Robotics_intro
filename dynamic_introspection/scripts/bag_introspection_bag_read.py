import rosbag
import dynamic_introspection.msg
import matplotlib.pyplot as plt

bag = rosbag.Bag('test.bag')
for topic, msg, t in bag.read_messages(topics=['dynamic_introspection']):
    print msg
    for v in msg.vectors:
       print v
       plt.plot(v.value)
         
bag.close()
plt.show()