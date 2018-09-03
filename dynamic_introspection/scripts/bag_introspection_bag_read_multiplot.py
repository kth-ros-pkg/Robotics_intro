import rosbag
import dynamic_introspection.msg
import matplotlib.pyplot as plt

bag = rosbag.Bag('test.bag')
for topic, msg, t in bag.read_messages(topics=['dynamic_introspection']):
    #print msg
    number_vectors = len(msg.vectors)
    index = 1
    for v in msg.vectors:
       plt.subplot(number_vectors, 1, index)
       plt.plot(v.value)
       plt.title(v.name)
       plt.grid(True)
       index += 1
         
bag.close()
plt.show()
