#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
import cv2
# import LogicController


class decompress():
    def __init__(self):
        self.color_topic = "/camera/color/image_raw/compressed"
        self.image_subscriber = rospy.Subscriber(self.color_topic, CompressedImage, self.color_callback)
        self.image_publisher = rospy.Publisher('/camera/color/image_raw/decompressed', Image, queue_size=5)
        self.decoded = None
        # self.samples = 0
        self.rate = rospy.Rate(100)
        self.color_compressed = False
        self.color_img = None

    def color_callback(self, data): # Need semaphore to protect self.color_img, because it is also used by is_waving_hand function
            #print("Having color image")
            #cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            #print(cv_image)
            if not self.color_compressed:
                # ===== not compressed =====
                self.decoded = np.frombuffer(data.data, np.uint8)
            else:
                # ===== compressed   =======
                self.arr = np.fromstring(data.data, np.uint8)
                decoded = cv2.imdecode(self.arr, cv2.IMREAD_COLOR)

            img = np.reshape(decoded, (480,640, 3))
            #print("COLOR_CALLBACK", img)
            #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if not self.color_compressed:
                self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
            self.color_img = img
            # self.samples += 1

            # try:
            #     if self.samples >= 3:
            #         self.color_img_queue.put(img, False)
            #         self.samples = 0
            # except Exception:
            #     #self.color_img_queue.get()
            #     pass
            # #LogicController.vtmp.head_img_color.send(img) # send to pipe
            # try:
            #     LogicController.vtmp.queue.put(img, False)
            #     #print("Queue size of thread-safe queue: ", LogicController.vtmp.queue.qsize())
            # except Exception:
            #     #print("In Exception ", e)
            #     pass
            #print(img.shape)
            #img = cv2.rectangle(img, (500, 100), (520, 120), (255,0,0), -1)
            #cv2.namedWindow("img")
            #cv2.imshow("img", img)
            #cv2.waitKey(1)

    def publish(self):
        while not rospy.is_shutdown():
            self.image_publisher.publish(self.color_img)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('decode_node', anonymous=True)
    test = decompress()
    try:
        test.publish()
    except rospy.ROSInterruptException:
        pass