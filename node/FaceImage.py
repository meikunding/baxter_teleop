import roslib
import rospy
import cv2
import cv_bridge
import sensor_msgs.msg

class FaceImage(object):
    def __init__(self):
    	super(FaceImage, self).__init__()
        self.images = {
            'waiting': self._get_image('face4.png'),
            'happy': self._get_image('gerty_happy.png'),
            'smile': self._get_image('smile-2.jpg'),
            'intro': self._get_image('face3.png'),
            'winking': self._get_image('gerty_winking.png'),
            'surprised': self._get_image('gerty_surprised.png'),
            'thinking_left': self._get_image('gerty_thinking_left.png'),
            'thinking_right': self._get_image('gerty_thinking_right.png'),
            'confused': self._get_image('gerty_confused.png'),
            'unhappy': self._get_image('gerty_unhappy.png'),
            'cat': self._get_image('cat2.png'),
            'big_lemongrab': self._get_image('big_lemongrab.png'),
            'eyes': self._get_image('eyes.jpg'),
        }
        self.current_image = ' '
        self.pub = rospy.Publisher(
            '/robot/xdisplay', sensor_msgs.msg.Image, latch=True, queue_size=1)
        self.set_image('happy')

    def _get_image(self, path):
        img = cv2.imread(roslib.packages.get_pkg_dir('baxter_teleop') + '/img/' + path)  
        return cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        
    def set_image(self, img_name):
        if self.current_image != img_name:
            self.current_image = img_name
            rospy.logdebug("Setting Head Image: %s" % img_name)
            self.pub.publish(self.images[img_name])
         
            
