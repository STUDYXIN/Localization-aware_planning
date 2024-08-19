import rospy

self.TFmini_sub = rospy.Subscriber("/tfmini_ros_node/TFmini",Range,self.callback2,queue_size=1)
RC = 1/(2*pi*1)
count = 0
h = 0
h_last = 0

t = 0
last_t = 0

v=0
v_last=0
def callback2(self,msg):
    h = msg.range

def main():
    rospy.init_node("debug double filter")
    while h == 0:
        print("Wait for TF Mini!!\n")
    t = rospy.Time.now().to_sec()
    h_filter = h + ((t-t_last)/(RC+(t-t_last)))*(p-p_last)
    v = (h - h_last)/(t-t_last)
    v = v_last + ((t-t_last)/(RC+(t-t_last)))*(v-v_last)





if __name__ == '__main__':
    main()