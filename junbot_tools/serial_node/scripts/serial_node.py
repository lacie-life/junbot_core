import roslib; roslib.load_manifest("rosserial_python")
import rospy
from rosserial_python import SerialClient

import sys

if __name__=="__main__":
    rospy.init_node("serial_node")
    rospy.loginfo("ROS Serial Python Node")

    port_name = rospy.get_param('~port','/dev/ttyUSB0')
    baud = int(rospy.get_param('~baud','57600'))

    sys.argv = rospy.myargv(argv=sys.argv) 
    
    if len(sys.argv) == 2 :
        port_name  = sys.argv[1]
    rospy.loginfo("Connected on %s at %d baud" % (port_name,baud) )
    client = SerialClient(port_name, baud)
    try:
        client.run()
    except KeyboardInterrupt:
        pass
