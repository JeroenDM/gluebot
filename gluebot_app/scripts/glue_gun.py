#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import String

from std_srvs.srv import *

from ur_driver.io_interface import *

def set_glue_gun(req):
    print("Glue gun set to:")
    print(req.data)
    set_digital_out(4, req.data)
    return SetBoolResponse(True, "Glue gun set.")

def glue_gun_server():
    rospy.Service('set_glue_gun', SetBool, set_glue_gun)
    print("Ready to control glue gun.")
    

if __name__ == "__main__":
    #rospy.init_node('glue_gun_server')

    print "testing io-interface"
    get_states()
    print "listener has been activated"
    set_states()

    glue_gun_server()
    rospy.spin()

    

# if __name__ == "__main__":
#     print "testing io-interface"
#     get_states()
#     print "listener has been activated"
#     set_states()
#     print "service-server has been started"
#     while(True and not rospy.is_shutdown() ):
#         set_digital_out(0, True)
#         set_digital_out(4, True)
#         print(Digital_Out_States[0])
#         print(Digital_Out_States[4])

#         time.sleep(1)

#         set_digital_out(0, False)
#         set_digital_out(4, False)
#         print(Digital_Out_States[0])
#         print(Digital_Out_States[4])

#         time.sleep(1)

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass