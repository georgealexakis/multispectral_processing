#!/usr/bin/env python

import rospy
import sys
import json
import dynamic_reconfigure.client
from std_msgs.msg import String
from std_msgs.msg import Int32


class CameraConfigurator:
    def __init__(self):
        # Client for camera reconfiguration
        self.client = dynamic_reconfigure.client.Client(
            '/ueye_cam_nodelet', 1000)
        # Topics for connection between controllers
        self.sub_cc = rospy.Subscriber(
            "/camera_controller", String, self.callback)
        self.sub_cl = rospy.Subscriber(
            "/client_count", Int32, self.clientsCallback)
        self.pub_cf = rospy.Publisher(
            "/camera_controller/feedback", String, queue_size=1)
        rospy.loginfo("Camera configurator node  started")

    def clientsCallback(self, data):
        if(data.data == 0):
            self.sub_cc.unregister()
        else:
            self.sub_cc = rospy.Subscriber(
                "/camera_controller", String, self.callback)

    def callback(self, data):
        # Get parameters as a string with - and split (only 6 parameters, but more can be used)
        inputData = str(data.data)
        if(inputData != "sync"):
            inputParameters = inputData.split("-")
            print("----------------------- Set Camera Parameters -----------------------")
            print("     Exposure:            " + inputParameters[0])
            print("     Pixel clock:         " + inputParameters[1])
            print("     Frame rate:          " + inputParameters[2])
            print("     Auto white balance:  " + inputParameters[3])
            print("     Auto frame rate:     " + inputParameters[4])
            print("     Auto exposure:       " + inputParameters[5])
            print("---------------------------------------------------------------------")

            # Set parameters to the camera client
            outParameters = {'exposure': inputParameters[0], 'pixel_clock': inputParameters[1], 'frame_rate': inputParameters[2],
                             'auto_white_balance': inputParameters[3], 'auto_frame_rate': inputParameters[4], 'auto_exposure': inputParameters[5]}
            self.client.update_configuration(outParameters)
        else:
            print("Synchronization message arrived.")

        # Get parameters
        inParameters = self.client.get_configuration()

        # Convert dictionary into string and send it back to the controller for feedback. Transmitted string example: "10-55-30-False-True-True"
        ex = inParameters.exposure
        pc = inParameters.pixel_clock
        fr = inParameters.frame_rate
        aw = inParameters.auto_white_balance
        af = inParameters.auto_frame_rate
        ae = inParameters.auto_exposure
        feedback = str(ex) + "-" + str(pc) + "-" + str(fr) + "-" + \
            str(aw) + "-" + str(af) + "-" + str(ae)
        print("-------------------- Updated Camera Parameters ----------------------")
        print("     Exposure:            " + str(ex))
        print("     Pixel clock:         " + str(pc))
        print("     Frame rate:          " + str(fr))
        print("     Auto white balance:  " + str(aw))
        print("     Auto frame rate:     " + str(af))
        print("     Auto exposure:       " + str(ae))
        print("---------------------------------------------------------------------")
        self.pub_cf.publish(feedback)

    # Shut down everything
    def stop(self):
        self.client.close()
        self.sub_cc.unregister()
        self.sub_cl.unregister()
        self.pub_cf.unregister()
        rospy.signal_shutdown("Exit")
        rospy.loginfo("Camera configurator node shutted down")


def main(args):
    rospy.init_node("camera_configurator", anonymous=True)
    configurator = CameraConfigurator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    configurator.stop()


if __name__ == "__main__":
    main(sys.argv)
