import os
import rospkg
import rospy

import numpy as np
import matplotlib.pyplot as plt

from PIL import Image as PIL_Image

from numpy import pi

# messages
from sensor_msgs.msg import JointState,Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotics_lab5.msg import PincherPose

# GUI
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')
        rp = rospkg.RosPack()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rqt_pincher_gui'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Parameters
        self.Dynamixel_connected = True
        
        # buttons
        self._widget.joint1Button.clicked.connect(self.call_j1)
        self._widget.joint2Button.clicked.connect(self.call_j2)        
        self._widget.joint3Button.clicked.connect(self.call_j3)
        self._widget.joint4Button.clicked.connect(self.call_j4)
        self._widget.joint5Button.clicked.connect(self.call_j5)
        
        self._widget.pose1Button.clicked.connect(self.pose_t1)

        # publishers
        self.targetPosePub = rospy.Publisher('/target_pincher_pose', PincherPose, queue_size=1)
        
        self.jTrajecPub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=1)
        
        # subcribers
        self.subJointState =  rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, self.callbackJointState)
        
        self.realPoseSub = rospy.Subscriber('/real_pose', PincherPose, self.update_pose)
        
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
         
    def callbackJointState(self,msg):
        
        q = np.round(np.array(msg.position)*180/pi,2)
        
        if self.Dynamixel_connected:    
            self._widget.jointDisplay.setText("qs:"+ str(q))   
                      
    def call_j1(self):
        self.callback_pub(1)
    def call_j2(self):
        self.callback_pub(2)
    def call_j3(self):
        self.callback_pub(3)
    def call_j4(self):
        self.callback_pub(4)
    def call_j5(self):
        self.callback_pub(5)

    def callback_pub(self,select_position):

        state = JointTrajectory()
        print(os.getcwd() )
        if select_position <= 5 and select_position > 1:
            img_path = "/home/jsds/catkin_ws/src/rqt_pincher_gui/resource/imgs/P"+str(select_position)+".png"
        else:            
            img_path = "/home/jsds/catkin_ws/src/rqt_pincher_gui/resource/imgs/P1.png"
        
        robot_img = np.asarray(PIL_Image.open(img_path))    
        plt.close()

        
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5"]
        point = JointTrajectoryPoint()
        
        pos_deg = self.choose_position2(select_position)
        point.positions = list(np.multiply(pi/180.0,pos_deg))

        point.time_from_start = rospy.Duration(0.5)
        state.points.append(point)
        self.jTrajecPub.publish(state)

        plt.imshow(robot_img)
        plt.show(block=False)

        print(state.points[0].positions)
    
    def pose_t1(self):
        self.pose_trajectory(1)
    def pose_t2(self):
        self.pose_trajectory(2)
    def pose_t3(self):
        self.pose_trajectory(3)
        
    def pose_trajectory (self,idx):
        if idx ==1:
            pose = PincherPose()
            pose.point.x = 100
            pose.point.y = 200 
            pose.point.z = 200
            pose.theta = 0.5
        else :
            pass
            
        self.targetPosePub.publish(pose)
        
    def update_pose(self, PincherPose_msg):
        position =PincherPose_msg.point
        self._widget.posDisplay.setText(str(PincherPose_msg)) 
        print("Pincher Position:" + str(position))
        
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.jTrajecPub.unregister()
        self.targetPosePub.unregister()
        self.new_pos.disconnect()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
    def choose_position2(self,select_position):
            
        PHome = np.array([  0,  0,   0,  0, 0])
        P1 =  PHome  + np.array([  0,  0,   0,  0, 0])
        P2 =  PHome + np.array([-25, 15, -20, 20, 0])
        P3 =  PHome + np.array([ 35,-35, 30, -30, 0])
        P4 =  PHome + np.array([-85, 20, -55, 17, 0])
        P5 =  PHome + np.array([-80, 35, -55, 45, 0])
        
        if(select_position==1) :
            return P1
        elif(select_position==2):
            return P2
        elif(select_position==3):
            return P3 
        elif(select_position==4):
            return P4 
        elif(select_position==5):
            return P5
        else:
            return PHome  
        
if __name__ == '__main__':
    print(Image())
    print(os.getcwd() )
    
    img_path = "/home/jsds/catkin_ws/src/rqt_pincher_gui/resource/imgs/P1.png"
    robot_img = np.asarray(PIL_Image.open(img_path))
    plt.imshow(robot_img)
    plt.show(block=True)

    rospy.init_node('gui',anonymous=False)
    try:
        print("Hl")
        #plug = MyPlugin()
    except rospy.ROSInterruptException:
        pass
        
        