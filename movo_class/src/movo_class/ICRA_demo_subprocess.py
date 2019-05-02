#!/usr/bin/env python
"""
This code is meant to control MOVO with voice commands using a map created prior to the demo. In short, when I tell it to go somewhere, for example Nassif's office, it should go there.
"""

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
import os
import subprocess
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy, JointState
import rospkg
import signal
from face_ICRA import say
class ICRA_demo:
    def __init__(self):
        rospy.init_node('icra_mapping')
        self.current_proc=0     #These parameters are to track which process is in action. 
        self.reboot_proc=0
        self.reboot_bool=False  #These booleans are used to indicate what launch file will run.
        self.dance_demo_bool=False
        self.ass_teleop_bool=False
        self.tuck_bool=False
        self.face_hello_bool=False
        self.Publisher=rospy.Publisher("/movo/voice/text", String, queue_size=1, latch=True)
        """
        Set the mapping for the various commands. With the Xbox 360 joystick 
        """        
        self.ctrl_map  = dict({'momentary': {'MAP_BASE_A'     : {'is_button':True,'index':1,'set_val':1},
                                                'MAP_rARM_B'    : {'is_button':True,'index':2,'set_val':1},
                                                'MAP_lARM_X'      : {'is_button':True,'index':3,'set_val':1},
                                                'MAP_HEAD_Y'      : {'is_button':True,'index':4,'set_val':1},
                                                'MAP_lWRIST_LB'       : {'is_button':True,'index':5,'set_val':1},
                                                'MAP_rWRIST_RB'       : {'is_button':True,'index':6,'set_val':1},
                                                'MAP_TORS_back_'        : {'is_button':True,'index':7,'set_val':1},
                                                'MAP_HOME_start'    : {'is_button':True,'index':8,'set_val':1},
                                                'MAP_eSTOP_power' : {'is_button':True,'index':9,'set_val':1},
                                                'MAP_TRACT_lstick'     : {'is_button':True,'index':10,'set_val':1},
                                                'MAP_stby_rstick': {'is_button':True,'index':11,'set_val':1}},
                                'axis'     : {'MAP_TWIST_LR_stickleft'   : {'index' :1, 'invert_axis':False},
                                                'MAP_TWIST_UD_stickleft'      : {'index' :2, 'invert_axis':False},
                                                'MAP_TRIGGER_LT'        : {'index' :3, 'invert_axis':False},
                                                'MAP_TWIST_LR_stickright'   : {'index' :4, 'invert_axis':False},
                                                'MAP_TWIST_UD_stickright'   : {'index' :5, 'invert_axis':False},
                                                'MAP_TRIGGER_RT'   : {'index' :6, 'invert_axis':False},
                                                'MAP_CROSS_LR'   : {'index' :7, 'invert_axis':False},
                                                'MAP_CROSS_UD'   : {'index' :8, 'invert_axis':False}}})
        
        """
        Initialize the debounce logic states
        """
        self.db_cnt = dict()
        self.axis_value = dict()
        self.button_state = dict()
        
        for key, value in self.ctrl_map.iteritems():
            if key == 'momentary':
                for key, value2 in value.iteritems():
                    self.db_cnt[key]=0
                    self.button_state[key]=False
            else:
                self.db_cnt[key]=0
                self.axis_value[key]=0.0
        
        rospy.Subscriber('/joy', Joy, self._movo_mode_change)
    def _movo_mode_change(self,joyMessage):
        
        self._parse_joy_input(joyMessage)
        """
        The trigger button is the LT button (it is a Deadman). So when the LT is pressed, you can simply press 
        on another button (A,B,Y,X) to change demos. 
        """

        if self.axis_value['MAP_TRIGGER_LT']==-1.0:
            if(self.button_state['MAP_BASE_A'] and not self.tuck_bool): # When a button is pressed, we launch the function for the demo associated for the button and we kill the process that was running.
                if(self.current_proc!=0):
                    self._kill_proc(self.current_proc)
                self.tuck_bool=True
                self.dance_demo_bool=False
                self.ass_teleop_bool=False
                self.reboot_bool=False
                self.face_hello_bool=False
                say(self.Publisher,"Tuck")
                self._tuck_robot()
            elif(self.button_state['MAP_HEAD_Y'] and not self.ass_teleop_bool):
                if(self.current_proc!=0):
                   self._kill_proc(self.current_proc)
                self.tuck_bool=False
                self.dance_demo_bool=False
                self.ass_teleop_bool=True
                self.reboot_bool=False
                self.face_hello_bool=False
                say(self.Publisher,"Teleop")
                self._ass_teleop()
            elif(self.button_state['MAP_lARM_X']and not self.dance_demo_bool):
                if(self.current_proc!=0):
                    self._kill_proc(self.current_proc)
                self.tuck_bool=False
                self.dance_demo_bool=True
                self.ass_teleop_bool=False
                self.reboot_bool=False
                self.face_hello_bool=False
                say(self.Publisher,"Dance")
                self._dance_demo()
            elif(self.button_state['MAP_TORS_back_']): #The reboot is problematic. The Movo can only be rebooted one time. Otherwise, you'd have to do a manual movostop movostart
                if(self.current_proc!=0):
                    try:
                        self._kill_proc(self.current_proc)
                    except:
                        print("Reboot failed")
                self.tuck_bool=False
                self.dance_demo_bool=False
                self.ass_teleop_bool=False
                self.face_hello_bool=False
                self.reboot_bool=True
                say(self.Publisher,"Reboot")
                self._reboot()
            elif(self.button_state['MAP_rARM_B'] and not self.face_hello_bool):
                if(self.current_proc!=0):
                    self._kill_proc(self.current_proc)
                self.face_hello_bool=True
                self.tuck_bool=False
                self.dance_demo_bool=False
                self.ass_teleop_bool=False
                self.reboot_bool=False
                say(self.Publisher,"Face activation")
                self._face_hello()
    """
    These functions are used to launch the different demos with the subprocess module.
    """
    def _tuck_robot(self):
        proc=subprocess.Popen("roslaunch movo_demos tuck_robot.launch", stdout=subprocess.PIPE,shell=True, preexec_fn=os.setsid)
        self.current_proc=proc.pid
        #print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"+ str(self.current_proc))
    def _dance_demo(self):
        #proc=subprocess.Popen("cd ~/movo_ws;source devel/setup.bash;roslaunch movo_demos dance_invitation.launch", shell=True, preexec_fn=os.setsid)
        proc=subprocess.Popen("roslaunch movo_class dance_invitation_ICRA.launch",stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        #proc=subprocess.Popen("roslaunch movo_demos follow_me.launch", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        self.current_proc=proc.pid
        #print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"+ str(self.current_proc))
    def _ass_teleop(self):
        proc=subprocess.Popen("roslaunch movo_demos robot_assisted_teleop.launch", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        self.current_proc=proc.pid
        #print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"+ str(self.current_proc))
    def _face_hello(self):
        proc=subprocess.Popen("roslaunch movo_class face_ICRA.launch",stdout=subprocess.PIPE,shell=True,preexec_fn=os.setsid)
        self.current_proc=proc.pid
    def _reboot(self):
        proc=subprocess.Popen("sudo systemctl stop movo-core;sleep 3.0s ; sudo systemctl start movo-core", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        rospy.wait_for_message("/movo/right_arm/joint_states", JointState)
        self.reboot_proc=proc.pid
        subprocess.Popen("roslaunch movo_class ICRA_demo.launch",stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    def _kill_proc(self, process):
        os.killpg(os.getpgid(process), signal.SIGTERM)
        #print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"+ str(process))
        return True
    def _parse_joy_input(self,joyMessage):
        raw_button_states=dict() # This callback is used to parse the joystick.
        self.button_state=dict()
        
        for key, value in self.ctrl_map.iteritems():
            if key == 'momentary':
                for key2, value2 in value.iteritems():
                    raw_button_states[key2]=True
                    self.button_state[key2]=False
            else:
                for key2, value2 in value.iteritems():
                    self.axis_value[key2] = 0.0
        
        for key, value in self.ctrl_map.iteritems():
            if key == 'momentary':
                for key2, item in value.iteritems():
                    if item['is_button']:
                        if item['set_val'] == joyMessage.buttons[item['index']-1]:
                            raw_button_states[key2] &= True
                        else:
                            raw_button_states[key2] = False
                    else:
                        temp = joyMessage.axes[item['index']-1]
                        if (item['invert_axis']):
                            temp *= -1.0
                        if (temp >= item['set_thresh']):
                            raw_button_states[key2] &= True
                        else:
                            raw_button_states[key2] = False
                        
                    if (True == raw_button_states[key2]):
                        self.db_cnt[key2]+=1
                        if (self.db_cnt[key2] > 10):
                            self.db_cnt[key2] = 10
                            self.button_state[key2] = True
                    else:
                        self.button_state[key2] = False
                        self.db_cnt[key2] = 0
            if key == 'axis':
                for key2, item in value.iteritems():
                    temp = joyMessage.axes[item['index']-1]
                    if (item['invert_axis']):
                        temp *= -1.0
                    self.axis_value[key2] = temp
    def __del__(self):
        print("Deleting ICRA_demo_subprocess.")
    
if __name__=="__main__":
	m=ICRA_demo()
	rospy.spin()
	
	
