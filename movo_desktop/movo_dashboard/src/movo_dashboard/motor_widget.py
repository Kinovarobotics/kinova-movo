import rospy
from functools import partial

from kobuki_msgs.msg import MotorPower

from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize

class MotorWidget(IconToolButton):
    def __init__(self, topic):
        self._pub = rospy.Publisher(topic, MotorPower, queue_size=5)

        self._off_icon = ['bg-red.svg', 'ic-motors.svg']
        self._on_icon = ['bg-green.svg', 'ic-motors.svg']
        self._stale_icon = ['bg-grey.svg', 'ic-motors.svg', 'ol-stale-badge.svg']

        icons = [self._off_icon, self._on_icon, self._stale_icon]
        super(MotorWidget, self).__init__(topic, icons=icons)
        self.setFixedSize(QSize(40,40))

        super(MotorWidget, self).update_state(2)
        self.setToolTip("Motors: Stale")

        self.clicked.connect(self.toggle)


    def update_state(self, state):
        if state is not super(MotorWidget, self).state:
            super(MotorWidget, self).update_state(state)
            if state is 0:
                self.setToolTip("Motors: Off")
            else:
                self.setToolTip("Motors: On")

    def toggle(self):
        if super(MotorWidget, self).state is 1:
            self._pub.publish(MotorPower(0))
        else:
            self._pub.publish(MotorPower(1))

    def close(self):
        self._pub.unregister()
 
