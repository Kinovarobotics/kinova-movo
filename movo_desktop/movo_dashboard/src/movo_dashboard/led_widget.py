import rospy
from functools import partial

from kobuki_msgs.msg import Led

from rqt_robot_dashboard.widgets import MenuDashWidget
from python_qt_binding.QtCore import QSize

class LedWidget(MenuDashWidget):
    def __init__(self, topic):
        self._pub = rospy.Publisher(topic, Led, queue_size=5)

        self._off_icon = ['bg-grey.svg', 'ic-led.svg']
        self._green_icon = ['bg-green.svg', 'ic-led.svg']
        self._orange_icon = ['bg-orange.svg', 'ic-led.svg']
        self._red_icon = ['bg-red.svg', 'ic-led.svg']

        icons = [self._off_icon, self._green_icon, self._orange_icon, self._red_icon]
        super(LedWidget, self).__init__(topic, icons=icons)
        self.setFixedSize(QSize(40,40))

        self.add_action('Off', partial(self.update_state, 0))
        self.add_action('Green', partial(self.update_state, 1))
        self.add_action('Orange', partial(self.update_state, 2))
        self.add_action('Red', partial(self.update_state, 3))
        
        self.setToolTip("LED: Off")

    def update_state(self, state):
        super(LedWidget, self).update_state(state)
        self._pub.publish(Led(state))
        if state is 1:
            self.setToolTip("LED: Green")
        elif state is 2:
            self.setToolTip("LED: Orange")
        elif state is 3:
            self.setToolTip("LED: Red")
        else:
            self.setToolTip("LED: Off")

    def close(self):
        self._pub.unregister()
    
