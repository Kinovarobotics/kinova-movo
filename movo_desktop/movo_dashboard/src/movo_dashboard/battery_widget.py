import rospy
from rqt_robot_dashboard.widgets import BatteryDashWidget
from python_qt_binding.QtGui import QToolButton

class BatteryWidget(BatteryDashWidget):
    def __init__(self, name):
        icons = []
        charge_icons = []
        icons.append(['ic-battery-0.svg'])
        icons.append(['ic-battery-20.svg'])
        icons.append(['ic-battery-40.svg'])
        icons.append(['ic-battery-60-green.svg'])
        icons.append(['ic-battery-80-green.svg'])
        icons.append(['ic-battery-100-green.svg'])
        charge_icons.append(['ic-battery-charge-0.svg'])
        charge_icons.append(['ic-battery-charge-20.svg'])
        charge_icons.append(['ic-battery-charge-40.svg'])
        charge_icons.append(['ic-battery-charge-60-green.svg'])
        charge_icons.append(['ic-battery-charge-80-green.svg'])
        charge_icons.append(['ic-battery-charge-100-green.svg'])
        super(BatteryWidget, self).__init__(name=name, icons=icons, charge_icons=charge_icons)

