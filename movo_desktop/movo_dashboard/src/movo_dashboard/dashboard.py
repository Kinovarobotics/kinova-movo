import rospy
from movo_msgs.msg import *

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget
from QtGui import QMessageBox, QAction
from python_qt_binding.QtCore import QSize
from .battery_widget import BatteryWidget

class MovoDashboard(Dashboard):
    def setup(self, context):
	super(Dashboard, self).__init__(context)
        self.name = 'Movo Dashboard'
        self.message = None

        self._console = ConsoleDashWidget(self.context, minimal=False)
        self._monitor = MonitorDashWidget(self.context)
        self._bat = BatteryWidget("Battery Power")

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0
        self._bat_sub = rospy.Subscriber('/movo/feedback/battery', Battery, self._battery_callback)
        self._last_dashboard_message_time = rospy.get_time()
        self._system_charging = False

    def get_widgets(self):
        return [[self._console, self._monitor], [self._bat]]

    def _battery_callback(self,msg):
        self._bat.update_perc(msg.battery_soc)
        self._bat.update_time(msg.battery_soc)
        if (0x1000 == (msg.battery_status & 0x1000)):
            self._bat.set_charging(True)
            self._system_charging = True
        else:
            self._bat.set_charging(False)
            self._system_charging = False

    def shutdown_dashboard(self):
        self._bat_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        self._console.save_settings(plugin_settings, instance_settings)
        self._monitor.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._console.restore_settings(plugin_settings, instance_settings)
        self._monitor.restore_settings(plugin_settings, instance_settings)

