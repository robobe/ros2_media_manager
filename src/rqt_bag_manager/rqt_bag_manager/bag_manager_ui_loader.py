from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
import os

class BagManagerWidget(QWidget):
    def __init__(self):
        super(BagManagerWidget, self).__init__()
        _, package_path = get_resource('packages', 'rqt_bag_manager')
        ui_file = os.path.join(package_path, 'share', 'rqt_bag_manager_', 'resource', 'bag_manager.ui')
        loadUi(ui_file, self)