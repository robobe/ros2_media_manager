import os
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class MediaManagerWidget(QWidget):
    def __init__(self):
        super(MediaManagerWidget, self).__init__()
        _, package_path = get_resource('packages', 'rqt_media_manager_control')
        ui_file = os.path.join(package_path, 'share', 'rqt_media_manager_control', 'resource', 'media_manager.ui')
        loadUi(ui_file, self)
        