#!/usr/bin/env python3

from rqt_gui_py.plugin import Plugin
from .bag_manager_ui_loader import BagManagerWidget
from .bag_manager_backend import BackendNode
from PyQt5.QtCore import QStringListModel, Qt
# from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout, QListView, QPushButton
from PyQt5.QtWidgets import QPushButton, QListView, QAbstractItemView, QMessageBox, QLineEdit, QFileDialog

class MediaManagerRqtPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self._widget = BagManagerWidget()
        self._backend = BackendNode()