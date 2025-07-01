#!/usr/bin/env python3

from rqt_gui_py.plugin import Plugin
from .media_manager_ui_loader import MediaManagerWidget
from .media_manager_backend import BackendNode
from PyQt5.QtCore import QStringListModel, Qt
# from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout, QListView, QPushButton
from PyQt5.QtWidgets import (
    QPushButton, 
    QListView, 
    QAbstractItemView, 
    QMessageBox, 
    QLineEdit, 
    QFileDialog,
    QRadioButton
    )
from functools import partial


class MediaManagerRqtPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self._widget = MediaManagerWidget()
        self._backend = BackendNode()

        self.radio_mp4: QRadioButton = self._widget.ra_mp4
        self.radio_mp4.toggled.connect(partial(self.on_radio_toggled, "mp4"))


        self.radio_bag: QRadioButton = self._widget.ra_bag
        self.radio_bag.toggled.connect(partial(self.on_radio_toggled, "bag"))
        self.radio_bag.setChecked(True)

        self.cmd_remove_all:QPushButton  = self._widget.cmdRemoveAll
        self.cmd_remove_all.clicked.connect(self.on_remove_all_clicked)
        self.cmd_remove_all.setEnabled(False)
        
        self.cmd_download:QPushButton  = self._widget.cmdDownload
        self.cmd_download.setEnabled(False)
        self.cmd_download.clicked.connect(self.on_download_clicked)

        self.cmd_set_media:QPushButton  = self._widget.cmdSetMedia
        self.cmd_set_media.setEnabled(False)
        self.cmd_set_media.clicked.connect(self.on_set_media_clicked)

        self.cmd_start_record:QPushButton  = self._widget.cmdStartRecord
        self.cmd_start_record.setEnabled(False)
        self.cmd_start_record.clicked.connect(self.on_start_stop_record_clicked)

        self.cmd_stop_record: QPushButton = self._widget.cmdStopRecord
        self.cmd_stop_record.setEnabled(False)
        self.cmd_stop_record.clicked.connect(self.on_start_stop_record_clicked)

        self.cmd_refresh: QPushButton = self._widget.cmdRefresh
        self.cmd_refresh.clicked.connect(self.on_refresh_clicked)
        self.cmd_refresh.setEnabled(False)

        self.cmd_remove:QPushButton  = self._widget.cmdRemove
        self.cmd_remove.setEnabled(False)
        self.cmd_remove.clicked.connect(self.on_cmd_remove_clicked)
        
        self.txt_media_name:QLineEdit = self._widget.mediaName
        self.txt_media_name.textChanged.connect(self.on_media_name_text_changed)

        self.lvMedia: QListView = self._widget.lvMedia
        self.lvMedia.setSelectionMode(QAbstractItemView.SingleSelection)
        self.m_media = QStringListModel()
        self.m_media.setStringList([])
        self.lvMedia.setModel(self.m_media)
        self.lvMedia.selectionModel().selectionChanged.connect(self.on_media_list_selection_changed)
        
        self._backend.on_media_list_fetch += self.__media_load_handler
        self._backend.on_error += self.__error_handler
        self._backend.on_set_media += self.set_media_handler
        self._backend.on_start_record += self.on_start_record_handler
        self._backend.on_stop_record += self.on_stop_record_handler
        self._backend.on_download_done += self.on_download_done_handler
        self._backend.on_connected += self.on_connected
        context.add_widget(self._widget)



    def on_radio_toggled(self, mode):
        self._backend.set_source(mode)

    def on_connected(self):
        self.cmd_remove_all.setEnabled(True)
        self.cmd_refresh.setEnabled(True)

    def on_remove_all_clicked(self):
        reply = QMessageBox.question(
            self._widget,
            "Confirm Action",
            "Remove all media files from remote?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.No:
            return
        
        self._backend.remove_all_remote_files()

    def on_set_media_clicked(self):
        self._backend.set_media(str(self.txt_media_name.text()))

    def on_start_stop_record_clicked(self):
        self._backend.start_stop_media_record()

    def on_refresh_clicked(self):
        self._backend.load_media()

    def on_download_clicked(self):
        indexes = self.lvMedia.selectedIndexes()
        file_name = self.m_media.data(indexes[0], Qt.DisplayRole)
        folder = QFileDialog.getExistingDirectory(self._widget, "Select Folder")
        if folder:
            self._backend.download_file(file_name, folder)

    def on_cmd_remove_clicked(self):
        reply = QMessageBox.question(
            self._widget,
            "Confirm Action",
            "Are you sure you want to continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.No:
            return
        indexes = self.lvMedia.selectedIndexes()
        file_name = self.m_media.data(indexes[0], Qt.DisplayRole)
        self._backend.remove_media(file_name)

    def on_media_list_selection_changed(self, selected, deselected):
        indexes = self.lvMedia.selectedIndexes()
        if indexes:
            self.cmd_remove.setEnabled(True)
            self.cmd_download.setEnabled(True)
            
        else:
            self.cmd_remove.setEnabled(False)
            self.cmd_download.setEnabled(False)

    def on_media_name_text_changed(self, text):
        if len(text) == 0:
            self.cmd_set_media.setEnabled(False)
            self.cmd_start_record.setEnabled(False)
            self.cmd_stop_record.setEnabled(False)
        else:
            self.cmd_set_media.setEnabled(True)


    def __media_load_handler(self, data) -> None:
        self.m_media.setStringList(data)

    def __error_handler(self, msg: str) -> None:
        QMessageBox.critical(self._widget, "Error", msg)

    def on_download_done_handler(self, msg: str):
        QMessageBox.information(self._widget, "info", f"Download {msg} file complete")

        
    def set_media_handler(self):
        """
        set media success
        - enable start record
        - disabled set media cmd
        - disabled set_media txt box
        """
        self.cmd_set_media.setEnabled(False)
        self.txt_media_name.setEnabled(False)
        self.cmd_start_record.setEnabled(True)

    def on_start_record_handler(self):
        """
        - disabled start
        - enable stop
        """
        self.cmd_start_record.setEnabled(False)
        self.cmd_stop_record.setEnabled(True)

    def on_stop_record_handler(self):
        """
        - enable set media
        - enable and clear media name entry box
        - disabled start and stop
        - reload data
        """
        self.cmd_set_media.setEnabled(True)
        self.txt_media_name.setEnabled(True)
        self.txt_media_name.setText("")
        self.cmd_start_record.setEnabled(False)
        self.cmd_stop_record.setEnabled(False)
        self._backend.load_media()