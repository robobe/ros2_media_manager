#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from rqt_gui_py.plugin import Plugin
from .media_manager_ui_loader import MediaManagerWidget
from .media_manager_backend import BackendNode
from PyQt5.QtCore import QStringListModel, Qt, QTimer, QSize
# from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout, QListView, QPushButton
from PyQt5.QtWidgets import (
    QPushButton, 
    QListView, 
    QAbstractItemView, 
    QMessageBox, 
    QLineEdit, 
    QFileDialog,
    QRadioButton,
    QComboBox,
    QLabel,
    QDialog
    )
from PyQt5.QtGui import QPixmap, QIcon
from pathlib import Path
from functools import partial
from .profile_creator_dialog import MyDialog

PKG = "rqt_media_manager_control"

class MediaManagerRqtPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self._widget = MediaManagerWidget()
        self._backend = BackendNode()
        self.elapsed_seconds = 0
        
        

        # Timer setup
        self.timer = QTimer(self._widget)
        self.timer.timeout.connect(self.update_timer)
        self.timer.setInterval(1000)  # 1 second

        icon_size = QSize(24, 24)
        icon = QIcon.fromTheme("document-open")
        actual_size = icon.actualSize(icon_size)

        self.profile_create_new = self._widget.cmd_profile_create_new
        self.profile_create_new.setIcon(QIcon.fromTheme("document-new"))  # You can replace with QIcon("path/to/new.png")
        self.profile_create_new.setText("")
        self.profile_create_new.setToolTip("Create New")
        self.profile_create_new.setIconSize(actual_size)
        self.profile_create_new.setFixedSize(actual_size)
        self.profile_create_new.clicked.connect(self.on_profile_add)

        self.profile_export = self._widget.cmd_profile_export
        self.profile_export.setIcon(QIcon.fromTheme("document-save-as"))  # You can replace with QIcon("path/to/new.png")
        self.profile_export.setText("")
        self.profile_export.setToolTip("export profiles")
        self.profile_export.setIconSize(actual_size)
        self.profile_export.setFixedSize(actual_size)
        self.profile_export.clicked.connect(self.on_export_profiles_clicked)

        self.profile_import = self._widget.cmd_profile_import
        self.profile_import.setIcon(QIcon.fromTheme("document-open"))  # You can replace with QIcon("path/to/new.png")
        self.profile_import.setText("")
        self.profile_import.setToolTip("import profiles")
        self.profile_import.setIconSize(actual_size)
        self.profile_import.setFixedSize(actual_size)
        self.profile_import.clicked.connect(self.on_import_profiles_clicked)

        self.la_timer: QLabel = self._widget.la_timer
        self.la_timer.setVisible(False)

        self.icon_label: QLabel = self._widget.icon_label
        icon_path = Path(get_package_share_directory(PKG)).joinpath("resource").joinpath("red_dot.png").as_posix()
        self.icon_label.setPixmap(QPixmap(icon_path).scaled(20, 20))
        self.icon_label.setVisible(False)

        self.combo_profiles: QComboBox = self._widget.combo_profiles
        self.m_profiles = QStringListModel()
        self.m_profiles.setStringList([])
        self.combo_profiles.setModel(self.m_profiles)
        self.combo_profiles.currentTextChanged.connect(self.on_profile_changed)

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
        self._backend.on_profile_fetch += self.__profile_loaded_handler
        context.add_widget(self._widget)
        self._backend.set_source("bag")

    def on_profile_changed(self, profile_name):
        """
        Load the selected profile and update the media list
        """
        print(profile_name)


    def on_profile_add(self):
        """
        Create a new profile
        """
        dialog = MyDialog(items=self._backend.get_all_topics(), parent=self._widget)
        if dialog.exec_() == QDialog.Accepted:
            selected_items = dialog.get_checked_items()
            comment = dialog.get_comment()
            if not selected_items:
                QMessageBox.warning(self._widget, "Warning", "No topics selected for the profile.")
                return
            self._backend.create_profile(comment, selected_items)

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


    def on_import_profiles_clicked(self):
        file_name, _ = QFileDialog.getOpenFileName(self._widget, "Select Profile File", "", "YAML Files (*.yaml)")
        if file_name:
            try:
                self._backend.import_profile(file_name)
                QMessageBox.information(self._widget, "Success", "Profile imported successfully.")
            except Exception as e:
                QMessageBox.critical(self._widget, "Error", f"Failed to import profile: {str(e)}")


    def on_export_profiles_clicked(self):
        folder = QFileDialog.getExistingDirectory(self._widget, "Select Folder")
        if folder:
            self._backend.export_profile(folder)

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

    def __profile_loaded_handler(self, data) -> None:
        self.m_profiles.setStringList(data)

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

    def update_timer(self):
        self.elapsed_seconds += 1
        h = self.elapsed_seconds // 3600
        m = (self.elapsed_seconds % 3600) // 60
        s = self.elapsed_seconds % 60
        self.la_timer.setText(f"{h:02}:{m:02}:{s:02}")

    def on_start_record_handler(self):
        """
        - disabled start
        - enable stop
        """
        self.cmd_start_record.setEnabled(False)
        self.cmd_stop_record.setEnabled(True)
        self.icon_label.setVisible(True)
        self.la_timer.setVisible(True)
        self.elapsed_seconds = 0
        self.timer.start()

    def on_stop_record_handler(self):
        """
        - enable set media
        - enable and clear media name entry box
        - disabled start and stop
        - reload data
        """
        self.timer.stop()
        self.la_timer.setText("00:00:00")
        self.la_timer.setVisible(False)
        self.cmd_set_media.setEnabled(True)
        self.txt_media_name.setEnabled(True)
        self.txt_media_name.setText("")
        self.cmd_start_record.setEnabled(False)
        self.cmd_stop_record.setEnabled(False)
        self.icon_label.setVisible(False)
        self._backend.load_media()