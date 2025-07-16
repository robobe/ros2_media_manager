

from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QCheckBox, QLabel, QLineEdit, QPushButton

class MyDialog(QDialog):
    def __init__(self, items, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Items")
        self.resize(300, 200)

        self.layout = QVBoxLayout(self)

        # Create checkboxes
        self.checkboxes = []
        for item in items:
            cb = QCheckBox(item)
            self.checkboxes.append(cb)
            self.layout.addWidget(cb)

        # Comment input
        self.layout.addWidget(QLabel("Comment:"))
        self.comment_box = QLineEdit()
        self.layout.addWidget(self.comment_box)

        # OK / Cancel buttons
        btn_layout = QHBoxLayout()
        self.ok_button = QPushButton("OK")
        self.cancel_button = QPushButton("Cancel")

        self.ok_button.clicked.connect(self.accept)
        self.cancel_button.clicked.connect(self.reject)

        btn_layout.addWidget(self.cancel_button)
        btn_layout.addStretch()
        btn_layout.addWidget(self.ok_button)
        self.layout.addLayout(btn_layout)

    def get_checked_items(self):
        return [cb.text() for cb in self.checkboxes if cb.isChecked()]

    def get_comment(self):
        return self.comment_box.text()