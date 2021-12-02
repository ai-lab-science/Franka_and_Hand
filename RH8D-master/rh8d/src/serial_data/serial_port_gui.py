from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QWidget

import sys

from random import randint


class AnotherWindow(QWidget):
    """
    This "window" is a QWidget. If it has no parent, it
    will appear as a free-floating window as we want.
    """
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()
        self.label = QLabel("Another Window % d" % randint(0,100))
        layout.addWidget(self.label)
        self.setLayout(layout)
        self.clicked = False


class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.w = None  # No external window yet.
        self.button = QPushButton("Record Tactile Data")
        self.button.clicked.connect(self.show_new_window)
        self.setCentralWidget(self.button)
        self.clicked = False

    def show_new_window(self):
        if self.w is None:
            self.clicked = False
        return self.clicked


app = QApplication(sys.argv)
w = MainWindow()
print(w.show_new_window())
w.show()
app.exec()