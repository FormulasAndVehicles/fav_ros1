import rospy
from PyQt5 import QtWidgets, QtCore, QtGui
import sys
from std_msgs.msg import Float64
import signal
import rospkg
import os

DOFS = ("roll", "pitch", "yaw", "thrust", "vertical_thrust", "lateral_thrust")
BUTTONS = ("W", "S", "A", "D", "Left", "Right", "Up", "Down")


class NotClickableButton(QtWidgets.QPushButton):
    def __init__(self, parent=None):
        super().__init__(parent)

    def mousePressEvent(self, e: QtGui.QMouseEvent) -> None:
        pass


class Window(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        pkg_path = rospkg.RosPack().get_path("fav_sim")
        logo_path = os.path.join(pkg_path, "res/icon.png")
        self.setWindowIcon(QtGui.QIcon(logo_path))

        rospy.init_node("keyboard_control", disable_signals=True)
        self.publishers = self.create_publishers()
        self.scaling = {}

        slider_box = QtWidgets.QGroupBox("Thruster Scaling")
        grid_slider = QtWidgets.QGridLayout()
        self.sliders = self.create_sliders()
        for i, slider in enumerate(self.sliders):
            label = QtWidgets.QLabel(slider)
            grid_slider.addWidget(label, i, 0)
            grid_slider.addWidget(self.sliders[slider], i, 1)
        slider_box.setLayout(grid_slider)

        control_box = QtWidgets.QGroupBox("Controls")
        grid_buttons_left = QtWidgets.QGridLayout()
        self.buttons = self.create_buttons()
        grid_buttons_left.addWidget(self.buttons["W"], 0, 1)
        grid_buttons_left.addWidget(self.buttons["A"], 1, 0)
        grid_buttons_left.addWidget(self.buttons["S"], 1, 1)
        grid_buttons_left.addWidget(self.buttons["D"], 1, 2)

        grid_buttons_right = QtWidgets.QGridLayout()
        grid_buttons_right.addWidget(self.buttons["Left"], 1, 0)
        grid_buttons_right.addWidget(self.buttons["Right"], 1, 2)
        grid_buttons_right.addWidget(self.buttons["Up"], 0, 1)
        grid_buttons_right.addWidget(self.buttons["Down"], 1, 1)
        hbox = QtWidgets.QHBoxLayout()
        hbox.addLayout(grid_buttons_left)
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.VLine)
        line.setFrameShadow(QtWidgets.QFrame.Sunken)
        hbox.addWidget(line)
        hbox.addLayout(grid_buttons_right)
        control_box.setLayout(hbox)

        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(slider_box)
        vbox.addWidget(control_box)
        self.setLayout(vbox)

    def create_sliders(self):
        sliders = {}
        for dof in DOFS:
            sliders[dof] = self.create_slider(dof)
        return sliders

    def create_publishers(self):
        publishers = {}
        for dof in DOFS:
            publishers[dof] = rospy.Publisher(dof, Float64, queue_size=1)
        return publishers

    def create_slider(self, name: str):
        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setMinimum(0)
        slider.setMaximum(100)
        slider.setValue(50)
        slider.setSingleStep(1)
        slider.setObjectName(name)
        slider.valueChanged.connect(
            lambda value: self.on_slider_changed(value, name))
        self.scaling[name] = slider.value() / 100.0
        slider.setFocusPolicy(QtCore.Qt.NoFocus)
        return slider

    def create_button(self, name: str):
        button = NotClickableButton()
        button.setObjectName(name)
        button.setText(name)
        button.setFocusPolicy(QtCore.Qt.NoFocus)
        button.setStyleSheet("background-color : #183b65; color : white;")
        if name == "W":
            button.setToolTip("Positive thrust.")
        elif name == "S":
            button.setToolTip("Negative thrust.")
        elif name == "A":
            button.setToolTip("Positive lateral thrust.")
        elif name == "D":
            button.setToolTip("Negative lateral thrust.")
        elif name == "Left":
            button.setToolTip("Positive yaw.")
        elif name == "Right":
            button.setToolTip("Negative yaw.")
        elif name == "Up":
            button.setToolTip("Positive vertical thrust.")
        elif name == "Down":
            button.setToolTip("Negative vertical thrust.")
        return button

    def create_buttons(self):
        buttons = {}
        for b in BUTTONS:
            buttons[b] = self.create_button(b)
        return buttons

    def on_key_pressed(self, name):
        try:
            self.buttons[name].setStyleSheet("background-color: #2dc6d6; color: white;")
        except KeyError:
            return

    def on_key_released(self, name):
        try:
            self.buttons[name].setStyleSheet("background-color : #183b65; color : white;")
        except KeyError:
            return

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        sign = 1.0
        dof = ""
        key = ""
        if event.key() == QtCore.Qt.Key_A:
            dof = "lateral_thrust"
            sign = 1.0
            key = "A"
        elif event.key() == QtCore.Qt.Key_D:
            dof = "lateral_thrust"
            sign = -1.0
            key = "D"
        elif event.key() == QtCore.Qt.Key_W:
            dof = "thrust"
            sign = 1.0
            key = "W"
        elif event.key() == QtCore.Qt.Key_S:
            dof = "thrust"
            sign = -1.0
            key = "S"
        elif event.key() == QtCore.Qt.Key_Left:
            dof = "yaw"
            sign = 1.0
            key = "Left"
        elif event.key() == QtCore.Qt.Key_Right:
            dof = "yaw"
            sign = -1.0
            key = "Right"
        elif event.key() == QtCore.Qt.Key_Up:
            dof = "vertical_thrust"
            sign = 1.0
            key = "Up"
        elif event.key() == QtCore.Qt.Key_Down:
            dof = "vertical_thrust"
            sign = -1.0
            key = "Down"
        else:
            return
        self.on_key_pressed(key)
        self.publishers[dof].publish(Float64(sign * self.scaling[dof]))

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.isAutoRepeat():
            return
        dof = ""
        key = ""
        if event.key() == QtCore.Qt.Key_A:
            dof = "lateral_thrust"
            key = "A"
        elif event.key() == QtCore.Qt.Key_D:
            dof = "lateral_thrust"
            key = "D"
        elif event.key() == QtCore.Qt.Key_W:
            dof = "thrust"
            key = "W"
        elif event.key() == QtCore.Qt.Key_S:
            dof = "thrust"
            key = "S"
        elif event.key() == QtCore.Qt.Key_Left:
            dof = "yaw"
            key = "Left"
        elif event.key() == QtCore.Qt.Key_Right:
            dof = "yaw"
            key = "Right"
        elif event.key() == QtCore.Qt.Key_Up:
            dof = "vertical_thrust"
            key = "Up"
        elif event.key() == QtCore.Qt.Key_Down:
            dof = "vertical_thrust"
            key = "Down"
        else:
            return
        self.on_key_released(key)
        self.publishers[dof].publish(Float64(0.0))

    def on_slider_changed(self, value: int, name: str):
        self.set_scaler(value=value, name=name)

    def set_scaler(self, value: int, name: str):
        self.scaling[name] = value / 100.0


def main():
    app = QtWidgets.QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    window = Window()
    window.show()
    exit(app.exec_())


if __name__ == "__main__":
    main()
