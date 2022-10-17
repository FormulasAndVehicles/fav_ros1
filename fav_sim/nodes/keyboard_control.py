import rospy
from PyQt5 import QtWidgets, QtCore, QtGui
import sys
from std_msgs.msg import Float64
import signal

DOFS = ("roll", "pitch", "yaw", "thrust", "vertical_thrust", "lateral_thrust")


class Window(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        rospy.init_node("keyboard_control", disable_signals=True)
        self.publishers = self.create_publishers()
        self.scaling = {}

        grid = QtWidgets.QGridLayout()
        self.sliders = self.create_sliders()
        for i, slider in enumerate(self.sliders):
            label = QtWidgets.QLabel(slider)
            grid.addWidget(label, i, 0)
            grid.addWidget(self.sliders[slider], i, 1)
        self.setLayout(grid)

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

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        sign = 1.0
        dof = ""
        if event.key() == QtCore.Qt.Key_A:
            dof = "lateral_thrust"
            sign = 1.0
        elif event.key() == QtCore.Qt.Key_D:
            dof = "lateral_thrust"
            sign = -1.0
        elif event.key() == QtCore.Qt.Key_W:
            dof = "thrust"
            sign = 1.0
        elif event.key() == QtCore.Qt.Key_S:
            dof = "thrust"
            sign = -1.0
        elif event.key() == QtCore.Qt.Key_Left:
            dof = "yaw"
            sign = 1.0
        elif event.key() == QtCore.Qt.Key_Right:
            dof = "yaw"
            sign = -1.0
        elif event.key() == QtCore.Qt.Key_Up:
            dof = "vertical_thrust"
            sign = 1.0
        elif event.key() == QtCore.Qt.Key_Down:
            dof = "vertical_thrust"
            sign = -1.0
        else:
            return
        self.publishers[dof].publish(Float64(sign * self.scaling[dof]))

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:
        if event.isAutoRepeat():
            return
        dof = ""
        if event.key() == QtCore.Qt.Key_A:
            dof = "lateral_thrust"
        elif event.key() == QtCore.Qt.Key_D:
            dof = "lateral_thrust"
        elif event.key() == QtCore.Qt.Key_W:
            dof = "thrust"
        elif event.key() == QtCore.Qt.Key_S:
            dof = "thrust"
        elif event.key() == QtCore.Qt.Key_Left:
            dof = "yaw"
        elif event.key() == QtCore.Qt.Key_Right:
            dof = "yaw"
        elif event.key() == QtCore.Qt.Key_Up:
            dof = "vertical_thrust"
        elif event.key() == QtCore.Qt.Key_Down:
            dof = "vertical_thrust"
        else:
            return
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
