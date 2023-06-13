import csv
import os
import time

from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QTabWidget
from PyQt5.QtWidgets import QVBoxLayout
from pyqtgraph import mkPen
from pyqtgraph import PlotWidget

from sensor_bands_gui.gui_ui import Ui_MainWindow


class SingleTrajectory:
    time: list[float]
    data: dict[str, list[float]]
    startTime: float

    def __init__(self) -> None:
        self.time = []
        self.order = []
        for name, y in (
            ("FSR", range(8)),
            ("Gyro", "XYZ"),
            ("LinAcc", "XYZ"),
            ("Quaternion", "WXYZ"),
            ("Gravity", "XYZ"),
        ):
            self.order.extend(f"{name}{x}" for x in y)
        self.data = {name: [] for name in self.order}
        self.startTime = time.time()

    def __repr__(self) -> str:
        return f"Time - {self.time}\nData - {self.data}"

    def add(self, data: list[float]) -> None:
        if len(data) != (need := len(self.order) + 1):
            raise ValueError(f"The data must be of lengh {need}")
        for i, name in enumerate(self.order):
            self.data[name].append(data[i])
        self.time.append(data[-1] - self.startTime)

    def __len__(self) -> int:
        return len(self.time)


class Window(QMainWindow, Ui_MainWindow):
    verticalLayoutSensorBandSelection: QVBoxLayout
    verticalLayoutCollectData: QVBoxLayout

    pushButtonSarch: QPushButton
    pushButtonCollecData: QPushButton

    lineEditFileName: QLineEdit

    tabWidget: QTabWidget

    graphicsViewFSR: PlotWidget
    graphicsViewGyro: PlotWidget
    graphicsViewLinAcc: PlotWidget
    graphicsViewGravity: PlotWidget
    graphicsViewEuler: PlotWidget

    comboBoxSensorBandGraph: QComboBox

    checkBoxValidate: QCheckBox

    trajectory: dict[str, SingleTrajectory]
    timeDisplay: float

    order: tuple

    startRecordIndex: int

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setupUi(self)

        self._initGraphs()
        self._setGaphVisuals()

        self.trajectory = {}
        self.timeDisplay = 30

    def _initGraphs(self) -> None:
        self.graphicsViewEuler.addLegend()
        self.graphicsViewEuler.setYRange(-1, 1, padding=0)  # type:ignore
        self.graphicsViewFSR.addLegend()
        self.graphicsViewFSR.setYRange(0, 500, padding=0)  # type:ignore
        self.graphicsViewGravity.addLegend()
        self.graphicsViewGravity.setYRange(-9.9, 9.9, padding=0)  # type:ignore
        self.graphicsViewGyro.addLegend()
        self.graphicsViewGyro.setYRange(-50, 50, padding=0)  # type:ignore
        self.graphicsViewLinAcc.addLegend()
        self.graphicsViewLinAcc.setYRange(-50, 50, padding=0)  # type:ignore

    def _setGaphVisuals(self) -> None:
        blackPen = mkPen(color=(0, 0, 0))
        redPen = mkPen(color=(255, 0, 0))
        bluePen = mkPen(color=(0, 0, 255))
        greenPen = mkPen(color=(0, 255, 0))
        maroonPen = mkPen(color=(128, 00, 0))
        navyPen = mkPen(color=(0, 0, 128))
        cyanPen = mkPen(color=(70, 240, 240))
        magentaPen = mkPen(color=(240, 50, 230))

        self.dataLineFSR0 = self.graphicsViewFSR.plot([], [], name="FSR0", pen=blackPen)
        self.dataLineFSR1 = self.graphicsViewFSR.plot([], [], name="FSR1", pen=redPen)
        self.dataLineFSR2 = self.graphicsViewFSR.plot([], [], name="FSR2", pen=bluePen)
        self.dataLineFSR3 = self.graphicsViewFSR.plot([], [], name="FSR3", pen=greenPen)
        self.dataLineFSR4 = self.graphicsViewFSR.plot(
            [], [], name="FSR4", pen=maroonPen
        )
        self.dataLineFSR5 = self.graphicsViewFSR.plot([], [], name="FSR5", pen=navyPen)
        self.dataLineFSR6 = self.graphicsViewFSR.plot([], [], name="FSR6", pen=cyanPen)
        self.dataLineFSR7 = self.graphicsViewFSR.plot(
            [], [], name="FSR7", pen=magentaPen
        )

        self.dataLineGyrox = self.graphicsViewGyro.plot(
            [], [], name="GyroX", pen=blackPen
        )
        self.dataLineGyroy = self.graphicsViewGyro.plot(
            [], [], name="GyroY", pen=redPen
        )
        self.dataLineGyroz = self.graphicsViewGyro.plot(
            [], [], name="GyroZ", pen=bluePen
        )

        self.dataLineGravityx = self.graphicsViewGravity.plot(
            [], [], name="GravityX", pen=blackPen
        )
        self.dataLineGravityy = self.graphicsViewGravity.plot(
            [], [], name="GravityY", pen=redPen
        )
        self.dataLineGravityz = self.graphicsViewGravity.plot(
            [], [], name="GravityZ", pen=bluePen
        )

        self.dataLineLinAccx = self.graphicsViewLinAcc.plot(
            [], [], name="LinAccX", pen=blackPen
        )
        self.dataLineLinAccy = self.graphicsViewLinAcc.plot(
            [], [], name="LinAccY", pen=redPen
        )
        self.dataLineLinAccz = self.graphicsViewLinAcc.plot(
            [], [], name="LinAccZ", pen=bluePen
        )

        self.dataLineQuatw = self.graphicsViewEuler.plot(
            [], [], name="QuaternionW", pen=blackPen
        )
        self.dataLineQuatx = self.graphicsViewEuler.plot(
            [], [], name="QuaternionX", pen=redPen
        )
        self.dataLineQuaty = self.graphicsViewEuler.plot(
            [], [], name="QuaternionY", pen=bluePen
        )
        self.dataLineQuatz = self.graphicsViewEuler.plot(
            [], [], name="QuaternionZ", pen=greenPen
        )

        self.order = (
            self.dataLineFSR0,
            self.dataLineFSR1,
            self.dataLineFSR2,
            self.dataLineFSR3,
            self.dataLineFSR4,
            self.dataLineFSR5,
            self.dataLineFSR6,
            self.dataLineFSR7,
            self.dataLineGyrox,
            self.dataLineGyroy,
            self.dataLineGyroz,
            self.dataLineLinAccx,
            self.dataLineLinAccy,
            self.dataLineLinAccz,
            self.dataLineQuatw,
            self.dataLineQuatx,
            self.dataLineQuaty,
            self.dataLineQuatz,
            self.dataLineGravityx,
            self.dataLineGravityy,
            self.dataLineGravityz,
        )

    def _startRecording(self) -> bool:
        if len(self.trajectory) == 0:
            return False
        band = list(self.trajectory.keys())[0]
        self.startRecordIndex = len(self.trajectory[band])
        if self.startRecordIndex > 0:
            self.startRecordIndex -= 1
        return True

    def _stopRecording(self) -> None:
        name = self._generateName()
        with open(name, "w") as f:
            writer = csv.writer(f)
            for band, traj in self.trajectory.items():
                times = self._getRecordTime(traj)
                writer.writerow([band, "Time"] + times)
                for el in traj.order:
                    writer.writerow([band, el] + self._getRecordData(traj, el))

    def _getRecordTime(self, traj: SingleTrajectory) -> list[float]:
        return [
            t - traj.time[self.startRecordIndex]
            for t in traj.time[self.startRecordIndex :]
        ]

    def _getRecordData(self, traj: SingleTrajectory, data: str) -> list[float]:
        return traj.data[data][self.startRecordIndex :]

    def _generateName(self) -> str:
        i = 0
        name = self.lineEditFileName.text() or "data"
        while os.path.exists(f"{name}{i}.csv"):
            i += 1
        return f"{name}{i}.csv"

