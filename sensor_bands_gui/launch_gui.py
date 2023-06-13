import contextlib
import socket
import struct
import sys
import time
from threading import Thread

import matplotlib.pyplot as plt
import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QWidget
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_bands_gui.window import SingleTrajectory
from sensor_bands_gui.window import Window
from sensor_bands_gui.dataStructures import Constants
from sensor_bands_gui.dataStructures import Quaternion


class MyGuiNode(Node):
    bindIp = "0.0.0.0"  # Set the server to listen allow connections from any client
    bindPort = 4444
    bufferSize = 10000

    bands: dict[str, str]

    group: list[list[str]]

    def __init__(self, uiArg: Window) -> None:
        super().__init__("sensorBandGui")  # type:ignore
        self.ui = uiArg
        self.logger = self.get_logger()

        self.numBytes = 2  # size in  bytes of selected fw Datatype
        self.numValues = (
            Constants.NUM_FSR.value * Constants.NUM_FSR_FRAMES.value
        ) + Constants.NUM_IMU.value
        self.totalBytes = self.numValues * self.numBytes

        self.timer = QTimer()
        self.timer.setInterval(5)
        self.timer.timeout.connect(self._timerCallback)
        self.timer.start()

        self.bands = {}

        self.searching = False
        self.recording = False

        self._connectButtons()

        self.group = [
            [f"{name}{x}" for x in y]
            for name, y in (
                ("FSR", range(8)),
                ("Gyro", "XYZ"),
                ("LinAcc", "XYZ"),
                ("Quaternion", "WXYZ"),
                ("Gravity", "XYZ"),
            )
        ]
        self.titles = "FSR Gyro LinAcc Quaternion Gravity".split(" ")

    def _connectButtons(self) -> None:
        self.ui.pushButtonSarch.released.connect(self._searchButtonCallback)
        self.ui.tabWidget.currentChanged.connect(self._changedTab)
        self.ui.pushButtonCollecData.released.connect(self._recordData)

    def _generateListItem(self, name: str) -> QWidget:
        # sourcery skip: class-extract-method
        widget = QWidget(self.ui.verticalLayoutWidget)
        layout = QHBoxLayout(widget)
        label = QLabel(widget)
        label.setText(name)
        line = QLineEdit(widget)
        layout.addWidget(label)
        layout.addWidget(line)
        return widget

    def _addItem(self, name: str) -> None:
        self.ui.verticalLayoutSensorBandSelection.addWidget(
            self._generateListItem(name)
        )

    def _removeItems(self, indexes: set[int]) -> None:
        listIndexes = list(indexes)
        listIndexes.sort(reverse=True)
        for index in listIndexes:
            widget = self.ui.verticalLayoutSensorBandSelection.itemAt(index).widget()
            self.ui.verticalLayoutSensorBandSelection.removeWidget(widget)

    def _timerCallback(self) -> None:
        with contextlib.suppress(TimeoutError), socket.socket(
            family=socket.AF_INET, type=socket.SOCK_DGRAM
        ) as sock:
            sock.bind((self.bindIp, self.bindPort))
            sock.settimeout(0.0001)
            msg, (addr, _) = sock.recvfrom(self.bufferSize)
            self._newIP(addr)
            self._getData(msg, addr)
            self._updateGraphs()

    def _getData(self, msg: bytes, ip: str) -> None:
        if ip not in self.bands or (name := self.bands[ip]) not in self.ui.trajectory:
            return
        data = self._decode(msg)

        numDataFrames = len(data)

        for i in range(numDataFrames):
            dataBuffer = self._getFSR(data, i)
            imuBuffer = self._getIMU(data, i)

            for sample in dataBuffer:
                sample.extend(imuBuffer)
                sample.append(time.time())
                self.ui.trajectory[name].add(sample)

    def _decode(self, msg: bytes) -> list[list[int]]:
        dataType = "h"

        dataDecoded = []
        for j in range(Constants.NUM_FRAMES.value):
            buff = msg[j * self.totalBytes : self.totalBytes * (j + 1)]
            dataFrame = []

            for i in range(self.numValues):
                dataRaw = buff[i * self.numBytes : self.numBytes * (i + 1)]
                (value,) = struct.unpack(dataType, dataRaw)
                dataFrame.append(value)

            dataDecoded.append(dataFrame)
        return dataDecoded

    def _getFSR(self, data: list[list[int]], i: int) -> list[list[float]]:
        dataBuffer: list[list[float]] = []
        numFSR = Constants.NUM_FSR.value
        dataBuffer.extend(
            [float(el) for el in data[i][j * numFSR : numFSR * (1 + j)]]
            for j in range(Constants.NUM_FSR_FRAMES.value)
        )
        return dataBuffer

    def _getIMU(self, data: list[list[int]], i: int) -> list[float]:
        j = Constants.NUM_FSR.value * Constants.NUM_FSR_FRAMES.value
        imuBuffer = [x / 10 for x in data[i][j : j + 3]]

        acc = [x / 100 for x in data[i][j + 3 : j + 6]]
        imuBuffer.extend(acc)

        quaternion = Quaternion([x / 1000 for x in data[i][j + 6 : j + 10]])
        imuBuffer.append(quaternion.w)
        imuBuffer.extend(quaternion.v.to_list())

        gravity = [x / 1000 for x in data[i][j + 10 : j + 13]]
        imuBuffer.extend(gravity)
        return imuBuffer

    def _newIP(self, ip: str) -> None:
        if ip in self.bands or not self.searching:
            return
        self.bands[ip] = ""
        self._addItem(ip)

    def _searchButtonCallback(self) -> None:
        if self.searching:
            self._stop_searching()
            return
        self.ui.pushButtonSarch.setText("Stop Search")
        self.searching = True

    def _stop_searching(self) -> None:
        self.ui.pushButtonSarch.setText("Search for Sensor Bands")
        self.searching = False
        self._removeUnusedBands()
        self._printBands()
        self._generateTrajectories()
        self.ui.comboBoxSensorBandGraph.clear()
        self.ui.comboBoxSensorBandGraph.addItem("")
        self.ui.comboBoxSensorBandGraph.addItems(self.bands.values())

    def _generateTrajectories(self) -> None:
        for name in self.bands.values():
            if name not in self.ui.trajectory:
                self.ui.trajectory[name] = SingleTrajectory()

    def _removeUnusedBands(self) -> None:
        remove = set()
        for i in range(self.ui.verticalLayoutSensorBandSelection.count()):
            widget = self.ui.verticalLayoutSensorBandSelection.itemAt(i).widget()
            layout = widget.layout()
            name = layout.itemAt(1).widget().text()  # type:ignore
            if not name:
                ip = layout.itemAt(0).widget().text()  # type:ignore
                if self.bands[ip] in self.ui.trajectory:
                    del self.ui.trajectory[self.bands[ip]]
                del self.bands[ip]
                remove.add(i)
        self._removeItems(remove)

    def _changedTab(self, index: int) -> None:
        if index == 0 or not self.searching:
            return
        self._searchButtonCallback()

    def _printBands(self) -> None:
        while self.ui.verticalLayoutCollectData.count():
            w = self.ui.verticalLayoutCollectData.takeAt(0).widget()
            w.deleteLater()
        self.bands = {}
        for i in range(self.ui.verticalLayoutSensorBandSelection.count()):
            ip = self._updateBands(i)
            self.ui.verticalLayoutCollectData.addWidget(
                self._generatePrintBandWidget(ip)
            )

    def _generatePrintBandWidget(self, ip: str) -> QWidget:
        # sourcery skip: extract-duplicate-method
        widget = QWidget(self.ui.verticalLayoutWidget_2)
        layout = QHBoxLayout(widget)

        ipLabel = QLabel()
        ipLabel.setText(ip)

        name = QLabel()
        name.setText(self.bands[ip])

        layout.addWidget(ipLabel)
        layout.addWidget(name)

        return widget

    def _updateBands(self, index: int) -> str:
        layout = (
            self.ui.verticalLayoutSensorBandSelection.itemAt(index).widget().layout()
        )
        ip = layout.itemAt(0).widget().text()  # type:ignore
        name = layout.itemAt(1).widget().text()  # type:ignore
        self.bands[ip] = name
        return ip

    def _updateGraphs(self) -> None:
        if self.ui.tabWidget.currentIndex() != 2:
            return
        if (selected := self.ui.comboBoxSensorBandGraph.currentText()) == "":
            for graph in self.ui.order:
                graph.setData([], [])
            return
        traj = self.ui.trajectory[selected]
        if len(traj.time) < 2:
            return
        times = traj.time[-2:]
        i = -3
        while (times[-1] - times[0]) < self.ui.timeDisplay and abs(i) < len(traj.time):
            times.insert(0, traj.time[i])
            i -= 1
        for graph, name in zip(self.ui.order, traj.order):
            data = traj.data[name][-(len(times)) :]
            graph.setData(times, data)

    def _recordData(self) -> None:
        if self.recording:
            self.recording = False
            self.ui.pushButtonCollecData.setText("Collect")
            self.ui._stopRecording()
            if self.ui.checkBoxValidate.isChecked():
                self._validatData()
            return
        if self.ui._startRecording():
            self.recording = True
            self.ui.pushButtonCollecData.setText("Stop")

    def _validatData(self) -> None:
        if not self.ui.trajectory:
            return
        for group, title in zip(self.group, self.titles):
            plt.figure(title)
            for category in group:
                for traj in self.ui.trajectory.values():
                    plt.plot(
                        self.ui._getRecordTime(traj),
                        self.ui._getRecordData(traj, category),
                    )
            plt.title(title)
        plt.show()


def main() -> None:
    rclpy.init()

    app = QApplication(sys.argv)
    win = Window()

    gui_node = MyGuiNode(win)

    executor = MultiThreadedExecutor()
    executor.add_node(gui_node)

    thread = Thread(target=executor.spin)
    thread.start()
    gui_node.logger.info("Sarted")

    try:
        win.show()
        sys.exit(app.exec())
    finally:
        gui_node.logger.info("Shutting down")
        gui_node.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()

