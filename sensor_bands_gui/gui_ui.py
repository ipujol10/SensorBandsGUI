# Form implementation generated from reading ui file 'gui.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets
from pyqtgraph import PlotWidget

from sensor_band.resources_rc import qInitResources

qInitResources()


class Ui_MainWindow:
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1072, 1001)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(0, 0, 1051, 951))
        self.tabWidget.setObjectName("tabWidget")
        self.tabBandSelection = QtWidgets.QWidget()
        self.tabBandSelection.setObjectName("tabBandSelection")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.tabBandSelection)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(0, 70, 471, 591))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayoutSensorBandSelection = QtWidgets.QVBoxLayout(
            self.verticalLayoutWidget
        )
        self.verticalLayoutSensorBandSelection.setContentsMargins(0, 0, 0, 0)
        self.verticalLayoutSensorBandSelection.setObjectName(
            "verticalLayoutSensorBandSelection"
        )
        self.pushButtonSarch = QtWidgets.QPushButton(self.tabBandSelection)
        self.pushButtonSarch.setGeometry(QtCore.QRect(498, 220, 181, 25))
        self.pushButtonSarch.setObjectName("pushButtonSarch")
        self.labelLogo = QtWidgets.QLabel(self.tabBandSelection)
        self.labelLogo.setGeometry(QtCore.QRect(910, 10, 111, 101))
        self.labelLogo.setPixmap(QtGui.QPixmap(":/images/images/AAU_logo_2012.png"))
        self.labelLogo.setScaledContents(True)
        self.labelLogo.setObjectName("labelLogo")
        self.label_8 = QtWidgets.QLabel(self.tabBandSelection)
        self.label_8.setGeometry(QtCore.QRect(10, 30, 67, 17))
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(self.tabBandSelection)
        self.label_9.setGeometry(QtCore.QRect(230, 30, 67, 17))
        self.label_9.setObjectName("label_9")
        self.verticalLayoutWidget.raise_()
        self.pushButtonSarch.raise_()
        self.label_8.raise_()
        self.label_9.raise_()
        self.labelLogo.raise_()
        self.tabWidget.addTab(self.tabBandSelection, "")
        self.tabColectData = QtWidgets.QWidget()
        self.tabColectData.setObjectName("tabColectData")
        self.label = QtWidgets.QLabel(self.tabColectData)
        self.label.setGeometry(QtCore.QRect(0, 4, 67, 17))
        self.label.setObjectName("label")
        self.lineEditFileName = QtWidgets.QLineEdit(self.tabColectData)
        self.lineEditFileName.setGeometry(QtCore.QRect(80, 0, 113, 25))
        self.lineEditFileName.setObjectName("lineEditFileName")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.tabColectData)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(0, 30, 421, 561))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayoutCollectData = QtWidgets.QVBoxLayout(
            self.verticalLayoutWidget_2
        )
        self.verticalLayoutCollectData.setContentsMargins(0, 0, 0, 0)
        self.verticalLayoutCollectData.setObjectName("verticalLayoutCollectData")
        self.pushButtonCollecData = QtWidgets.QPushButton(self.tabColectData)
        self.pushButtonCollecData.setGeometry(QtCore.QRect(490, 270, 89, 25))
        self.pushButtonCollecData.setObjectName("pushButtonCollecData")
        self.checkBoxValidate = QtWidgets.QCheckBox(self.tabColectData)
        self.checkBoxValidate.setGeometry(QtCore.QRect(490, 150, 117, 23))
        self.checkBoxValidate.setObjectName("checkBoxValidate")
        self.tabWidget.addTab(self.tabColectData, "")
        self.tabGraph = QtWidgets.QWidget()
        self.tabGraph.setObjectName("tabGraph")
        self.comboBoxSensorBandGraph = QtWidgets.QComboBox(self.tabGraph)
        self.comboBoxSensorBandGraph.setGeometry(QtCore.QRect(560, 10, 121, 25))
        self.comboBoxSensorBandGraph.setObjectName("comboBoxSensorBandGraph")
        self.scrollArea = QtWidgets.QScrollArea(self.tabGraph)
        self.scrollArea.setGeometry(QtCore.QRect(0, 40, 1050, 880))
        self.scrollArea.setMinimumSize(QtCore.QSize(1050, 880))
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents = QtWidgets.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 1034, 1552))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.scrollAreaWidgetContents)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayoutFSR = QtWidgets.QHBoxLayout()
        self.horizontalLayoutFSR.setObjectName("horizontalLayoutFSR")
        self.label_2 = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        self.label_2.setObjectName("label_2")
        self.horizontalLayoutFSR.addWidget(self.label_2)
        self.graphicsViewFSR = PlotWidget(self.scrollAreaWidgetContents)
        self.graphicsViewFSR.setMinimumSize(QtCore.QSize(0, 300))
        self.graphicsViewFSR.setMaximumSize(QtCore.QSize(877, 300))
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.BrushStyle.NoBrush)
        self.graphicsViewFSR.setBackgroundBrush(brush)
        self.graphicsViewFSR.setObjectName("graphicsViewFSR")
        self.horizontalLayoutFSR.addWidget(self.graphicsViewFSR)
        self.verticalLayout.addLayout(self.horizontalLayoutFSR)
        self.horizontalLayoutLinAcc = QtWidgets.QHBoxLayout()
        self.horizontalLayoutLinAcc.setObjectName("horizontalLayoutLinAcc")
        self.label_4 = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        self.label_4.setObjectName("label_4")
        self.horizontalLayoutLinAcc.addWidget(self.label_4)
        self.graphicsViewLinAcc = PlotWidget(self.scrollAreaWidgetContents)
        self.graphicsViewLinAcc.setMinimumSize(QtCore.QSize(0, 300))
        self.graphicsViewLinAcc.setMaximumSize(QtCore.QSize(16777215, 300))
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.BrushStyle.NoBrush)
        self.graphicsViewLinAcc.setBackgroundBrush(brush)
        self.graphicsViewLinAcc.setObjectName("graphicsViewLinAcc")
        self.horizontalLayoutLinAcc.addWidget(self.graphicsViewLinAcc)
        self.verticalLayout.addLayout(self.horizontalLayoutLinAcc)
        self.horizontalLayoutGravity = QtWidgets.QHBoxLayout()
        self.horizontalLayoutGravity.setObjectName("horizontalLayoutGravity")
        self.label_5 = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        self.label_5.setObjectName("label_5")
        self.horizontalLayoutGravity.addWidget(self.label_5)
        self.graphicsViewGravity = PlotWidget(self.scrollAreaWidgetContents)
        self.graphicsViewGravity.setMinimumSize(QtCore.QSize(0, 300))
        self.graphicsViewGravity.setMaximumSize(QtCore.QSize(877, 300))
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.BrushStyle.NoBrush)
        self.graphicsViewGravity.setBackgroundBrush(brush)
        self.graphicsViewGravity.setObjectName("graphicsViewGravity")
        self.horizontalLayoutGravity.addWidget(self.graphicsViewGravity)
        self.verticalLayout.addLayout(self.horizontalLayoutGravity)
        self.horizontalLayoutGyro = QtWidgets.QHBoxLayout()
        self.horizontalLayoutGyro.setObjectName("horizontalLayoutGyro")
        self.label_3 = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        self.label_3.setObjectName("label_3")
        self.horizontalLayoutGyro.addWidget(self.label_3)
        self.graphicsViewGyro = PlotWidget(self.scrollAreaWidgetContents)
        self.graphicsViewGyro.setMinimumSize(QtCore.QSize(0, 300))
        self.graphicsViewGyro.setMaximumSize(QtCore.QSize(877, 300))
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.BrushStyle.NoBrush)
        self.graphicsViewGyro.setBackgroundBrush(brush)
        self.graphicsViewGyro.setObjectName("graphicsViewGyro")
        self.horizontalLayoutGyro.addWidget(self.graphicsViewGyro)
        self.verticalLayout.addLayout(self.horizontalLayoutGyro)
        self.horizontalLayoutEuler = QtWidgets.QHBoxLayout()
        self.horizontalLayoutEuler.setObjectName("horizontalLayoutEuler")
        self.label_6 = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        self.label_6.setObjectName("label_6")
        self.horizontalLayoutEuler.addWidget(self.label_6)
        self.graphicsViewEuler = PlotWidget(self.scrollAreaWidgetContents)
        self.graphicsViewEuler.setMinimumSize(QtCore.QSize(0, 300))
        self.graphicsViewEuler.setMaximumSize(QtCore.QSize(877, 300))
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.BrushStyle.NoBrush)
        self.graphicsViewEuler.setBackgroundBrush(brush)
        self.graphicsViewEuler.setObjectName("graphicsViewEuler")
        self.horizontalLayoutEuler.addWidget(self.graphicsViewEuler)
        self.verticalLayout.addLayout(self.horizontalLayoutEuler)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.tabWidget.addTab(self.tabGraph, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1072, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Sensor Bands"))
        self.pushButtonSarch.setText(
            _translate("MainWindow", "Search for Sensor Bands")
        )
        self.label_8.setText(_translate("MainWindow", "IP"))
        self.label_9.setText(_translate("MainWindow", "Label"))
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.tabBandSelection),
            _translate("MainWindow", "Sensor Band Selection"),
        )
        self.label.setText(_translate("MainWindow", "File Name"))
        self.pushButtonCollecData.setText(_translate("MainWindow", "Collect"))
        self.checkBoxValidate.setText(_translate("MainWindow", "Validate Data"))
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.tabColectData),
            _translate("MainWindow", "Collect Data"),
        )
        self.label_2.setText(_translate("MainWindow", "FSR"))
        self.label_4.setText(_translate("MainWindow", "Linear Acceleration"))
        self.label_5.setText(_translate("MainWindow", "Gravty"))
        self.label_3.setText(_translate("MainWindow", "Gyroscope"))
        self.label_6.setText(_translate("MainWindow", "Quaternion"))
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.tabGraph), _translate("MainWindow", "Graphs")
        )
