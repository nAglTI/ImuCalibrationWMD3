# -*- coding: utf-8 -*-
# Form implementation generated from reading ui file 'gui.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMessageBox
import serial
import serial.tools.list_ports
import calibration_wmd3


alert_map = {0: ["Ошибка", "Длина значений в полях осей не должна превышать 4!"],
             1: ["Внимание", "Калибровка завершена"],
             2: ["Внимание", "Калибровка началась. Ввод будет временно отключен!"],
             3: ["Ошибка", "Нельзя отправлять команды во время работы калибровки!"],
             4: ["Ошибка", "Нельзя использовать одиннаковые порты для подключения к разным устройствам!"]}

is_start_calibration = False
is_start_allan_dev = False


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setWindowModality(QtCore.Qt.NonModal)
        MainWindow.setEnabled(True)
        MainWindow.resize(350, 350)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMinimumSize(QtCore.QSize(350, 350))
        MainWindow.setMaximumSize(QtCore.QSize(350, 350))
        MainWindow.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        MainWindow.setAnimated(True)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setMinimumSize(QtCore.QSize(7, 0))
        self.centralwidget.setMaximumSize(QtCore.QSize(1000, 320))
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 321, 284))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setContentsMargins(-1, -1, -1, 5)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_4 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_4.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_7.addWidget(self.label_4)
        self.wmd3ComList = QtWidgets.QComboBox(self.verticalLayoutWidget)
        self.wmd3ComList.setObjectName("wmd3ComList")
        self.horizontalLayout_7.addWidget(self.wmd3ComList)
        self.horizontalLayout_7.setStretch(0, 1)
        self.horizontalLayout_7.setStretch(1, 2)
        self.verticalLayout.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setContentsMargins(-1, -1, -1, 5)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label_5 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_5.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_8.addWidget(self.label_5)
        self.imuComList = QtWidgets.QComboBox(self.verticalLayoutWidget)
        self.imuComList.setObjectName("imuComList")
        self.horizontalLayout_8.addWidget(self.imuComList)
        self.horizontalLayout_8.setStretch(0, 1)
        self.horizontalLayout_8.setStretch(1, 2)
        self.verticalLayout.addLayout(self.horizontalLayout_8)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setSizeConstraint(QtWidgets.QLayout.SetNoConstraint)
        self.horizontalLayout_6.setContentsMargins(-1, -1, -1, 5)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label.setEnabled(True)
        self.label.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label.setObjectName("label")
        self.horizontalLayout_6.addWidget(self.label)
        self.xText = QtWidgets.QTextEdit(self.verticalLayoutWidget)
        self.xText.setMaximumSize(QtCore.QSize(16777215, 30))
        self.xText.setInputMethodHints(QtCore.Qt.ImhDigitsOnly)
        self.xText.setObjectName("xText")
        self.horizontalLayout_6.addWidget(self.xText)
        self.horizontalLayout_6.setStretch(0, 1)
        self.horizontalLayout_6.setStretch(1, 15)
        self.verticalLayout.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setContentsMargins(-1, -1, -1, 5)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_2 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_2.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_5.addWidget(self.label_2)
        self.yText = QtWidgets.QTextEdit(self.verticalLayoutWidget)
        self.yText.setMaximumSize(QtCore.QSize(16777215, 30))
        self.yText.setObjectName("yText")
        self.horizontalLayout_5.addWidget(self.yText)
        self.horizontalLayout_5.setStretch(0, 1)
        self.horizontalLayout_5.setStretch(1, 15)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setContentsMargins(-1, -1, -1, 5)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_3 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_3.setMaximumSize(QtCore.QSize(16777215, 20))
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_3.addWidget(self.label_3)
        self.zText = QtWidgets.QTextEdit(self.verticalLayoutWidget)
        self.zText.setMaximumSize(QtCore.QSize(16777215, 30))
        self.zText.setObjectName("zText")
        self.horizontalLayout_3.addWidget(self.zText)
        self.horizontalLayout_3.setStretch(0, 1)
        self.horizontalLayout_3.setStretch(1, 15)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setContentsMargins(-1, -1, -1, 10)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.home0Btn = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.home0Btn.setObjectName("home0Btn")
        self.home0Btn.clicked.connect(lambda: self.home_event())
        self.horizontalLayout_4.addWidget(self.home0Btn)
        self.startBtn = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.startBtn.setObjectName("startBtn")
        self.startBtn.clicked.connect(lambda: self.start_event())
        self.horizontalLayout_4.addWidget(self.startBtn)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.calibBtn = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.calibBtn.setObjectName("calibBtn")
        self.calibBtn.clicked.connect(lambda: self.calibration_event())
        self.verticalLayout.addWidget(self.calibBtn)
        self.pushButton = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.pushButton.setObjectName("pushButton")
        self.pushButton.clicked.connect(lambda: self.stop_allan_dev() if is_start_allan_dev else self.start_allan_dev())
        self.verticalLayout.addWidget(self.pushButton)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 350, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Calibration GUI"))
        self.label_4.setText(_translate("MainWindow", "COM для WMD3:"))
        self.label_5.setText(_translate("MainWindow", "COM для IMU:"))
        self.label.setText(_translate("MainWindow", "X:"))
        self.label_2.setText(_translate("MainWindow", "Y:"))
        self.label_3.setText(_translate("MainWindow", "Z:"))
        self.home0Btn.setText(_translate("MainWindow", "Исходное положение"))
        self.startBtn.setText(_translate("MainWindow", "Запуск"))
        self.calibBtn.setText(_translate("MainWindow", "Калибровка"))
        self.pushButton.setText(_translate("MainWindow", "Старт AllanDeviation"))
        self.wmd3ComList.addItems((com_info.name for com_info in serial.tools.list_ports.comports()))
        self.imuComList.addItems((com_info.name for com_info in serial.tools.list_ports.comports()))

    def start_allan_dev(self):
        global is_start_allan_dev
        self.pushButton.setText("Остановить AllanDeviation")
        is_start_allan_dev = True
        calibration_wmd3.start_allan_dev(self.imuComList.currentText())

    def stop_allan_dev(self):
        global is_start_allan_dev
        is_start_allan_dev = False
        self.pushButton.setText("Старт AllanDeviation")
        calibration_wmd3.stop_allan_dev()

    def start_event(self):
        if self.wmd3ComList.currentText() != self.imuComList.currentText():
            if not is_start_calibration:
                ser = serial.Serial(self.wmd3ComList.currentText(), 9600, timeout=5)
                if len(self.xText.toPlainText()) > 4 or len(self.yText.toPlainText()) > 4 or len(
                        self.zText.toPlainText()) > 4:
                    show_alert_dialog(0)
                else:
                    message = f"0deg{self.xText.toPlainText().zfill(5)}.000-{self.yText.toPlainText().zfill(5)}.000-{self.zText.toPlainText().zfill(5)}.000-00000.000-040.0\n"
                    ser.write(message.encode())
                ser.close()
            else:
                show_alert_dialog(3)
        else:
            show_alert_dialog(4)


    def home_event(self):
        if self.wmd3ComList.currentText() != self.imuComList.currentText():
            if not is_start_calibration:
                ser = serial.Serial(self.wmd3ComList.currentText(), 9600, timeout=5)
                message = "home0\n"
                ser.write(message.encode())
                ser.close()
            else:
                show_alert_dialog(3)
        else:
            show_alert_dialog(4)

    def calibration_event(self):
        if self.wmd3ComList.currentText() != self.imuComList.currentText():
            global is_start_calibration
            calibration_wmd3.start_calibration(self.wmd3ComList.currentText(), self.imuComList.currentText())
            is_start_calibration = True
            show_alert_dialog(2)
        else:
            show_alert_dialog(4)


def show_alert_dialog(alert_id):
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Critical)
    msg.setText(alert_map[alert_id][0])
    msg.setInformativeText(alert_map[alert_id][1])
    msg.setWindowTitle(alert_map[alert_id][0])
    msg.exec_()


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
