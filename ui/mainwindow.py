# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created by: PyQt5 UI code generator 5.12.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setWindowModality(QtCore.Qt.WindowModal)
        MainWindow.resize(1091, 780)
        MainWindow.setMinimumSize(QtCore.QSize(830, 586))
        font = QtGui.QFont()
        font.setFamily("Adobe Devanagari")
        font.setPointSize(10)
        MainWindow.setFont(font)
        MainWindow.setAccessibleName("")
        MainWindow.setStyleSheet("background:rgb(240, 240, 240)")
        MainWindow.setInputMethodHints(QtCore.Qt.ImhNone)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setEnabled(True)
        self.centralwidget.setMinimumSize(QtCore.QSize(830, 564))
        self.centralwidget.setObjectName("centralwidget")
        self.ConnectRobtoArm = QtWidgets.QPushButton(self.centralwidget)
        self.ConnectRobtoArm.setGeometry(QtCore.QRect(10, 190, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.ConnectRobtoArm.setFont(font)
        self.ConnectRobtoArm.setObjectName("ConnectRobtoArm")
        self.GripperClose = QtWidgets.QPushButton(self.centralwidget)
        self.GripperClose.setGeometry(QtCore.QRect(10, 370, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.GripperClose.setFont(font)
        self.GripperClose.setObjectName("GripperClose")
        self.CameraOn = QtWidgets.QPushButton(self.centralwidget)
        self.CameraOn.setGeometry(QtCore.QRect(10, 10, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.CameraOn.setFont(font)
        self.CameraOn.setObjectName("CameraOn")
        self.GripperOpen = QtWidgets.QPushButton(self.centralwidget)
        self.GripperOpen.setGeometry(QtCore.QRect(10, 330, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.GripperOpen.setFont(font)
        self.GripperOpen.setObjectName("GripperOpen")
        self.CameraCalibration = QtWidgets.QPushButton(self.centralwidget)
        self.CameraCalibration.setGeometry(QtCore.QRect(10, 50, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.CameraCalibration.setFont(font)
        self.CameraCalibration.setObjectName("CameraCalibration")
        self.SaveImages = QtWidgets.QPushButton(self.centralwidget)
        self.SaveImages.setGeometry(QtCore.QRect(10, 90, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.SaveImages.setFont(font)
        self.SaveImages.setObjectName("SaveImages")
        self.SetInitPos = QtWidgets.QPushButton(self.centralwidget)
        self.SetInitPos.setGeometry(QtCore.QRect(10, 230, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.SetInitPos.setFont(font)
        self.SetInitPos.setObjectName("SetInitPos")
        self.DebugCalib = QtWidgets.QPushButton(self.centralwidget)
        self.DebugCalib.setGeometry(QtCore.QRect(10, 470, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.DebugCalib.setFont(font)
        self.DebugCalib.setObjectName("DebugCalib")
        self.TestCalibration = QtWidgets.QPushButton(self.centralwidget)
        self.TestCalibration.setGeometry(QtCore.QRect(10, 510, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.TestCalibration.setFont(font)
        self.TestCalibration.setObjectName("TestCalibration")
        self.AutoEIHCalib = QtWidgets.QPushButton(self.centralwidget)
        self.AutoEIHCalib.setGeometry(QtCore.QRect(10, 430, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.AutoEIHCalib.setFont(font)
        self.AutoEIHCalib.setObjectName("AutoEIHCalib")
        self.GetPos = QtWidgets.QPushButton(self.centralwidget)
        self.GetPos.setGeometry(QtCore.QRect(10, 270, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.GetPos.setFont(font)
        self.GetPos.setObjectName("GetPos")
        self.RGBFrame = QtWidgets.QLabel(self.centralwidget)
        self.RGBFrame.setGeometry(QtCore.QRect(190, 10, 890, 500))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(10)
        self.RGBFrame.setFont(font)
        self.RGBFrame.setStyleSheet("background:rgb(255,255,255)")
        self.RGBFrame.setAlignment(QtCore.Qt.AlignCenter)
        self.RGBFrame.setObjectName("RGBFrame")
        self.DepthFrame = QtWidgets.QLabel(self.centralwidget)
        self.DepthFrame.setGeometry(QtCore.QRect(670, 520, 409, 230))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(10)
        self.DepthFrame.setFont(font)
        self.DepthFrame.setStyleSheet("background:rgb(255,255,255)")
        self.DepthFrame.setAlignment(QtCore.Qt.AlignCenter)
        self.DepthFrame.setObjectName("DepthFrame")
        self.OutPut = QtWidgets.QTextEdit(self.centralwidget)
        self.OutPut.setGeometry(QtCore.QRect(190, 520, 471, 231))
        font = QtGui.QFont()
        font.setPointSize(8)
        self.OutPut.setFont(font)
        self.OutPut.setObjectName("OutPut")
        self.PenTouchTest = QtWidgets.QPushButton(self.centralwidget)
        self.PenTouchTest.setGeometry(QtCore.QRect(10, 590, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.PenTouchTest.setFont(font)
        self.PenTouchTest.setObjectName("PenTouchTest")
        self.FindPlane = QtWidgets.QPushButton(self.centralwidget)
        self.FindPlane.setGeometry(QtCore.QRect(10, 550, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.FindPlane.setFont(font)
        self.FindPlane.setObjectName("FindPlane")
        self.DepthTest = QtWidgets.QPushButton(self.centralwidget)
        self.DepthTest.setGeometry(QtCore.QRect(10, 130, 160, 30))
        font = QtGui.QFont()
        font.setFamily("Sans Serif")
        font.setPointSize(8)
        self.DepthTest.setFont(font)
        self.DepthTest.setObjectName("DepthTest")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.ConnectRobtoArm.setText(_translate("MainWindow", "Connect TM_Arm"))
        self.GripperClose.setText(_translate("MainWindow", "Gripper Close"))
        self.CameraOn.setText(_translate("MainWindow", "Camera ON"))
        self.GripperOpen.setText(_translate("MainWindow", "Gripper Open"))
        self.CameraCalibration.setText(_translate("MainWindow", "Camera Calib"))
        self.SaveImages.setText(_translate("MainWindow", "Save Images"))
        self.SetInitPos.setText(_translate("MainWindow", "Set Init Pos"))
        self.DebugCalib.setText(_translate("MainWindow", "Debug Calib"))
        self.TestCalibration.setText(_translate("MainWindow", "Test Calib Result"))
        self.AutoEIHCalib.setText(_translate("MainWindow", "Auto EIH Calib"))
        self.GetPos.setText(_translate("MainWindow", "Get Pos"))
        self.RGBFrame.setText(_translate("MainWindow", "Please connect the Camera"))
        self.DepthFrame.setText(_translate("MainWindow", "Depth Frame"))
        self.OutPut.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Sans Serif\'; font-size:8pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:9pt;\"><br /></p></body></html>"))
        self.PenTouchTest.setText(_translate("MainWindow", "Step 2: Pen Touch Test"))
        self.FindPlane.setText(_translate("MainWindow", "Step 1: Find Plane"))
        self.DepthTest.setText(_translate("MainWindow", "BBox Depth Test"))


