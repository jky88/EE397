
'''
EE397 pyqt5 remote controller

'''
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtNetwork import *
import socket
import sys

class MainWindow(QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        #socket
        self.sock = QTcpSocket()
        self.isConnect = False
        self.setupUi()
        self.setupSlot()
    def setupUi(self):
        self.resize(1069, 788)
        self.upButton = QtWidgets.QPushButton(self)
        self.upButton.setGeometry(QtCore.QRect(750, 490, 93, 28))
        self.upButton.setObjectName("upButton")
        self.upButton.setText("前进")
        self.leftButton = QtWidgets.QPushButton(self)
        self.leftButton.setGeometry(QtCore.QRect(650, 530, 93, 28))
        self.leftButton.setObjectName("leftButton")
        self.leftButton.setText("后退")
        self.rightButton = QtWidgets.QPushButton(self)
        self.rightButton.setGeometry(QtCore.QRect(850, 530, 93, 28))
        self.rightButton.setObjectName("rightButton")
        self.rightButton.setText("左转")
        self.downButton = QtWidgets.QPushButton(self)
        self.downButton.setGeometry(QtCore.QRect(750, 570, 93, 28))
        self.downButton.setObjectName("downButton")
        self.downButton.setText("右转")
        self.Connect = QtWidgets.QPushButton(self)
        self.Connect.setGeometry(QtCore.QRect(650, 680, 93, 28))
        self.Connect.setObjectName("Connect")
        self.Connect.setText("Connect")
        self.disConnect = QtWidgets.QPushButton(self)
        self.disConnect.setGeometry(QtCore.QRect(870, 680, 93, 28))
        self.disConnect.setObjectName("disConnect")
        self.disConnect.setText("Disconnect")
        self.verticalLayoutWidget = QtWidgets.QWidget(self)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 40, 581, 390))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.controlLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.controlLayout.setContentsMargins(0, 0, 0, 0)
        self.controlLayout.setObjectName("controlLayout")
        self.label = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label.setObjectName("label")
        self.label.setText("BKP")
        self.controlLayout.addWidget(self.label)
        self.BKPEdit = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.BKPEdit.setObjectName("BKPEdit")
        self.controlLayout.addWidget(self.BKPEdit)
        self.BKPSlider = QtWidgets.QSlider(self.verticalLayoutWidget)
        self.BKPSlider.setOrientation(QtCore.Qt.Horizontal)
        self.BKPSlider.setObjectName("BKPSlider")
        self.controlLayout.addWidget(self.BKPSlider)
        self.label_2 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.label_2.setText("BKD")
        self.controlLayout.addWidget(self.label_2)
        self.BKDEdit = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.BKDEdit.setObjectName("BKDEdit")
        self.controlLayout.addWidget(self.BKDEdit)
        self.BKDSlider = QtWidgets.QSlider(self.verticalLayoutWidget)
        self.BKDSlider.setOrientation(QtCore.Qt.Horizontal)
        self.BKDSlider.setObjectName("BKDSlider")
        self.controlLayout.addWidget(self.BKDSlider)
        self.label_3 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.label_3.setText("VKP")
        self.controlLayout.addWidget(self.label_3)
        self.VKPEdit = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.VKPEdit.setObjectName("VKPEdit")
        self.controlLayout.addWidget(self.VKPEdit)
        self.VKPSlider = QtWidgets.QSlider(self.verticalLayoutWidget)
        self.VKPSlider.setOrientation(QtCore.Qt.Horizontal)
        self.VKPSlider.setObjectName("VKPSlider")
        self.controlLayout.addWidget(self.VKPSlider)
        self.label_4 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.label_4.setText("VKI")
        self.controlLayout.addWidget(self.label_4)
        self.VKIEdit = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.VKIEdit.setObjectName("VKIEdit")
        self.controlLayout.addWidget(self.VKIEdit)
        self.VKISlider = QtWidgets.QSlider(self.verticalLayoutWidget)
        self.VKISlider.setOrientation(QtCore.Qt.Horizontal)
        self.VKISlider.setObjectName("VKISlider")
        self.controlLayout.addWidget(self.VKISlider)
        self.label_5 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_5.setObjectName("label_5")
        self.label_5.setText("VKDEdit")
        self.controlLayout.addWidget(self.label_5)
        self.VKDEdit = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.VKDEdit.setObjectName("VKDEdit")
        self.controlLayout.addWidget(self.VKDEdit)
        self.VKDSlider = QtWidgets.QSlider(self.verticalLayoutWidget)
        self.VKDSlider.setOrientation(QtCore.Qt.Horizontal)
        self.VKDSlider.setObjectName("VKDSlider")
        self.controlLayout.addWidget(self.VKDSlider)
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(10, 480, 581, 241))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.commandLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.commandLayout.setContentsMargins(0, 0, 0, 0)
        self.commandLayout.setObjectName("commandLayout")
        self.commandBrowser = QtWidgets.QTextBrowser(self.verticalLayoutWidget_2)
        self.commandBrowser.setObjectName("commandBrowser")
        self.commandLayout.addWidget(self.commandBrowser)
        self.CommandEdit = QtWidgets.QLineEdit(self.verticalLayoutWidget_2)
        self.CommandEdit.setText("")
        self.CommandEdit.setObjectName("CommandEdit")
        self.commandLayout.addWidget(self.CommandEdit)

        #set the
        self.setSlider()

    def setupSlot(self):
        #Slot connect#
        self.Connect.clicked.connect(self.SockConnect)
        self.CommandEdit.editingFinished.connect(self.WriteCommand)
        # self.disConnect.click.connect(self.SockDisConnect)
        #Value changed in Edit
        self.BKPEdit.editingFinished.connect(self.BKPEdit2Slider)
        self.BKDEdit.editingFinished.connect(self.BKDEdit2Slider)
        self.VKPEdit.editingFinished.connect(self.VKPEdit2Slider)
        self.VKIEdit.editingFinished.connect(self.VKIEdit2Slider)
        self.VKDEdit.editingFinished.connect(self.VKDEdit2Slider)

        #Value changed in Slider
        self.BKPSlider.valueChanged.connect(self.BKPSlider2Edit)
        self.BKDSlider.valueChanged.connect(self.BKDSlider2Edit)
        self.VKPSlider.valueChanged.connect(self.VKPSlider2Edit)
        self.VKISlider.valueChanged.connect(self.VKISlider2Edit)
        self.VKDSlider.valueChanged.connect(self.VKDSlider2Edit)

    def setSlider(self):
        #initialize slider paras
        self.BKDSlider.setRange(-10,10)
        self.BKPSlider.setRange(-10,10)
        self.VKPSlider.setRange(-20,20)
        self.VKDSlider.setRange(-20,20)
        self.VKISlider.setRange(-20,20)
        self.BKPSlider.setTickPosition(QSlider.TicksBelow)
        self.BKDSlider.setTickPosition(QSlider.TicksBelow)
        self.VKPSlider.setTickPosition(QSlider.TicksBelow)
        self.VKISlider.setTickPosition(QSlider.TicksBelow)
        self.VKDSlider.setTickPosition(QSlider.TicksBelow)
        self.BKDSlider.setTickInterval(2)
        self.BKPSlider.setTickInterval(2)
        self.VKPSlider.setTickInterval(2)
        self.VKISlider.setTickInterval(2)
        self.VKDSlider.setTickInterval(2)

    def SockConnect(self):
        if self.isConnect ==True:
            return False
        else:
            IP ='101.132.151.237'
            Port = 81
            self.sock.connectToHost(IP,Port)
            if not self.sock.waitForConnected(2500):
                self.commandBrowser.setText("连接失败\n")
                return False

            else:
                print('writing')
                data = QByteArray()
                stream = QDataStream()
                txstring = 'bala_control 2222'
                self.sock.write(txstring.encode('utf-8'))
                self.sock.waitForBytesWritten()
            #
            self.commandBrowser.insertPlainText("连接成功!\n")
            self.isConnect =True
            return True

    def WriteCommand(self):
        #get text from Lineedit and show in browser
        self.commandBrowser.insertPlainText('Wrote: '+self.CommandEdit.text()+'\n')
        #check command
        self.checkcmd(self.CommandEdit.text())
        self.CommandEdit.clear()

    def checkcmd(self,Message):
        #去除空格
        Message.replace(' ','')
        if Message =='clear':
            self.commandBrowser.clear()
        elif Message =='connect':
            self.SockConnect()

        else:
            if self.isConnect ==True:
                self.sock.write(Message.encode('utf-8'))




    def BKPEdit2Slider(self):
        value = float(self.BKPEdit.text())
        self.BKPSlider.setValue(value)

    def BKDEdit2Slider(self):
        value = float(self.BKDEdit.text())
        self.BKDSlider.setValue(value)


    def VKPEdit2Slider(self):
        value = float(self.VKPEdit.text())
        self.VKPSlider.setValue(value)

    def VKIEdit2Slider(self):
        value = float(self.VKIEdit.text())
        self.VKISlider.setValue(value)

    def VKDEdit2Slider(self):
        value = float(self.VKDEdit.text())
        self.VKDSlider.setValue(value)

    def BKPSlider2Edit(self):
        value = self.BKPSlider.value()
        self.BKPEdit.setText(str(value))

    def BKDSlider2Edit(self):
        value = self.BKDSlider.value()
        self.BKDEdit.setText(str(value))

    def VKPSlider2Edit(self):
        value = self.VKPSlider.value()
        self.VKPEdit.setText(str(value))

    def VKISlider2Edit(self):
        value = self.VKISlider.value()
        self.VKIEdit.setText(str(value))

    def VKDSlider2Edit(self):
        value = self.VKDSlider.value()
        self.VKDEdit.setText(str(value))




app = QtWidgets.QApplication(sys.argv)
ui = MainWindow()
ui.show()
sys.exit(app.exec_())