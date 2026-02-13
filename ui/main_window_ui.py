# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main_window.ui'
##
## Created by: Qt User Interface Compiler version 6.10.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QLabel, QListWidget, QListWidgetItem,
    QMainWindow, QMenuBar, QPushButton, QSizePolicy,
    QStatusBar, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.btn_go = QPushButton(self.centralwidget)
        self.btn_go.setObjectName(u"btn_go")
        self.btn_go.setGeometry(QRect(90, 100, 71, 41))
        self.btn_left = QPushButton(self.centralwidget)
        self.btn_left.setObjectName(u"btn_left")
        self.btn_left.setGeometry(QRect(20, 140, 71, 41))
        self.btn_right = QPushButton(self.centralwidget)
        self.btn_right.setObjectName(u"btn_right")
        self.btn_right.setGeometry(QRect(160, 140, 71, 41))
        self.btn_back = QPushButton(self.centralwidget)
        self.btn_back.setObjectName(u"btn_back")
        self.btn_back.setGeometry(QRect(90, 180, 71, 41))
        self.btn_stop = QPushButton(self.centralwidget)
        self.btn_stop.setObjectName(u"btn_stop")
        self.btn_stop.setGeometry(QRect(90, 140, 71, 41))
        self.listWidget = QListWidget(self.centralwidget)
        self.listWidget.setObjectName(u"listWidget")
        self.listWidget.setGeometry(QRect(250, 60, 256, 192))
        self.label_pos_x = QLabel(self.centralwidget)
        self.label_pos_x.setObjectName(u"label_pos_x")
        self.label_pos_x.setGeometry(QRect(30, 280, 67, 17))
        self.label_pos_y = QLabel(self.centralwidget)
        self.label_pos_y.setObjectName(u"label_pos_y")
        self.label_pos_y.setGeometry(QRect(30, 330, 67, 17))
        self.label_warning = QLabel(self.centralwidget)
        self.label_warning.setObjectName(u"label_warning")
        self.label_warning.setGeometry(QRect(30, 390, 67, 17))
        self.btn_patrol_square = QPushButton(self.centralwidget)
        self.btn_patrol_square.setObjectName(u"btn_patrol_square")
        self.btn_patrol_square.setGeometry(QRect(20, 440, 111, 31))
        self.btn_patrol_triangle = QPushButton(self.centralwidget)
        self.btn_patrol_triangle.setObjectName(u"btn_patrol_triangle")
        self.btn_patrol_triangle.setGeometry(QRect(140, 440, 111, 31))
        self.btn_safety_toggle = QPushButton(self.centralwidget)
        self.btn_safety_toggle.setObjectName(u"btn_safety_toggle")
        self.btn_safety_toggle.setGeometry(QRect(260, 440, 111, 31))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 800, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.btn_go.setText(QCoreApplication.translate("MainWindow", u"GO", None))
        self.btn_left.setText(QCoreApplication.translate("MainWindow", u"LEFT", None))
        self.btn_right.setText(QCoreApplication.translate("MainWindow", u"RIGHT", None))
        self.btn_back.setText(QCoreApplication.translate("MainWindow", u"BACK", None))
        self.btn_stop.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.label_pos_x.setText(QCoreApplication.translate("MainWindow", u"X:", None))
        self.label_pos_y.setText(QCoreApplication.translate("MainWindow", u"Y:", None))
        self.label_warning.setText(QCoreApplication.translate("MainWindow", u"Status", None))
        self.btn_patrol_square.setText(QCoreApplication.translate("MainWindow", u"patrol_square", None))
        self.btn_patrol_triangle.setText(QCoreApplication.translate("MainWindow", u"patrol_triangle", None))
        self.btn_safety_toggle.setText(QCoreApplication.translate("MainWindow", u"safety_toggle", None))
    # retranslateUi

