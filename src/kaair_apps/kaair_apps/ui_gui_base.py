# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'gui_baseoaTncW.ui'
##
## Created by: Qt User Interface Compiler version 6.11.0
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
from PySide6.QtWidgets import (QApplication, QFrame, QLabel, QMainWindow,
    QPushButton, QSizePolicy, QSpacerItem, QStackedWidget,
    QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1167, 617)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.menu_frame = QFrame(self.centralwidget)
        self.menu_frame.setObjectName(u"menu_frame")
        self.menu_frame.setGeometry(QRect(0, 0, 100, 617))
        self.menu_frame.setStyleSheet(u"background-color: #2c3e50;")
        self.verticalLayout = QVBoxLayout(self.menu_frame)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.btn_home = QPushButton(self.menu_frame)
        self.btn_home.setObjectName(u"btn_home")
        self.btn_home.setMinimumSize(QSize(0, 50))
        self.btn_home.setStyleSheet(u"color: white; font-weight: bold;")

        self.verticalLayout.addWidget(self.btn_home)

        self.btn_arm = QPushButton(self.menu_frame)
        self.btn_arm.setObjectName(u"btn_arm")
        self.btn_arm.setMinimumSize(QSize(0, 50))
        self.btn_arm.setStyleSheet(u"color: white; font-weight: bold;")

        self.verticalLayout.addWidget(self.btn_arm)

        self.btn_lift = QPushButton(self.menu_frame)
        self.btn_lift.setObjectName(u"btn_lift")
        self.btn_lift.setMinimumSize(QSize(0, 50))
        self.btn_lift.setStyleSheet(u"color: white; font-weight: bold;")

        self.verticalLayout.addWidget(self.btn_lift)

        self.btn_head = QPushButton(self.menu_frame)
        self.btn_head.setObjectName(u"btn_head")
        self.btn_head.setMinimumSize(QSize(0, 50))
        self.btn_head.setStyleSheet(u"color: white; font-weight: bold;")

        self.verticalLayout.addWidget(self.btn_head)

        self.btn_tool = QPushButton(self.menu_frame)
        self.btn_tool.setObjectName(u"btn_tool")
        self.btn_tool.setMinimumSize(QSize(0, 50))
        self.btn_tool.setStyleSheet(u"color: white; font-weight: bold;")

        self.verticalLayout.addWidget(self.btn_tool)

        self.btn_mobile = QPushButton(self.menu_frame)
        self.btn_mobile.setObjectName(u"btn_mobile")
        self.btn_mobile.setMinimumSize(QSize(0, 50))
        self.btn_mobile.setStyleSheet(u"color: white; font-weight: bold;")

        self.verticalLayout.addWidget(self.btn_mobile)

        self.btn_vision = QPushButton(self.menu_frame)
        self.btn_vision.setObjectName(u"btn_vision")
        self.btn_vision.setMinimumSize(QSize(0, 50))
        self.btn_vision.setStyleSheet(u"color: white; font-weight: bold;")

        self.verticalLayout.addWidget(self.btn_vision)

        self.verticalSpacer = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer)

        self.stackedWidget = QStackedWidget(self.centralwidget)
        self.stackedWidget.setObjectName(u"stackedWidget")
        self.stackedWidget.setGeometry(QRect(100, 0, 1067, 617))
        self.page_home = QWidget()
        self.page_home.setObjectName(u"page_home")
        self.vboxLayout = QVBoxLayout(self.page_home)
        self.vboxLayout.setObjectName(u"vboxLayout")
        self.label = QLabel(self.page_home)
        self.label.setObjectName(u"label")
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vboxLayout.addWidget(self.label)

        self.stackedWidget.addWidget(self.page_home)
        self.page_arm = QWidget()
        self.page_arm.setObjectName(u"page_arm")
        self.vboxLayout1 = QVBoxLayout(self.page_arm)
        self.vboxLayout1.setObjectName(u"vboxLayout1")
        self.label1 = QLabel(self.page_arm)
        self.label1.setObjectName(u"label1")
        self.label1.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vboxLayout1.addWidget(self.label1)

        self.stackedWidget.addWidget(self.page_arm)
        self.page_lift = QWidget()
        self.page_lift.setObjectName(u"page_lift")
        self.vboxLayout2 = QVBoxLayout(self.page_lift)
        self.vboxLayout2.setObjectName(u"vboxLayout2")
        self.label2 = QLabel(self.page_lift)
        self.label2.setObjectName(u"label2")
        self.label2.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vboxLayout2.addWidget(self.label2)

        self.stackedWidget.addWidget(self.page_lift)
        self.page_head = QWidget()
        self.page_head.setObjectName(u"page_head")
        self.vboxLayout3 = QVBoxLayout(self.page_head)
        self.vboxLayout3.setObjectName(u"vboxLayout3")
        self.label3 = QLabel(self.page_head)
        self.label3.setObjectName(u"label3")
        self.label3.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vboxLayout3.addWidget(self.label3)

        self.stackedWidget.addWidget(self.page_head)
        self.page_tool = QWidget()
        self.page_tool.setObjectName(u"page_tool")
        self.vboxLayout4 = QVBoxLayout(self.page_tool)
        self.vboxLayout4.setObjectName(u"vboxLayout4")
        self.label4 = QLabel(self.page_tool)
        self.label4.setObjectName(u"label4")
        self.label4.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vboxLayout4.addWidget(self.label4)

        self.stackedWidget.addWidget(self.page_tool)
        self.page_mobile = QWidget()
        self.page_mobile.setObjectName(u"page_mobile")
        self.vboxLayout5 = QVBoxLayout(self.page_mobile)
        self.vboxLayout5.setObjectName(u"vboxLayout5")
        self.label5 = QLabel(self.page_mobile)
        self.label5.setObjectName(u"label5")
        self.label5.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vboxLayout5.addWidget(self.label5)

        self.stackedWidget.addWidget(self.page_mobile)
        self.page_vision = QWidget()
        self.page_vision.setObjectName(u"page_vision")
        self.vboxLayout6 = QVBoxLayout(self.page_vision)
        self.vboxLayout6.setObjectName(u"vboxLayout6")
        self.label6 = QLabel(self.page_vision)
        self.label6.setObjectName(u"label6")
        self.label6.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vboxLayout6.addWidget(self.label6)

        self.stackedWidget.addWidget(self.page_vision)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        self.stackedWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"KAAIR Robot Manager", None))
        self.btn_home.setText(QCoreApplication.translate("MainWindow", u"Home", None))
        self.btn_arm.setText(QCoreApplication.translate("MainWindow", u"Arm", None))
        self.btn_lift.setText(QCoreApplication.translate("MainWindow", u"Lift", None))
        self.btn_head.setText(QCoreApplication.translate("MainWindow", u"Head", None))
        self.btn_tool.setText(QCoreApplication.translate("MainWindow", u"Tool", None))
        self.btn_mobile.setText(QCoreApplication.translate("MainWindow", u"Mobile", None))
        self.btn_vision.setText(QCoreApplication.translate("MainWindow", u"Vision", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"HOME PAGE", None))
        self.label1.setText(QCoreApplication.translate("MainWindow", u"ARM CONTROL PAGE", None))
        self.label2.setText(QCoreApplication.translate("MainWindow", u"LIFT CONTROL PAGE", None))
        self.label3.setText(QCoreApplication.translate("MainWindow", u"HEAD CONTROL PAGE", None))
        self.label4.setText(QCoreApplication.translate("MainWindow", u"TOOL CONTROL PAGE", None))
        self.label5.setText(QCoreApplication.translate("MainWindow", u"MOBILE CONTROL PAGE", None))
        self.label6.setText(QCoreApplication.translate("MainWindow", u"VISION PAGE", None))
    # retranslateUi

