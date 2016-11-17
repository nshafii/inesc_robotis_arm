/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QTabWidget *tab_manager;
    QWidget *tab_status;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_12;
    QGridLayout *gridLayout_3;
    QListView *view_logging;
    QMenuBar *menubar;
    QMenu *menu_File;
    QMenu *menuCat_Manipulator_Gui;
    QStatusBar *statusbar;
    QDockWidget *dock_status;
    QWidget *dockWidgetContents_2;
    QVBoxLayout *verticalLayout;
    QFrame *frame;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *currentJointState;
    QGridLayout *gridLayout;
    QPushButton *getJointState_button;
    QGroupBox *FK;
    QVBoxLayout *verticalLayout_4;
    QLabel *joint1;
    QDoubleSpinBox *joint1_spinbox;
    QLabel *joint2;
    QDoubleSpinBox *joint2_spinbox;
    QLabel *joint3;
    QDoubleSpinBox *joint3_spinbox;
    QLabel *joint4;
    QDoubleSpinBox *joint4_spinbox;
    QLabel *joint5;
    QDoubleSpinBox *joint5_spinbox;
    QLabel *joint6;
    QDoubleSpinBox *joint6_spinbox;
    QPushButton *setJointState_button;
    QSpacerItem *verticalSpacer;
    QPushButton *quit_button;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(944, 704);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        tab_manager = new QTabWidget(centralwidget);
        tab_manager->setObjectName(QString::fromUtf8("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 0));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_status = new QWidget();
        tab_status->setObjectName(QString::fromUtf8("tab_status"));
        verticalLayout_2 = new QVBoxLayout(tab_status);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        groupBox_12 = new QGroupBox(tab_status);
        groupBox_12->setObjectName(QString::fromUtf8("groupBox_12"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_12->sizePolicy().hasHeightForWidth());
        groupBox_12->setSizePolicy(sizePolicy);
        gridLayout_3 = new QGridLayout(groupBox_12);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        view_logging = new QListView(groupBox_12);
        view_logging->setObjectName(QString::fromUtf8("view_logging"));

        gridLayout_3->addWidget(view_logging, 0, 0, 1, 1);


        verticalLayout_2->addWidget(groupBox_12);

        tab_manager->addTab(tab_status, QString());

        hboxLayout->addWidget(tab_manager);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 944, 25));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        menuCat_Manipulator_Gui = new QMenu(menubar);
        menuCat_Manipulator_Gui->setObjectName(QString::fromUtf8("menuCat_Manipulator_Gui"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);
        dock_status = new QDockWidget(MainWindowDesign);
        dock_status->setObjectName(QString::fromUtf8("dock_status"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(dock_status->sizePolicy().hasHeightForWidth());
        dock_status->setSizePolicy(sizePolicy1);
        dock_status->setMinimumSize(QSize(325, 603));
        dock_status->setAllowedAreas(Qt::RightDockWidgetArea);
        dockWidgetContents_2 = new QWidget();
        dockWidgetContents_2->setObjectName(QString::fromUtf8("dockWidgetContents_2"));
        verticalLayout = new QVBoxLayout(dockWidgetContents_2);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        frame = new QFrame(dockWidgetContents_2);
        frame->setObjectName(QString::fromUtf8("frame"));
        sizePolicy1.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy1);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_3 = new QVBoxLayout(frame);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        currentJointState = new QGroupBox(frame);
        currentJointState->setObjectName(QString::fromUtf8("currentJointState"));
        gridLayout = new QGridLayout(currentJointState);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        getJointState_button = new QPushButton(currentJointState);
        getJointState_button->setObjectName(QString::fromUtf8("getJointState_button"));

        gridLayout->addWidget(getJointState_button, 0, 0, 1, 1);


        verticalLayout_3->addWidget(currentJointState);

        FK = new QGroupBox(frame);
        FK->setObjectName(QString::fromUtf8("FK"));
        verticalLayout_4 = new QVBoxLayout(FK);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        joint1 = new QLabel(FK);
        joint1->setObjectName(QString::fromUtf8("joint1"));

        verticalLayout_4->addWidget(joint1);

        joint1_spinbox = new QDoubleSpinBox(FK);
        joint1_spinbox->setObjectName(QString::fromUtf8("joint1_spinbox"));
        joint1_spinbox->setDecimals(3);
        joint1_spinbox->setMinimum(-10);
        joint1_spinbox->setMaximum(10);
        joint1_spinbox->setSingleStep(0.01);

        verticalLayout_4->addWidget(joint1_spinbox);

        joint2 = new QLabel(FK);
        joint2->setObjectName(QString::fromUtf8("joint2"));

        verticalLayout_4->addWidget(joint2);

        joint2_spinbox = new QDoubleSpinBox(FK);
        joint2_spinbox->setObjectName(QString::fromUtf8("joint2_spinbox"));
        joint2_spinbox->setDecimals(3);
        joint2_spinbox->setMinimum(-10);
        joint2_spinbox->setMaximum(10);
        joint2_spinbox->setSingleStep(0.01);

        verticalLayout_4->addWidget(joint2_spinbox);

        joint3 = new QLabel(FK);
        joint3->setObjectName(QString::fromUtf8("joint3"));

        verticalLayout_4->addWidget(joint3);

        joint3_spinbox = new QDoubleSpinBox(FK);
        joint3_spinbox->setObjectName(QString::fromUtf8("joint3_spinbox"));
        joint3_spinbox->setDecimals(3);
        joint3_spinbox->setMinimum(-10);
        joint3_spinbox->setMaximum(10);
        joint3_spinbox->setSingleStep(0.01);

        verticalLayout_4->addWidget(joint3_spinbox);

        joint4 = new QLabel(FK);
        joint4->setObjectName(QString::fromUtf8("joint4"));

        verticalLayout_4->addWidget(joint4);

        joint4_spinbox = new QDoubleSpinBox(FK);
        joint4_spinbox->setObjectName(QString::fromUtf8("joint4_spinbox"));
        joint4_spinbox->setDecimals(3);
        joint4_spinbox->setMinimum(-360);
        joint4_spinbox->setMaximum(360);

        verticalLayout_4->addWidget(joint4_spinbox);

        joint5 = new QLabel(FK);
        joint5->setObjectName(QString::fromUtf8("joint5"));

        verticalLayout_4->addWidget(joint5);

        joint5_spinbox = new QDoubleSpinBox(FK);
        joint5_spinbox->setObjectName(QString::fromUtf8("joint5_spinbox"));
        joint5_spinbox->setDecimals(3);
        joint5_spinbox->setMinimum(-360);
        joint5_spinbox->setMaximum(360);

        verticalLayout_4->addWidget(joint5_spinbox);

        joint6 = new QLabel(FK);
        joint6->setObjectName(QString::fromUtf8("joint6"));

        verticalLayout_4->addWidget(joint6);

        joint6_spinbox = new QDoubleSpinBox(FK);
        joint6_spinbox->setObjectName(QString::fromUtf8("joint6_spinbox"));
        joint6_spinbox->setDecimals(3);
        joint6_spinbox->setMinimum(-360);
        joint6_spinbox->setMaximum(360);

        verticalLayout_4->addWidget(joint6_spinbox);

        setJointState_button = new QPushButton(FK);
        setJointState_button->setObjectName(QString::fromUtf8("setJointState_button"));

        verticalLayout_4->addWidget(setJointState_button);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer);


        verticalLayout_3->addWidget(FK);


        verticalLayout->addWidget(frame);

        quit_button = new QPushButton(dockWidgetContents_2);
        quit_button->setObjectName(QString::fromUtf8("quit_button"));
        QSizePolicy sizePolicy2(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy2);

        verticalLayout->addWidget(quit_button);

        dock_status->setWidget(dockWidgetContents_2);
        MainWindowDesign->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dock_status);

        menubar->addAction(menu_File->menuAction());
        menubar->addAction(menuCat_Manipulator_Gui->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addAction(actionAbout_Qt);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));
        QObject::connect(quit_button, SIGNAL(clicked()), MainWindowDesign, SLOT(close()));

        tab_manager->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "QRosApp", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
        groupBox_12->setTitle(QApplication::translate("MainWindowDesign", "Logging", 0, QApplication::UnicodeUTF8));
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindowDesign", "Ros Communications", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&App", 0, QApplication::UnicodeUTF8));
        menuCat_Manipulator_Gui->setTitle(QApplication::translate("MainWindowDesign", "Cat_Manipulator_Gui", 0, QApplication::UnicodeUTF8));
        dock_status->setWindowTitle(QApplication::translate("MainWindowDesign", "Command Panel", 0, QApplication::UnicodeUTF8));
        currentJointState->setTitle(QApplication::translate("MainWindowDesign", "Joint States", 0, QApplication::UnicodeUTF8));
        getJointState_button->setText(QApplication::translate("MainWindowDesign", "Get current Joint States", 0, QApplication::UnicodeUTF8));
        FK->setTitle(QApplication::translate("MainWindowDesign", "Forward Kinematics", 0, QApplication::UnicodeUTF8));
        joint1->setText(QApplication::translate("MainWindowDesign", "Joint 1 (-3.14 <-> 2.83)", 0, QApplication::UnicodeUTF8));
        joint2->setText(QApplication::translate("MainWindowDesign", "Joint 2(-1.41 <-> 1.41)", 0, QApplication::UnicodeUTF8));
        joint3->setText(QApplication::translate("MainWindowDesign", "Joint 3 (-1.26 <-> 2.20)", 0, QApplication::UnicodeUTF8));
        joint4->setText(QApplication::translate("MainWindowDesign", "Joint 4 (-1.41 <-> 1.41)", 0, QApplication::UnicodeUTF8));
        joint5->setText(QApplication::translate("MainWindowDesign", "Joint 5 (-1.41 <-> 1.41)", 0, QApplication::UnicodeUTF8));
        joint6->setText(QApplication::translate("MainWindowDesign", "Joint 6 (-1.88 <-> 1.88)", 0, QApplication::UnicodeUTF8));
        setJointState_button->setText(QApplication::translate("MainWindowDesign", "Set Joint Values", 0, QApplication::UnicodeUTF8));
        quit_button->setText(QApplication::translate("MainWindowDesign", "Quit", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
