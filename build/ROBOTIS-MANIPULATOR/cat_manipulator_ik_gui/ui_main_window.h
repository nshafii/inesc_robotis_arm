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
    QPushButton *closegripper_button;
    QPushButton *opengripper_button;
    QFrame *IK_frame;
    QVBoxLayout *verticalLayout_9;
    QGroupBox *currentEEFpose;
    QGridLayout *gridLayout_5;
    QPushButton *getEEF_button;
    QPushButton *getTagPose_button;
    QGroupBox *IK;
    QVBoxLayout *verticalLayout_10;
    QLabel *eef_posx;
    QDoubleSpinBox *eef_pos_x_spinbox;
    QLabel *eef_posy;
    QDoubleSpinBox *eef_pos_y_spinbox;
    QLabel *eef_posz;
    QDoubleSpinBox *eef_pos_z_spinbox;
    QLabel *eef_zyxangle_z;
    QDoubleSpinBox *eef_zyxangle_z_spinbox;
    QLabel *eef_zyxangle_y;
    QDoubleSpinBox *eef_zyxangle_y_spinbox;
    QLabel *eef_zyxangle_x;
    QDoubleSpinBox *eef_zyxangle_x_spinbox;
    QPushButton *setEEF_button;
    QLabel *label;
    QPushButton *move_gotoPos1_button;
    QPushButton *move_pickX_button;
    QPushButton *move_pickY_button;
    QPushButton *move_pickYback_button;
    QSpacerItem *verticalSpacer_4;
    QPushButton *move_gobackdroptable_button;
    QFrame *FK_frame;
    QVBoxLayout *verticalLayout_7;
    QGroupBox *currentJointState_theta;
    QGridLayout *gridLayout_4;
    QPushButton *getJointState_button;
    QGroupBox *FK_joints;
    QVBoxLayout *verticalLayout_8;
    QLabel *joint1_theta;
    QDoubleSpinBox *joint1_theta_spinbox;
    QLabel *joint2_theta;
    QDoubleSpinBox *joint2_theta_spinbox;
    QLabel *joint3_theta;
    QDoubleSpinBox *joint3_theta_spinbox;
    QLabel *joint4_theta;
    QDoubleSpinBox *joint4_theta_spinbox;
    QLabel *joint5_theta;
    QDoubleSpinBox *joint5_theta_spinbox;
    QLabel *joint6_theta;
    QDoubleSpinBox *joint6_theta_spinbox;
    QPushButton *setJointState_button;
    QPushButton *publishJointState_button;
    QLabel *label_2;
    QPushButton *move_gotoPos2_button;
    QPushButton *move_gotoPos3_button;
    QPushButton *move_drop_button;
    QSpacerItem *verticalSpacer_3;
    QPushButton *move_gobackdroprobot_button;
    QPushButton *move_gotoRest_button;
    QMenuBar *menubar;
    QMenu *menu_File;
    QMenu *menuCat_Manipulator_Ik_Gui;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(944, 782);
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

        closegripper_button = new QPushButton(tab_status);
        closegripper_button->setObjectName(QString::fromUtf8("closegripper_button"));

        verticalLayout_2->addWidget(closegripper_button);

        opengripper_button = new QPushButton(tab_status);
        opengripper_button->setObjectName(QString::fromUtf8("opengripper_button"));

        verticalLayout_2->addWidget(opengripper_button);

        tab_manager->addTab(tab_status, QString());

        hboxLayout->addWidget(tab_manager);

        IK_frame = new QFrame(centralwidget);
        IK_frame->setObjectName(QString::fromUtf8("IK_frame"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(IK_frame->sizePolicy().hasHeightForWidth());
        IK_frame->setSizePolicy(sizePolicy1);
        IK_frame->setFrameShape(QFrame::StyledPanel);
        IK_frame->setFrameShadow(QFrame::Raised);
        verticalLayout_9 = new QVBoxLayout(IK_frame);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        currentEEFpose = new QGroupBox(IK_frame);
        currentEEFpose->setObjectName(QString::fromUtf8("currentEEFpose"));
        gridLayout_5 = new QGridLayout(currentEEFpose);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        getEEF_button = new QPushButton(currentEEFpose);
        getEEF_button->setObjectName(QString::fromUtf8("getEEF_button"));

        gridLayout_5->addWidget(getEEF_button, 0, 0, 1, 1);

        getTagPose_button = new QPushButton(currentEEFpose);
        getTagPose_button->setObjectName(QString::fromUtf8("getTagPose_button"));

        gridLayout_5->addWidget(getTagPose_button, 0, 1, 1, 1);


        verticalLayout_9->addWidget(currentEEFpose);

        IK = new QGroupBox(IK_frame);
        IK->setObjectName(QString::fromUtf8("IK"));
        verticalLayout_10 = new QVBoxLayout(IK);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        eef_posx = new QLabel(IK);
        eef_posx->setObjectName(QString::fromUtf8("eef_posx"));

        verticalLayout_10->addWidget(eef_posx);

        eef_pos_x_spinbox = new QDoubleSpinBox(IK);
        eef_pos_x_spinbox->setObjectName(QString::fromUtf8("eef_pos_x_spinbox"));
        eef_pos_x_spinbox->setDecimals(3);
        eef_pos_x_spinbox->setMinimum(-10);
        eef_pos_x_spinbox->setMaximum(10);
        eef_pos_x_spinbox->setSingleStep(0.01);

        verticalLayout_10->addWidget(eef_pos_x_spinbox);

        eef_posy = new QLabel(IK);
        eef_posy->setObjectName(QString::fromUtf8("eef_posy"));

        verticalLayout_10->addWidget(eef_posy);

        eef_pos_y_spinbox = new QDoubleSpinBox(IK);
        eef_pos_y_spinbox->setObjectName(QString::fromUtf8("eef_pos_y_spinbox"));
        eef_pos_y_spinbox->setDecimals(3);
        eef_pos_y_spinbox->setMinimum(-10);
        eef_pos_y_spinbox->setMaximum(10);
        eef_pos_y_spinbox->setSingleStep(0.01);

        verticalLayout_10->addWidget(eef_pos_y_spinbox);

        eef_posz = new QLabel(IK);
        eef_posz->setObjectName(QString::fromUtf8("eef_posz"));

        verticalLayout_10->addWidget(eef_posz);

        eef_pos_z_spinbox = new QDoubleSpinBox(IK);
        eef_pos_z_spinbox->setObjectName(QString::fromUtf8("eef_pos_z_spinbox"));
        eef_pos_z_spinbox->setDecimals(3);
        eef_pos_z_spinbox->setMinimum(-10);
        eef_pos_z_spinbox->setMaximum(10);
        eef_pos_z_spinbox->setSingleStep(0.01);

        verticalLayout_10->addWidget(eef_pos_z_spinbox);

        eef_zyxangle_z = new QLabel(IK);
        eef_zyxangle_z->setObjectName(QString::fromUtf8("eef_zyxangle_z"));

        verticalLayout_10->addWidget(eef_zyxangle_z);

        eef_zyxangle_z_spinbox = new QDoubleSpinBox(IK);
        eef_zyxangle_z_spinbox->setObjectName(QString::fromUtf8("eef_zyxangle_z_spinbox"));
        eef_zyxangle_z_spinbox->setDecimals(3);
        eef_zyxangle_z_spinbox->setMinimum(-360);
        eef_zyxangle_z_spinbox->setMaximum(360);

        verticalLayout_10->addWidget(eef_zyxangle_z_spinbox);

        eef_zyxangle_y = new QLabel(IK);
        eef_zyxangle_y->setObjectName(QString::fromUtf8("eef_zyxangle_y"));

        verticalLayout_10->addWidget(eef_zyxangle_y);

        eef_zyxangle_y_spinbox = new QDoubleSpinBox(IK);
        eef_zyxangle_y_spinbox->setObjectName(QString::fromUtf8("eef_zyxangle_y_spinbox"));
        eef_zyxangle_y_spinbox->setDecimals(3);
        eef_zyxangle_y_spinbox->setMinimum(-360);
        eef_zyxangle_y_spinbox->setMaximum(360);

        verticalLayout_10->addWidget(eef_zyxangle_y_spinbox);

        eef_zyxangle_x = new QLabel(IK);
        eef_zyxangle_x->setObjectName(QString::fromUtf8("eef_zyxangle_x"));

        verticalLayout_10->addWidget(eef_zyxangle_x);

        eef_zyxangle_x_spinbox = new QDoubleSpinBox(IK);
        eef_zyxangle_x_spinbox->setObjectName(QString::fromUtf8("eef_zyxangle_x_spinbox"));
        eef_zyxangle_x_spinbox->setDecimals(3);
        eef_zyxangle_x_spinbox->setMinimum(-360);
        eef_zyxangle_x_spinbox->setMaximum(360);

        verticalLayout_10->addWidget(eef_zyxangle_x_spinbox);

        setEEF_button = new QPushButton(IK);
        setEEF_button->setObjectName(QString::fromUtf8("setEEF_button"));

        verticalLayout_10->addWidget(setEEF_button);

        label = new QLabel(IK);
        label->setObjectName(QString::fromUtf8("label"));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setItalic(true);
        font.setWeight(75);
        label->setFont(font);

        verticalLayout_10->addWidget(label);

        move_gotoPos1_button = new QPushButton(IK);
        move_gotoPos1_button->setObjectName(QString::fromUtf8("move_gotoPos1_button"));

        verticalLayout_10->addWidget(move_gotoPos1_button);

        move_pickX_button = new QPushButton(IK);
        move_pickX_button->setObjectName(QString::fromUtf8("move_pickX_button"));

        verticalLayout_10->addWidget(move_pickX_button);

        move_pickY_button = new QPushButton(IK);
        move_pickY_button->setObjectName(QString::fromUtf8("move_pickY_button"));

        verticalLayout_10->addWidget(move_pickY_button);

        move_pickYback_button = new QPushButton(IK);
        move_pickYback_button->setObjectName(QString::fromUtf8("move_pickYback_button"));

        verticalLayout_10->addWidget(move_pickYback_button);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_10->addItem(verticalSpacer_4);

        move_gobackdroptable_button = new QPushButton(IK);
        move_gobackdroptable_button->setObjectName(QString::fromUtf8("move_gobackdroptable_button"));

        verticalLayout_10->addWidget(move_gobackdroptable_button);


        verticalLayout_9->addWidget(IK);


        hboxLayout->addWidget(IK_frame);

        FK_frame = new QFrame(centralwidget);
        FK_frame->setObjectName(QString::fromUtf8("FK_frame"));
        sizePolicy1.setHeightForWidth(FK_frame->sizePolicy().hasHeightForWidth());
        FK_frame->setSizePolicy(sizePolicy1);
        FK_frame->setFrameShape(QFrame::StyledPanel);
        FK_frame->setFrameShadow(QFrame::Raised);
        verticalLayout_7 = new QVBoxLayout(FK_frame);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        currentJointState_theta = new QGroupBox(FK_frame);
        currentJointState_theta->setObjectName(QString::fromUtf8("currentJointState_theta"));
        gridLayout_4 = new QGridLayout(currentJointState_theta);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        getJointState_button = new QPushButton(currentJointState_theta);
        getJointState_button->setObjectName(QString::fromUtf8("getJointState_button"));

        gridLayout_4->addWidget(getJointState_button, 0, 0, 1, 1);


        verticalLayout_7->addWidget(currentJointState_theta);

        FK_joints = new QGroupBox(FK_frame);
        FK_joints->setObjectName(QString::fromUtf8("FK_joints"));
        verticalLayout_8 = new QVBoxLayout(FK_joints);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        joint1_theta = new QLabel(FK_joints);
        joint1_theta->setObjectName(QString::fromUtf8("joint1_theta"));

        verticalLayout_8->addWidget(joint1_theta);

        joint1_theta_spinbox = new QDoubleSpinBox(FK_joints);
        joint1_theta_spinbox->setObjectName(QString::fromUtf8("joint1_theta_spinbox"));
        joint1_theta_spinbox->setDecimals(3);
        joint1_theta_spinbox->setMinimum(-10);
        joint1_theta_spinbox->setMaximum(10);
        joint1_theta_spinbox->setSingleStep(0.01);

        verticalLayout_8->addWidget(joint1_theta_spinbox);

        joint2_theta = new QLabel(FK_joints);
        joint2_theta->setObjectName(QString::fromUtf8("joint2_theta"));

        verticalLayout_8->addWidget(joint2_theta);

        joint2_theta_spinbox = new QDoubleSpinBox(FK_joints);
        joint2_theta_spinbox->setObjectName(QString::fromUtf8("joint2_theta_spinbox"));
        joint2_theta_spinbox->setDecimals(3);
        joint2_theta_spinbox->setMinimum(-10);
        joint2_theta_spinbox->setMaximum(10);
        joint2_theta_spinbox->setSingleStep(0.01);

        verticalLayout_8->addWidget(joint2_theta_spinbox);

        joint3_theta = new QLabel(FK_joints);
        joint3_theta->setObjectName(QString::fromUtf8("joint3_theta"));

        verticalLayout_8->addWidget(joint3_theta);

        joint3_theta_spinbox = new QDoubleSpinBox(FK_joints);
        joint3_theta_spinbox->setObjectName(QString::fromUtf8("joint3_theta_spinbox"));
        joint3_theta_spinbox->setDecimals(3);
        joint3_theta_spinbox->setMinimum(-10);
        joint3_theta_spinbox->setMaximum(10);
        joint3_theta_spinbox->setSingleStep(0.01);

        verticalLayout_8->addWidget(joint3_theta_spinbox);

        joint4_theta = new QLabel(FK_joints);
        joint4_theta->setObjectName(QString::fromUtf8("joint4_theta"));

        verticalLayout_8->addWidget(joint4_theta);

        joint4_theta_spinbox = new QDoubleSpinBox(FK_joints);
        joint4_theta_spinbox->setObjectName(QString::fromUtf8("joint4_theta_spinbox"));
        joint4_theta_spinbox->setDecimals(3);
        joint4_theta_spinbox->setMinimum(-360);
        joint4_theta_spinbox->setMaximum(360);

        verticalLayout_8->addWidget(joint4_theta_spinbox);

        joint5_theta = new QLabel(FK_joints);
        joint5_theta->setObjectName(QString::fromUtf8("joint5_theta"));

        verticalLayout_8->addWidget(joint5_theta);

        joint5_theta_spinbox = new QDoubleSpinBox(FK_joints);
        joint5_theta_spinbox->setObjectName(QString::fromUtf8("joint5_theta_spinbox"));
        joint5_theta_spinbox->setDecimals(3);
        joint5_theta_spinbox->setMinimum(-360);
        joint5_theta_spinbox->setMaximum(360);

        verticalLayout_8->addWidget(joint5_theta_spinbox);

        joint6_theta = new QLabel(FK_joints);
        joint6_theta->setObjectName(QString::fromUtf8("joint6_theta"));

        verticalLayout_8->addWidget(joint6_theta);

        joint6_theta_spinbox = new QDoubleSpinBox(FK_joints);
        joint6_theta_spinbox->setObjectName(QString::fromUtf8("joint6_theta_spinbox"));
        joint6_theta_spinbox->setDecimals(3);
        joint6_theta_spinbox->setMinimum(-360);
        joint6_theta_spinbox->setMaximum(360);

        verticalLayout_8->addWidget(joint6_theta_spinbox);

        setJointState_button = new QPushButton(FK_joints);
        setJointState_button->setObjectName(QString::fromUtf8("setJointState_button"));

        verticalLayout_8->addWidget(setJointState_button);

        publishJointState_button = new QPushButton(FK_joints);
        publishJointState_button->setObjectName(QString::fromUtf8("publishJointState_button"));

        verticalLayout_8->addWidget(publishJointState_button);

        label_2 = new QLabel(FK_joints);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setFont(font);

        verticalLayout_8->addWidget(label_2);

        move_gotoPos2_button = new QPushButton(FK_joints);
        move_gotoPos2_button->setObjectName(QString::fromUtf8("move_gotoPos2_button"));

        verticalLayout_8->addWidget(move_gotoPos2_button);

        move_gotoPos3_button = new QPushButton(FK_joints);
        move_gotoPos3_button->setObjectName(QString::fromUtf8("move_gotoPos3_button"));

        verticalLayout_8->addWidget(move_gotoPos3_button);

        move_drop_button = new QPushButton(FK_joints);
        move_drop_button->setObjectName(QString::fromUtf8("move_drop_button"));

        verticalLayout_8->addWidget(move_drop_button);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_8->addItem(verticalSpacer_3);

        move_gobackdroprobot_button = new QPushButton(FK_joints);
        move_gobackdroprobot_button->setObjectName(QString::fromUtf8("move_gobackdroprobot_button"));

        verticalLayout_8->addWidget(move_gobackdroprobot_button);

        move_gotoRest_button = new QPushButton(FK_joints);
        move_gotoRest_button->setObjectName(QString::fromUtf8("move_gotoRest_button"));

        verticalLayout_8->addWidget(move_gotoRest_button);


        verticalLayout_7->addWidget(FK_joints);


        hboxLayout->addWidget(FK_frame);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 944, 25));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        menuCat_Manipulator_Ik_Gui = new QMenu(menubar);
        menuCat_Manipulator_Ik_Gui->setObjectName(QString::fromUtf8("menuCat_Manipulator_Ik_Gui"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);

        menubar->addAction(menu_File->menuAction());
        menubar->addAction(menuCat_Manipulator_Ik_Gui->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addAction(actionAbout_Qt);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));

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
        closegripper_button->setText(QApplication::translate("MainWindowDesign", "Close Gripper", 0, QApplication::UnicodeUTF8));
        opengripper_button->setText(QApplication::translate("MainWindowDesign", "Open Gripper", 0, QApplication::UnicodeUTF8));
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindowDesign", "Ros Communications", 0, QApplication::UnicodeUTF8));
        currentEEFpose->setTitle(QApplication::translate("MainWindowDesign", "EEF", 0, QApplication::UnicodeUTF8));
        getEEF_button->setText(QApplication::translate("MainWindowDesign", "Update EEF", 0, QApplication::UnicodeUTF8));
        getTagPose_button->setText(QApplication::translate("MainWindowDesign", "Get Tag Pose", 0, QApplication::UnicodeUTF8));
        IK->setTitle(QApplication::translate("MainWindowDesign", "EEF POSE", 0, QApplication::UnicodeUTF8));
        eef_posx->setText(QApplication::translate("MainWindowDesign", "pos_x", 0, QApplication::UnicodeUTF8));
        eef_posy->setText(QApplication::translate("MainWindowDesign", "pos_y", 0, QApplication::UnicodeUTF8));
        eef_posz->setText(QApplication::translate("MainWindowDesign", "pos_z", 0, QApplication::UnicodeUTF8));
        eef_zyxangle_z->setText(QApplication::translate("MainWindowDesign", "z angle (zyx)", 0, QApplication::UnicodeUTF8));
        eef_zyxangle_y->setText(QApplication::translate("MainWindowDesign", "y angle (zyx)", 0, QApplication::UnicodeUTF8));
        eef_zyxangle_x->setText(QApplication::translate("MainWindowDesign", "x angle (zyx)", 0, QApplication::UnicodeUTF8));
        setEEF_button->setText(QApplication::translate("MainWindowDesign", "Set Desired Position", 0, QApplication::UnicodeUTF8));
        label->setText(QString());
        move_gotoPos1_button->setText(QApplication::translate("MainWindowDesign", "Table top", 0, QApplication::UnicodeUTF8));
        move_pickX_button->setText(QApplication::translate("MainWindowDesign", "Align X", 0, QApplication::UnicodeUTF8));
        move_pickY_button->setText(QApplication::translate("MainWindowDesign", "Align Y", 0, QApplication::UnicodeUTF8));
        move_pickYback_button->setText(QApplication::translate("MainWindowDesign", "Go back Y", 0, QApplication::UnicodeUTF8));
        move_gobackdroptable_button->setText(QApplication::translate("MainWindowDesign", "Go Back Drop Table", 0, QApplication::UnicodeUTF8));
        currentJointState_theta->setTitle(QApplication::translate("MainWindowDesign", "Joints", 0, QApplication::UnicodeUTF8));
        getJointState_button->setText(QApplication::translate("MainWindowDesign", "Get current Joint States", 0, QApplication::UnicodeUTF8));
        FK_joints->setTitle(QString());
        joint1_theta->setText(QApplication::translate("MainWindowDesign", "Joint 1 (-3.14 <-> 2.83)", 0, QApplication::UnicodeUTF8));
        joint2_theta->setText(QApplication::translate("MainWindowDesign", "Joint 2(-1.41 <-> 1.41)", 0, QApplication::UnicodeUTF8));
        joint3_theta->setText(QApplication::translate("MainWindowDesign", "Joint 3 (-1.26 <-> 2.20)", 0, QApplication::UnicodeUTF8));
        joint4_theta->setText(QApplication::translate("MainWindowDesign", "Joint 4 (-1.41 <-> 1.41)", 0, QApplication::UnicodeUTF8));
        joint5_theta->setText(QApplication::translate("MainWindowDesign", "Joint 5 (-1.41 <-> 1.41)", 0, QApplication::UnicodeUTF8));
        joint6_theta->setText(QApplication::translate("MainWindowDesign", "Joint 6 (-1.88 <-> 1.88)", 0, QApplication::UnicodeUTF8));
        setJointState_button->setText(QApplication::translate("MainWindowDesign", "Set Joint Values", 0, QApplication::UnicodeUTF8));
        publishJointState_button->setText(QApplication::translate("MainWindowDesign", "Publish joints", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindowDesign", "robot table", 0, QApplication::UnicodeUTF8));
        move_gotoPos2_button->setText(QApplication::translate("MainWindowDesign", "Up table", 0, QApplication::UnicodeUTF8));
        move_gotoPos3_button->setText(QApplication::translate("MainWindowDesign", "Up table robot", 0, QApplication::UnicodeUTF8));
        move_drop_button->setText(QApplication::translate("MainWindowDesign", "Table Robot", 0, QApplication::UnicodeUTF8));
        move_gobackdroprobot_button->setText(QApplication::translate("MainWindowDesign", "Go Back Drop Robot", 0, QApplication::UnicodeUTF8));
        move_gotoRest_button->setText(QApplication::translate("MainWindowDesign", "REST", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&App", 0, QApplication::UnicodeUTF8));
        menuCat_Manipulator_Ik_Gui->setTitle(QApplication::translate("MainWindowDesign", "Cat_Manipulator_Ik_Gui", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
