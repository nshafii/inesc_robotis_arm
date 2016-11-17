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
    QStatusBar *statusbar;
    QDockWidget *dock_status;
    QWidget *dockWidgetContents_2;
    QVBoxLayout *verticalLayout;
    QFrame *frame;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *FK;
    QGridLayout *gridLayout;
    QPushButton *currpos_button;
    QGroupBox *IK;
    QVBoxLayout *verticalLayout_4;
    QLabel *pos_x;
    QDoubleSpinBox *pos_x_spinbox;
    QLabel *pos_y;
    QDoubleSpinBox *pos_y_spinbox;
    QLabel *pos_z;
    QDoubleSpinBox *pos_z_spinbox;
    QLabel *ori_x;
    QDoubleSpinBox *ori_x_spinbox;
    QLabel *ori_y;
    QDoubleSpinBox *ori_y_spinbox;
    QLabel *ori_z;
    QDoubleSpinBox *ori_z_spinbox;
    QLabel *ori_w;
    QDoubleSpinBox *ori_w_spinbox;
    QPushButton *despos_button;
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
        dock_status->setMinimumSize(QSize(325, 626));
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
        FK = new QGroupBox(frame);
        FK->setObjectName(QString::fromUtf8("FK"));
        gridLayout = new QGridLayout(FK);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        currpos_button = new QPushButton(FK);
        currpos_button->setObjectName(QString::fromUtf8("currpos_button"));

        gridLayout->addWidget(currpos_button, 0, 0, 1, 1);


        verticalLayout_3->addWidget(FK);

        IK = new QGroupBox(frame);
        IK->setObjectName(QString::fromUtf8("IK"));
        verticalLayout_4 = new QVBoxLayout(IK);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        pos_x = new QLabel(IK);
        pos_x->setObjectName(QString::fromUtf8("pos_x"));

        verticalLayout_4->addWidget(pos_x);

        pos_x_spinbox = new QDoubleSpinBox(IK);
        pos_x_spinbox->setObjectName(QString::fromUtf8("pos_x_spinbox"));
        pos_x_spinbox->setDecimals(3);
        pos_x_spinbox->setMinimum(-10);
        pos_x_spinbox->setMaximum(10);
        pos_x_spinbox->setSingleStep(0.01);

        verticalLayout_4->addWidget(pos_x_spinbox);

        pos_y = new QLabel(IK);
        pos_y->setObjectName(QString::fromUtf8("pos_y"));

        verticalLayout_4->addWidget(pos_y);

        pos_y_spinbox = new QDoubleSpinBox(IK);
        pos_y_spinbox->setObjectName(QString::fromUtf8("pos_y_spinbox"));
        pos_y_spinbox->setDecimals(3);
        pos_y_spinbox->setMinimum(-10);
        pos_y_spinbox->setMaximum(10);
        pos_y_spinbox->setSingleStep(0.01);

        verticalLayout_4->addWidget(pos_y_spinbox);

        pos_z = new QLabel(IK);
        pos_z->setObjectName(QString::fromUtf8("pos_z"));

        verticalLayout_4->addWidget(pos_z);

        pos_z_spinbox = new QDoubleSpinBox(IK);
        pos_z_spinbox->setObjectName(QString::fromUtf8("pos_z_spinbox"));
        pos_z_spinbox->setDecimals(3);
        pos_z_spinbox->setMinimum(-10);
        pos_z_spinbox->setMaximum(10);
        pos_z_spinbox->setSingleStep(0.01);

        verticalLayout_4->addWidget(pos_z_spinbox);

        ori_x = new QLabel(IK);
        ori_x->setObjectName(QString::fromUtf8("ori_x"));

        verticalLayout_4->addWidget(ori_x);

        ori_x_spinbox = new QDoubleSpinBox(IK);
        ori_x_spinbox->setObjectName(QString::fromUtf8("ori_x_spinbox"));
        ori_x_spinbox->setDecimals(3);
        ori_x_spinbox->setMinimum(-360);
        ori_x_spinbox->setMaximum(360);

        verticalLayout_4->addWidget(ori_x_spinbox);

        ori_y = new QLabel(IK);
        ori_y->setObjectName(QString::fromUtf8("ori_y"));

        verticalLayout_4->addWidget(ori_y);

        ori_y_spinbox = new QDoubleSpinBox(IK);
        ori_y_spinbox->setObjectName(QString::fromUtf8("ori_y_spinbox"));
        ori_y_spinbox->setDecimals(3);
        ori_y_spinbox->setMinimum(-360);
        ori_y_spinbox->setMaximum(360);

        verticalLayout_4->addWidget(ori_y_spinbox);

        ori_z = new QLabel(IK);
        ori_z->setObjectName(QString::fromUtf8("ori_z"));

        verticalLayout_4->addWidget(ori_z);

        ori_z_spinbox = new QDoubleSpinBox(IK);
        ori_z_spinbox->setObjectName(QString::fromUtf8("ori_z_spinbox"));
        ori_z_spinbox->setDecimals(3);
        ori_z_spinbox->setMinimum(-360);
        ori_z_spinbox->setMaximum(360);

        verticalLayout_4->addWidget(ori_z_spinbox);

        ori_w = new QLabel(IK);
        ori_w->setObjectName(QString::fromUtf8("ori_w"));

        verticalLayout_4->addWidget(ori_w);

        ori_w_spinbox = new QDoubleSpinBox(IK);
        ori_w_spinbox->setObjectName(QString::fromUtf8("ori_w_spinbox"));
        ori_w_spinbox->setDecimals(3);
        ori_w_spinbox->setMinimum(-360);
        ori_w_spinbox->setMaximum(360);

        verticalLayout_4->addWidget(ori_w_spinbox);

        despos_button = new QPushButton(IK);
        despos_button->setObjectName(QString::fromUtf8("despos_button"));

        verticalLayout_4->addWidget(despos_button);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer);


        verticalLayout_3->addWidget(IK);


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
        dock_status->setWindowTitle(QApplication::translate("MainWindowDesign", "Command Panel", 0, QApplication::UnicodeUTF8));
        FK->setTitle(QApplication::translate("MainWindowDesign", "Forward Kinematics", 0, QApplication::UnicodeUTF8));
        currpos_button->setText(QApplication::translate("MainWindowDesign", "curr_pos", 0, QApplication::UnicodeUTF8));
        IK->setTitle(QApplication::translate("MainWindowDesign", "Inverse Kinematics", 0, QApplication::UnicodeUTF8));
        pos_x->setText(QApplication::translate("MainWindowDesign", "position x [m]", 0, QApplication::UnicodeUTF8));
        pos_y->setText(QApplication::translate("MainWindowDesign", "position y [m]", 0, QApplication::UnicodeUTF8));
        pos_z->setText(QApplication::translate("MainWindowDesign", "position z [m]", 0, QApplication::UnicodeUTF8));
        ori_x->setText(QApplication::translate("MainWindowDesign", "ori_x", 0, QApplication::UnicodeUTF8));
        ori_y->setText(QApplication::translate("MainWindowDesign", "ori_y", 0, QApplication::UnicodeUTF8));
        ori_z->setText(QApplication::translate("MainWindowDesign", "ori_z", 0, QApplication::UnicodeUTF8));
        ori_w->setText(QApplication::translate("MainWindowDesign", "ori w", 0, QApplication::UnicodeUTF8));
        despos_button->setText(QApplication::translate("MainWindowDesign", "des_pos", 0, QApplication::UnicodeUTF8));
        quit_button->setText(QApplication::translate("MainWindowDesign", "Quit", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
