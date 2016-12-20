/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robotis_manipulator_h_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_manipulator_h_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Logging
    **********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /****************************
    ** Connect
    ****************************/

    QObject::connect(&qnode, SIGNAL(update_curr_pos(double , double , double)), this, SLOT(update_curr_pos_spinbox(double , double , double)));
    QObject::connect(&qnode, SIGNAL(update_curr_ori(double , double , double, double)), this, SLOT(update_curr_ori_spinbox(double , double , double , double)));

    /*********************
    ** Auto Start
    **********************/
    qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_currpos_button_clicked( bool check )
{
    std_msgs::Bool msgs;

    msgs.data = true;

    qnode.send_fk_msgs( msgs );
}

void MainWindow::on_despos_button_clicked( bool check )
{
    geometry_msgs::Pose msgs;

    msgs.position.x = ui.pos_x_spinbox->value();
    msgs.position.y = ui.pos_y_spinbox->value();
    msgs.position.z = ui.pos_z_spinbox->value();

//    double roll = ui.ori_roll_spinbox->value() * M_PI / 180.0;
//    double pitch = ui.ori_pitch_spinbox->value() * M_PI / 180.0;
//    double yaw = ui.ori_yaw_spinbox->value() * M_PI / 180.0;

//    Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

//    msgs.orientation.x = QR.x();
//    msgs.orientation.y = QR.y();
//    msgs.orientation.z = QR.z();
//    msgs.orientation.w = QR.w();

    msgs.orientation.x = ui.ori_x_spinbox->value();
    msgs.orientation.y = ui.ori_y_spinbox->value();
    msgs.orientation.z = ui.ori_z_spinbox->value();
    msgs.orientation.w = ui.ori_w_spinbox->value();
    qnode.send_ik_msgs( msgs );
}

void MainWindow::update_curr_pos_spinbox( double x, double y, double z )
{
    ui.pos_x_spinbox->setValue( x );
    ui.pos_y_spinbox->setValue( y );
    ui.pos_z_spinbox->setValue( z );
}

void MainWindow::update_curr_ori_spinbox( double x , double y , double z , double w )
{
//    Eigen::Quaterniond QR(w,x,y,z);

//    Eigen::MatrixXd R = QR.toRotationMatrix();

//    double roll = atan2( R.coeff(2,1), R.coeff(2,2) ) * 180.0 / M_PI;
//    double pitch = atan2( -R.coeff(2,0), sqrt( pow(R.coeff(2,1),2) + pow(R.coeff(2,2),2) ) ) * 180.0 / M_PI;
//    double yaw = atan2 ( R.coeff(1,0) , R.coeff(0,0) ) * 180.0 / M_PI;

//    ui.ori_roll_spinbox->setValue( roll );
//    ui.ori_pitch_spinbox->setValue( pitch );
//    ui.ori_yaw_spinbox->setValue( yaw );

    ui.ori_x_spinbox->setValue( x );
    ui.ori_y_spinbox->setValue( y );
    ui.ori_z_spinbox->setValue( z );
    ui.ori_w_spinbox->setValue( w );
}

//Eigen::MatrixXd MainWindow::rx( double s )
//{
//    Eigen::MatrixXd R(3,3);

//    R << 1.0, 	 0.0, 	  0.0,
//         0.0, cos(s), -sin(s),
//         0.0, sin(s),  cos(s);

//    return R;
//}

//Eigen::MatrixXd MainWindow::ry( double s )
//{
//    Eigen::MatrixXd R(3,3);

//    R << cos(s), 0.0, sin(s),
//         0.0, 	 1.0, 	 0.0,
//        -sin(s), 0.0, cos(s);

//    return R;
//}

//Eigen::MatrixXd MainWindow::rz( double s )
//{
//    Eigen::MatrixXd R(3,3);

//    R << cos(s), -sin(s), 0.0,
//         sin(s),  cos(s), 0.0,
//            0.0,     0.0, 1.0;

//    return R;
//}

//Eigen::MatrixXd MainWindow::rpy2rotation( double r, double p, double y )
//{
//    Eigen::MatrixXd R = rz(y)*ry(p)*rx(r);

//    return R;
//}

//Eigen::Quaterniond MainWindow::rpy2quaternion( double r, double p, double y )
//{
//    Eigen::MatrixXd R = rpy2rotation(r, p, y);

//    Eigen::Matrix3d R_plus;
//    R_plus = R.block(0,0,3,3);

//    Eigen::Quaterniond QR;
//    QR = R_plus;

//    return QR;
//}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace robotis_manipulator_h_gui

