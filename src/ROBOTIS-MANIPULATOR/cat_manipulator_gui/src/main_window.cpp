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
#include "../include/cat_manipulator_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cat_manipulator_gui {

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

    QObject::connect(&qnode, SIGNAL(update_joint_states(Eigen::VectorXd)), this, SLOT(update_joint_states_spinbox(Eigen::VectorXd)));
    QObject::connect(&qnode, SIGNAL(update_eef(Eigen::VectorXd)), this, SLOT(update_eef_spinbox(Eigen::VectorXd)));

    /*********************
    ** Auto Start
    **********************/
    qnode.init();

    started_ = false;

    qnode.printSTARTmessage();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::updateEEFinWindow(){
    //get data from joint states
    //print joint states
    //qnode.printEEFState();

    Eigen::Vector3d eef_xyz;
    Eigen::Vector3d eef_zyxAngle;
    eef_xyz = qnode.get_cur_eef_pos();
    eef_zyxAngle = qnode.get_cur_eef_zyxAngle();

    Eigen::VectorXd eef(6);
    eef(0) = eef_xyz(0);
    eef(1) = eef_xyz(1);
    eef(2) = eef_xyz(2);
    eef(3) = eef_zyxAngle(0);
    eef(4) = eef_zyxAngle(1);
    eef(5) = eef_zyxAngle(2);
    update_eef_spinbox(eef);
}

void MainWindow::updateJointStatesinWindow(){

    //get data from joint states
    //print joint states
    //qnode.printJointState();

    Eigen::VectorXd js(6);
    js= qnode.get_cur_joint_state();
    update_joint_states_spinbox(js);
}

void MainWindow::sendJoints(Eigen::VectorXd js){
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    int num_joints = 6;
    msg.name.resize(num_joints);
    msg.position.resize(num_joints);

    msg.name[0] = "joint1";
    msg.name[1] = "joint2";
    msg.name[2] = "joint3";
    msg.name[3] = "joint4";
    msg.name[4] = "joint5";
    msg.name[5] = "joint6";

    msg.position[0] = js(0);
    msg.position[1] = js(1);
    msg.position[2] = js(2);
    msg.position[3] = js(3);
    msg.position[4] = js(4);
    msg.position[5] = js(5);

    ROS_INFO("should be publishing the joint states");

    std::cout << msg << std::endl;

    qnode.send_jointposition_msgs( msg );
}

void MainWindow::on_start_button_clicked(bool check){

    updateEEFinWindow();

    updateJointStatesinWindow();

    started_ = true;

    qnode.logInfo( "Start complete." );

}

void MainWindow::on_update_button_clicked(bool check){

    updateJointStatesinWindow();

    updateEEFinWindow();
}

void MainWindow::on_EEFfromJointState_button_clicked( bool check)
{
    Eigen::VectorXd js(6);
    js <<   ui.joint1_theta_spinbox->value(),
            ui.joint2_theta_spinbox->value(),
            ui.joint3_theta_spinbox->value(),
            ui.joint4_theta_spinbox->value(),
            ui.joint5_theta_spinbox->value(),
            ui.joint6_theta_spinbox->value();

    std::cout << " ********* js ************** \n";
    std::cout << js << std::endl;

    Eigen::VectorXd eef(6), eef_pos(3), eef_angles(3);

    qnode.set_cur_joint_state(js);
    qnode.callFK();
    //qnode.printEEFState();

    std::cout << " here * \n";

    eef_pos = qnode.get_cur_eef_pos();
    eef_angles = qnode.get_cur_eef_zyxAngle();
    eef << eef_pos , eef_angles;

    std::cout << " here * \n";
    std::cout << eef << std::endl;

    update_eef_spinbox(eef);

    std::cout << " here * \n";

    ROS_INFO("should be publishing the joint states");

    //updateEEFinWindow();
    //updateJointStatesinWindow();

}

void MainWindow::on_JSfromEEF_button_clicked( bool check)
{
    Eigen::Vector3d eef_pos, eef_zyxAngle;
    eef_pos(0) = ui.eef_pos_x_spinbox->value();
    eef_pos(1) = ui.eef_pos_y_spinbox->value();
    eef_pos(2) = ui.eef_pos_z_spinbox->value();
    eef_zyxAngle(2) = ui.eef_zyxangle_z_spinbox->value();
    eef_zyxAngle(1) = ui.eef_zyxangle_y_spinbox->value();
    eef_zyxAngle(0) = ui.eef_zyxangle_x_spinbox->value();

    qnode.set_goal_eef_pos(eef_pos);
    qnode.set_goal_eef_zyxAngle(eef_zyxAngle);
    //if(! qnode.callIK())
    //{
    //    qnode.logInfo("Returned false from callIK! ");
    //}

    qnode.logInfo("Setting the joint states ans stop is flase ");
    qnode.stop=false;
}

void MainWindow::update_joint_states_spinbox(Eigen::VectorXd js){
    ui.joint1_theta_spinbox->setValue(js(0));
    ui.joint2_theta_spinbox->setValue(js(1));
    ui.joint3_theta_spinbox->setValue(js(2));
    ui.joint4_theta_spinbox->setValue(js(3));
    ui.joint5_theta_spinbox->setValue(js(4));
    ui.joint6_theta_spinbox->setValue(js(5));
}

void MainWindow::update_eef_spinbox(Eigen::VectorXd eef){
    ui.eef_pos_x_spinbox->setValue(eef(0));
    ui.eef_pos_y_spinbox->setValue(eef(1));
    ui.eef_pos_z_spinbox->setValue(eef(2));
    ui.eef_zyxangle_z_spinbox->setValue(eef(3));
    ui.eef_zyxangle_y_spinbox->setValue(eef(4));
    ui.eef_zyxangle_x_spinbox->setValue(eef(5));
}


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

void MainWindow::on_publishJointState_button_clicked( bool check)
{

    Eigen::VectorXd js(6);
    js <<   ui.joint1_theta_spinbox->value(),
            ui.joint2_theta_spinbox->value(),
            ui.joint3_theta_spinbox->value(),
            ui.joint4_theta_spinbox->value(),
            ui.joint5_theta_spinbox->value(),
            ui.joint6_theta_spinbox->value();

    std::cout << " ********* js ************** \n";
    std::cout << js << std::endl;

    Eigen::VectorXd eef(6), eef_pos(3), eef_angles(3);

    qnode.set_cur_joint_state(js);
    qnode.callFK();
    //qnode.printEEFState();

    eef_pos = qnode.get_cur_eef_pos();
    eef_angles = qnode.get_cur_eef_zyxAngle();
    eef << eef_pos , eef_angles;

    std::cout << eef << std::endl;

    //update_eef_spinbox(eef);

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    int num_joints = 6;
    msg.name.resize(num_joints);
    msg.position.resize(num_joints);

    msg.name[0] = "joint1";
    msg.name[1] = "joint2";
    msg.name[2] = "joint3";
    msg.name[3] = "joint4";
    msg.name[4] = "joint5";
    msg.name[5] = "joint6";

    msg.position[0] = ui.joint1_theta_spinbox->value();
    msg.position[1] = ui.joint2_theta_spinbox->value();
    msg.position[2] = ui.joint3_theta_spinbox->value();
    msg.position[3] = ui.joint4_theta_spinbox->value();
    msg.position[4] = ui.joint5_theta_spinbox->value();
    msg.position[5] = ui.joint6_theta_spinbox->value();

    ROS_INFO("should be publishing the joint states");


    qnode.stop = true;

    // check if msg was sent. check new position. update the other window

}

void MainWindow::on_move_gotoHomePosition_button_clicked( bool check)
{

    Eigen::VectorXd js(6);
    js << 0, 0, 0, 0, 0, 0;

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    int num_joints = 6;
    msg.name.resize(num_joints);
    msg.position.resize(num_joints);

    msg.name[0] = "joint1";
    msg.name[1] = "joint2";
    msg.name[2] = "joint3";
    msg.name[3] = "joint4";
    msg.name[4] = "joint5";
    msg.name[5] = "joint6";

    msg.position[0] = js(0);
    msg.position[1] = js(1);
    msg.position[2] = js(2);
    msg.position[3] = js(3);
    msg.position[4] = js(4);
    msg.position[5] = js(5);

    ROS_INFO("should be publishing the joint states");

    std::cout << msg << std::endl;
    qnode.stop = true;
    qnode.send_jointposition_msgs( msg );

    updateEEFinWindow();
    updateJointStatesinWindow();

}

void MainWindow::on_closegripper_button_clicked(bool check){

    std_msgs::Bool gripper_command;
    gripper_command.data = true;
    qnode.send_grippercommand_msgs( gripper_command );

}

void MainWindow::on_opengripper_button_clicked(bool check){

    std_msgs::Bool gripper_command;
    gripper_command.data = false;
    qnode.send_grippercommand_msgs( gripper_command );

}

}  // namespace cat_manipulator_gui
