/**
 * @file /include/robotis_manipulator_h_gui/main_window.hpp
 *
 * @brief Adapted from Robotis Code
 **/
#ifndef cat_manipulator_ik_gui_MAIN_WINDOW_H
#define cat_manipulator_ik_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#define PI 3.141592654
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace cat_manipulator_ik_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    void on_actionAbout_triggered();

    void on_update_button_clicked( bool check );
    void on_publishJointState_button_clicked( bool check);

    void on_start_button_clicked(bool check);

    void on_move_gotoHomePosition_button_clicked( bool check);
    void on_JSfromEEF_button_clicked( bool check);
    void on_EEFfromJointState_button_clicked( bool check);

    void on_closegripper_button_clicked(bool check);
    void on_opengripper_button_clicked(bool check);

    void updateEEFinWindow();
    void updateJointStatesinWindow();
    void sendJoints(Eigen::VectorXd js);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void update_joint_states_spinbox(Eigen::VectorXd cur_joint_states);
    void update_eef_spinbox(Eigen::VectorXd eef);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    Eigen::Vector3d cylinder_tag_pose_;
    int numTimesY_;
    bool started_;
};

}  // cat_manipulator_ik_gui

#endif // cat_manipulator_ik_gui_MAIN_WINDOW_H
