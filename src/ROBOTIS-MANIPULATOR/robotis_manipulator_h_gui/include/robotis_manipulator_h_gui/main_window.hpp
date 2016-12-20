/**
 * @file /include/robotis_manipulator_h_gui/main_window.hpp
 *
 * @brief Qt based gui for robotis_manipulator_h_gui.
 *
 * @date November 2010
 **/
#ifndef robotis_manipulator_h_gui_MAIN_WINDOW_H
#define robotis_manipulator_h_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robotis_manipulator_h_gui {

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

//    /******************************************
//    ** Transformation
//    *******************************************/
//    Eigen::MatrixXd rx( double s );
//    Eigen::MatrixXd ry( double s );
//    Eigen::MatrixXd rz( double s );
//    Eigen::MatrixXd rpy2rotation(double r, double p, double y);
//    Eigen::Quaterniond rpy2quaternion(double r, double p, double y);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    void on_actionAbout_triggered();

    void on_currpos_button_clicked( bool check );
    void on_despos_button_clicked( bool check );

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void update_curr_pos_spinbox( double x , double y , double z  );
    void update_curr_ori_spinbox( double x , double y , double z , double w );



private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace robotis_manipulator_h_gui

#endif // robotis_manipulator_h_gui_MAIN_WINDOW_H
