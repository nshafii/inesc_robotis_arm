/*
 * trajectory.h
 *
 *  Created on: Jul 13, 2015
 *      Author: sch
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <Eigen/Dense>

// minimum jerk trajectory

Eigen::MatrixXd Tra_via0( double x0 , double v0 , double a0,
		                  double xf , double vf , double af,
						  double smp , double tf );

// minimum jerk trajectory with n number of via-points

Eigen::MatrixXd Tra_vian_q( int n,
		                    double x0 , double v0 , double a0 ,
	 	                    Eigen::MatrixXd x ,
		                    double xf , double vf , double af ,
		                    double smp , Eigen::MatrixXd t , double tf);

Eigen::MatrixXd Tra_vian_qdq( int n ,
						      double x0 , double v0 , double a0 ,
						      Eigen::MatrixXd x ,  Eigen::MatrixXd dx ,
						      double xf , double vf , double af ,
						      double smp , Eigen::MatrixXd t , double tf );

Eigen::MatrixXd Tra_vian_qdqddq( int n,
		                         double x0 , double v0 , double a0 ,
		                         Eigen::MatrixXd x,  Eigen::MatrixXd dx, Eigen::MatrixXd ddx,
		                         double xf, double vf, double af,
		                         double smp, Eigen::MatrixXd t, double tf );

#endif /* TRAJECTORY_H_ */
