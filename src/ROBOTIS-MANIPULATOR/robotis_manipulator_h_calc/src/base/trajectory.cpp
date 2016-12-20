/*
 * trajectory.cpp
 *
 *  Created on: Jul 13, 2015
 *      Author: sch
 */

#include <ros/ros.h>

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <stdlib.h>

#include "../../include/trajectory.h"

Eigen::MatrixXd Tra_via0( double x0 , double v0 , double a0,
		                  double xf , double vf , double af,
						  double smp , double tf )
/*
   simple minimum jerk trajectory

   x0 : position at initial state
   v0 : velocity at initial state
   a0 : acceleration at initial state

   xf : position at final state
   vf : velocity at final state
   af : acceleration at final state

   smp : sampling time

   tf : movement time
*/

{
	Eigen::MatrixXd A( 3 , 3 );
	Eigen::MatrixXd B( 3 , 1 );

	A << pow( tf , 3 )	   , pow( tf , 4 )	    , pow( tf , 5 ),
		 3 * pow( tf , 2 ) , 4 * pow( tf , 3 )	, 5 * pow( tf , 4 ),
		 6 * tf		       , 12 * pow( tf , 2 ) , 20 * pow( tf , 3 );

	B << xf - x0 - v0 * tf - a0 * pow( tf , 2 ) / 2,
		 vf - v0 - a0 * tf,
		 af - a0 ;

	Eigen::Matrix<double,3,1> C = A.inverse() * B;

	double N;

	N = tf / smp;
	int NN = round( N + 1 );

	Eigen::MatrixXd Time = Eigen::MatrixXd::Zero( NN , 1 );
	Eigen::MatrixXd Tra = Eigen::MatrixXd::Zero( NN , 1 );

	int i;

	for ( i = 1; i <= NN; i++ )
		Time.coeffRef( i - 1 , 0 ) = ( i - 1 ) * smp;

	for ( i = 1; i <= NN; i++ )
	{
		Tra.coeffRef(i-1,0) =
				x0 +
				v0 * Time.coeff( i - 1 ) +
				0.5 * a0 * pow( Time.coeff( i - 1 ) , 2 ) +
				C.coeff( 0 , 0 ) * pow( Time.coeff( i - 1 ) , 3 ) +
				C.coeff( 1 , 0 ) * pow( Time.coeff( i - 1 ) , 4 ) +
				C.coeff( 2 , 0 ) * pow( Time.coeff( i - 1 ) , 5 );
	}

	return Tra;
}

Eigen::MatrixXd Tra_vian_q( int n ,
		                    double x0 , double v0 , double a0 ,
		                    Eigen::MatrixXd x ,
		                    double xf , double vf , double af ,
		                    double smp , Eigen::MatrixXd t , double tf)
/*
   minimum jerk trajectory with via-points
   (via-point constraints: position at each point)

   n  : the number of via-points

   x0 : position at initial state
   v0 : velocity at initial state
   a0 : acceleration at initial state

   x  : position matrix at via-points state ( size : n x 1 )

   xf : position at final state
   vf : velocity at final state
   af : acceleration at final state

   smp : sampling time

   t  : time matrix passing through via-points state ( size : n x 1 )
   tf : movement time
*/

{
	int i , j , k ;

	/* Calculation Matrix B	*/

	Eigen::MatrixXd B = Eigen::MatrixXd::Zero( n + 3 , 1 );

	for ( i = 1; i <= n; i++ )
	{
		B.coeffRef( i - 1 , 0 ) =
				x.coeff( i - 1 , 0 ) -
				x0 -
				v0 * t.coeff( i - 1 , 0 ) -
				( a0 / 2 ) * pow( t.coeff( i - 1 , 0 ) , 2 ) ;
	}

	B.coeffRef( n , 0 ) =
			xf -
			x0 -
			v0 * tf -
			( a0 / 2 ) * pow( tf , 2 ) ;

	B.coeffRef( n + 1 , 0 ) =
			vf -
			v0 -
			a0 * tf ;

	B.coeffRef( n + 2 , 0 ) =
			af -
			a0 ;

	/* Calculation Matrix A	*/

	Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero( n , 3 );

	for ( i = 1; i <= n; i++ )
	{
		A1.coeffRef( i - 1 , 0 ) = pow( t.coeff( i - 1 , 0 ) , 3 );
		A1.coeffRef( i - 1 , 1 ) = pow( t.coeff( i - 1 , 0 ) , 4 );
		A1.coeffRef( i - 1 , 2 ) = pow( t.coeff( i - 1 , 0 ) , 5 );
	}

	Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero( n , n );

	for ( i = 1; i <= n; i++ )
	{
		for ( j = 1; j <= n; j++ )
		{
			if ( i > j )
				k = i;
			else
				k = j;

			A2.coeffRef( j - 1 , i - 1 ) =
					pow( ( t.coeff( k - 1 , 0 ) - t.coeff( i - 1 , 0 ) ) , 5 ) / 120;
		}
	}

	Eigen::MatrixXd A3 = Eigen::MatrixXd::Zero( 3 , n + 3 );

	A3.coeffRef( 0 , 0 ) = pow( tf , 3 );
	A3.coeffRef( 0 , 1 ) = pow( tf , 4 );
	A3.coeffRef( 0 , 2 ) = pow( tf , 5 );

	A3.coeffRef( 1 , 0 ) = 3 * pow( tf , 2 );
	A3.coeffRef( 1 , 1 ) = 4 * pow( tf , 3 );
	A3.coeffRef( 1 , 2 ) = 5 * pow( tf , 4 );

	A3.coeffRef( 2 , 0 ) = 6 * tf;
	A3.coeffRef( 2 , 1 ) = 12 * pow( tf , 2 );
	A3.coeffRef( 2 , 2 ) = 20 * pow( tf , 3 );

	for ( i = 1; i <= n; i++ )
	{
		A3.coeffRef( 0 , i + 2 ) = pow( tf - t.coeff( i - 1 , 0 ) , 5 ) / 120;
		A3.coeffRef( 1 , i + 2 ) = pow( tf - t.coeff( i - 1 , 0 ) , 4 ) / 24;
		A3.coeffRef( 2 , i + 2 ) = pow( tf - t.coeff( i - 1 , 0 ) , 3 ) / 6;
	}

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero( n + 3 , n + 3 );

	A.block( 0 , 0 , n , 3 ) = A1;
	A.block( 0 , 3 , n , n ) = A2;
	A.block( n , 0 , 3 , n + 3 ) = A3;

	/* Calculation Matrix C (coefficient of polynomial function) */

	Eigen::MatrixXd C( 2 * n + 3 , 1 );
	//C = A.inverse() * B;
	C = A.colPivHouseholderQr().solve(B);

	/* Time */

	int NN;
	double N;

	N = tf / smp ;
	NN = round( N ) ;

	Eigen::MatrixXd Time = Eigen::MatrixXd::Zero( NN + 1 , 1 );

	for ( i = 1; i <= NN + 1; i++ )
		Time.coeffRef( i - 1 , 0 ) = ( i - 1 ) * smp;

	/* Time_via */

	Eigen::MatrixXd Time_via = Eigen::MatrixXd::Zero( n , 1 );

	for ( i = 1; i <= n; i++ )
		Time_via.coeffRef( i - 1 , 0 ) = round( t.coeff( i - 1 , 0 ) / smp ) + 2;

	/* Minimum Jerk Trajectory with Via-points */

	Eigen::MatrixXd Tra_jerk_via = Eigen::MatrixXd::Zero( NN + 1 , 1 );

	for ( i = 1; i <= NN + 1; i++ )
	{
		Tra_jerk_via.coeffRef( i - 1 , 0 ) =
				x0 +
				v0 * Time.coeff( i - 1 , 0 ) +
				0.5 * a0 * pow( Time.coeff( i - 1 , 0 ) , 2 ) +
				C.coeff( 0 , 0 ) * pow( Time.coeff( i - 1 , 0 ) , 3 ) +
				C.coeff( 1 , 0 ) * pow( Time.coeff( i - 1 , 0 ) , 4 ) +
				C.coeff( 2 , 0 ) * pow( Time.coeff( i - 1 , 0 ) , 5 ) ;
	}

	for ( i = 1 ; i <= n; i++ )
	{
		for ( j = Time_via.coeff( i - 1 , 0 ); j <= NN + 1; j++ )
		{
			Tra_jerk_via.coeffRef( j - 1 , 0 ) =
					Tra_jerk_via.coeff( j - 1 , 0 ) +
					C.coeff( i + 2 , 0 ) * pow( ( Time.coeff( j - 1 , 0 ) - t.coeff( i - 1 , 0 ) ) , 5 ) / 120 ;
		}
	}

	return Tra_jerk_via;
}

Eigen::MatrixXd Tra_vian_qdq( int n ,
						      double x0 , double v0 , double a0 ,
						      Eigen::MatrixXd x ,  Eigen::MatrixXd dx ,
						      double xf , double vf , double af ,
						      double smp , Eigen::MatrixXd t , double tf )
/*
   minimum jerk trajectory with via-points
   (via-point constraints: position and velocity at each point)

   n  : the number of via-points

   x0 : position at initial state
   v0 : velocity at initial state
   a0 : acceleration at initial state

   x  : position matrix at via-points state ( size : n x 1 )
   dx : velocity matrix at via-points state ( size : n x 1 )

   xf : position at final state
   vf : velocity at final state
   af : acceleration at final state

   smp : sampling time

   t  : time matrix passing through via-points state ( size : n x 1 )
   tf : movement time
*/

{
	int i,j,k ;

	/* Calculation Matrix B	*/

	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2*n+3,1);

	for ( i = 1; i <= n; i++ )
	{
		B.coeffRef( 2 * i - 2 , 0 ) =
				x.coeff( i - 1 , 0 ) -
				x0 -
				v0 * t.coeff( i - 1 , 0 ) -
				( a0 / 2 ) * pow( t.coeff( i - 1 , 0 ) , 2 ) ;

		B.coeffRef( 2 * i - 1 , 0 ) =
				dx.coeff( i - 1 , 0 ) -
				v0 -
				a0 * t.coeff( i - 1 , 0 ) ;
	}

	B.coeffRef( 2 * n , 0 ) =
			xf -
			x0 -
			v0 * tf -
			( a0 / 2 ) * pow( tf , 2 ) ;

	B.coeffRef( 2 * n + 1 , 0 ) =
			vf -
			v0 -
			a0 * tf ;

	B.coeffRef( 2 * n + 2 , 0 ) =
			af -
			a0 ;

	/* Calculation Matrix A	*/

	Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero( 2 * n , 3 );

	for ( i = 1; i <= n; i++ )
	{
		A1.coeffRef( 2 * i - 2 , 0 ) = pow( t.coeff( i - 1 , 0 ) , 3 ) ;
		A1.coeffRef( 2 * i - 2 , 1 ) = pow( t.coeff( i - 1 , 0 ) , 4 ) ;
		A1.coeffRef( 2 * i - 2 , 2 ) = pow( t.coeff( i - 1 , 0 ) , 5 ) ;

		A1.coeffRef( 2 * i - 1 , 0 ) = 3 * pow( t.coeff( i - 1 , 0 ) , 2 ) ;
		A1.coeffRef( 2 * i - 1 , 1 ) = 4 * pow( t.coeff( i - 1 , 0 ) , 3 ) ;
		A1.coeffRef( 2 * i - 1 , 2 ) = 5 * pow( t.coeff( i - 1 , 0 ) , 4 ) ;
	}


	Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero( 2 * n , 2 * n );

	for ( i = 1; i <= n; i++ )
	{
		for ( j = 1; j <= n; j++ )
		{
			if ( i > j )
				k = i ;
			else
				k = j ;

			A2.coeffRef( 2 * j - 2 , 2 * i - 2 ) = pow( ( t.coeff( k - 1 , 0 ) - t.coeff( i - 1 , 0 ) ) , 4 ) / 24 ;
			A2.coeffRef( 2 * j - 2 , 2 * i - 1 ) = pow( ( t.coeff( k - 1 , 0 ) - t.coeff( i - 1 , 0 ) ) , 5 ) / 120 ;

			A2.coeffRef( 2 * j - 1 , 2 * i - 2 ) = pow( ( t.coeff( k - 1 , 0 ) - t.coeff( i - 1 , 0 ) ) , 3 ) / 6 ;
			A2.coeffRef( 2 * j - 1 , 2 * i - 1 ) = pow( ( t.coeff( k - 1 , 0 ) - t.coeff( i - 1 , 0 ) ) , 4 ) / 24 ;
		}
	}


	Eigen::MatrixXd A3 = Eigen::MatrixXd::Zero( 3 , 2 * n + 3 );

	A3.coeffRef( 0 , 0 ) = pow( tf , 3 );
	A3.coeffRef( 0 , 1 ) = pow( tf , 4 );
	A3.coeffRef( 0 , 2 ) = pow( tf , 5 );

	A3.coeffRef( 1 , 0 ) = 3 * pow( tf , 2 );
	A3.coeffRef( 1 , 1 ) = 4 * pow( tf , 3 );
	A3.coeffRef( 1 , 2 ) = 5 * pow( tf , 4 );

	A3.coeffRef( 2 , 0 ) = 6 * tf;
	A3.coeffRef( 2 , 1 ) = 12 * pow( tf , 2 );
	A3.coeffRef( 2 , 2 ) = 20 * pow( tf , 3 );

	for ( i = 1; i <= n; i++ )
	{
		A3.coeffRef( 0 , 2 * i + 1 ) = pow( tf - t.coeff( i - 1 , 0 ) , 4 ) / 24 ;
		A3.coeffRef( 1 , 2 * i + 1 ) = pow( tf - t.coeff( i - 1 , 0 ) , 3 ) / 6 ;
		A3.coeffRef( 2 , 2 * i + 1 ) = pow( tf - t.coeff( i - 1 , 0 ) , 2 ) / 2 ;

		A3.coeffRef( 0 , 2 * i + 2 ) = pow( tf - t.coeff( i - 1 , 0 ) , 5 ) / 120 ;
		A3.coeffRef( 1 , 2 * i + 2 ) = pow( tf - t.coeff( i - 1 , 0 ) , 4 ) / 24 ;
		A3.coeffRef( 2 , 2 * i + 2 ) = pow( tf - t.coeff( i - 1 , 0 ) , 3 ) / 6 ;
	}

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero( 2 * n + 3 , 2 * n + 3 );

	A.block( 0 , 0 , 2 * n , 3 ) = A1 ;
	A.block( 0 , 3 , 2 * n , 2 * n ) = A2 ;
	A.block( 2 * n , 0 , 3 , 2 * n + 3 ) = A3 ;

	/* Calculation Matrix C (coefficient of polynomial function) */

	Eigen::MatrixXd C( 2 * n + 3 , 1 );
	//C = A.inverse()*B;
	C = A.colPivHouseholderQr().solve( B );

	/* Time */

	int NN;
	double N;

	N = tf/smp ;
	NN = round( N ) ;

	Eigen::MatrixXd Time = Eigen::MatrixXd::Zero( NN + 1 , 1 );

	for ( i = 1; i <= NN+1; i++ )
		Time.coeffRef( i - 1 , 0 ) = ( i - 1 ) * smp;

	/* Time_via */

	Eigen::MatrixXd Time_via = Eigen::MatrixXd::Zero( n , 1 );

	for ( i = 1; i <= n; i++ )
		Time_via.coeffRef( i - 1 , 0 ) = round( t.coeff( i - 1 , 0 ) / smp ) + 2;

	/* Minimum Jerk Trajectory with Via-points */

	Eigen::MatrixXd Tra_jerk_via = Eigen::MatrixXd::Zero( NN+1 , 1 );

	for ( i = 1; i <= NN + 1; i++ )
	{
		Tra_jerk_via.coeffRef( i - 1 , 0 ) =
				x0 +
				v0*Time.coeff( i - 1 , 0 ) +
				0.5 * a0 * pow( Time.coeff( i - 1 , 0 ) , 2 ) +
				C.coeff( 0 , 0 )*pow( Time.coeff( i - 1 , 0 ) , 3 ) +
				C.coeff( 1 , 0 )*pow( Time.coeff( i - 1 , 0 ) , 4 ) +
				C.coeff( 2 , 0 )*pow( Time.coeff( i - 1 , 0 ) , 5 ) ;
	}

	for ( i = 1; i <= n; i++ )
	{
		for ( j = Time_via.coeff( i - 1 , 0 ); j <= NN + 1; j++ )
		{
			Tra_jerk_via.coeffRef( j - 1 , 0 ) =
					Tra_jerk_via.coeff( j - 1 , 0 ) +
					C.coeff( 2 * i + 1 , 0 ) * pow( ( Time.coeff( j - 1 , 0 ) - t.coeff( i - 1 , 0 ) ) , 4 ) / 24 +
					C.coeff( 2 * i + 2 , 0 ) * pow( ( Time.coeff( j - 1 , 0 ) - t.coeff( i - 1 , 0 ) ) , 5 ) / 120 ;
		}
	}

	return Tra_jerk_via;
}

Eigen::MatrixXd Tra_vian_qdqddq( int n,
		                         double x0 , double v0 , double a0 ,
		                         Eigen::MatrixXd x,  Eigen::MatrixXd dx, Eigen::MatrixXd ddx,
		                         double xf, double vf, double af,
		                         double smp, Eigen::MatrixXd t, double tf )
/*
   minimum jerk trajectory with via-points
   (via-point constraints: position and velocity at each point)

   n  : the number of via-points

   x0 : position at initial state
   v0 : velocity at initial state
   a0 : acceleration at initial state

   x  : position matrix at via-points state ( size : n x 1 )
   dx : velocity matrix at via-points state ( size : n x 1 )
   ddx : acceleration matrix at via-points state ( size : n x 1 )

   xf : position at final state
   vf : velocity at final state
   af : acceleration at final state

   smp : sampling time

   t  : time matrix passing through via-points state ( size : n x 1 )
   tf : movement time
*/

{
	int i,j,k ;

	/* Calculation Matrix B */

	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3*n+3,1);

	for (i=1; i<=n; i++){
		B.coeffRef(3*i-3,0) =
				x.coeff(i-1,0) -
				x0 -
				v0*t.coeff(i-1,0) -
				(a0/2)*pow(t.coeff(i-1,0),2) ;

		B.coeffRef(3*i-2,0) =
				dx.coeff(i-1,0) -
				v0 -
				a0*t.coeff(i-1,0) ;

		B.coeffRef(3*i-1,0) =
				ddx.coeff(i-1,0) -
				a0 ;
	}

	B.coeffRef(3*n,0) =
			xf -
			x0 -
			v0*tf -
			(a0/2)*pow(tf,2) ;

	B.coeffRef(3*n+1,0) =
			vf -
			v0 -
			a0*tf ;

	B.coeffRef(3*n+2,0) =
			af -
			a0 ;


	/* Calculation Matrix A */

	Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(3*n,3);

	for (i=1; i<=n; i++){
		A1.coeffRef(3*i-3,0) = pow(t.coeff(i-1,0),3) ;
		A1.coeffRef(3*i-3,1) = pow(t.coeff(i-1,0),4) ;
		A1.coeffRef(3*i-3,2) = pow(t.coeff(i-1,0),5) ;

		A1.coeffRef(3*i-2,0) = 3*pow(t.coeff(i-1,0),2) ;
		A1.coeffRef(3*i-2,1) = 4*pow(t.coeff(i-1,0),3) ;
		A1.coeffRef(3*i-2,2) = 5*pow(t.coeff(i-1,0),4) ;

		A1.coeffRef(3*i-1,0) = 6*t.coeff(i-1,0) ;
		A1.coeffRef(3*i-1,1) = 12*pow(t.coeff(i-1,0),2) ;
		A1.coeffRef(3*i-1,2) = 20*pow(t.coeff(i-1,0),3) ;
	}


	Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(3*n,3*n);

	for (i=1; i<=n; i++){
		for (j=1; j<=n; j++){
			if (i > j){
				k = i ;
			}else{
				k = j ;
			}
			A2.coeffRef(3*j-3,3*i-3) = pow((t.coeff(k-1,0)-t.coeff(i-1,0)),3)/6 ;
			A2.coeffRef(3*j-3,3*i-2) = pow((t.coeff(k-1,0)-t.coeff(i-1,0)),4)/24 ;
			A2.coeffRef(3*j-3,3*i-1) = pow((t.coeff(k-1,0)-t.coeff(i-1,0)),5)/120 ;

			A2.coeffRef(3*j-2,3*i-3) = pow((t.coeff(k-1,0)-t.coeff(i-1,0)),2)/2 ;
			A2.coeffRef(3*j-2,3*i-2) = pow((t.coeff(k-1,0)-t.coeff(i-1,0)),3)/6 ;
			A2.coeffRef(3*j-2,3*i-1) = pow((t.coeff(k-1,0)-t.coeff(i-1,0)),4)/24 ;

			A2.coeffRef(3*j-1,3*i-3) = t.coeff(k-1,0)-t.coeff(i-1,0) ;
			A2.coeffRef(3*j-1,3*i-2) = pow((t.coeff(k-1,0)-t.coeff(i-1,0)),2)/2 ;
			A2.coeffRef(3*j-1,3*i-1) = pow((t.coeff(k-1,0)-t.coeff(i-1,0)),3)/6 ;
		}
	}


	Eigen::MatrixXd A3 = Eigen::MatrixXd::Zero(3,3*n+3);

	A3.coeffRef(0,0) = pow(tf,3);
	A3.coeffRef(0,1) = pow(tf,4);
	A3.coeffRef(0,2) = pow(tf,5);

	A3.coeffRef(1,0) = 3*pow(tf,2);
	A3.coeffRef(1,1) = 4*pow(tf,3);
	A3.coeffRef(1,2) = 5*pow(tf,4);

	A3.coeffRef(2,0) = 6*tf;
	A3.coeffRef(2,1) = 12*pow(tf,2);
	A3.coeffRef(2,2) = 20*pow(tf,3);

	for (i=1; i<=n; i++){
		A3.coeffRef(0,3*i) = pow(tf-t.coeff(i-1,0),3)/6 ;
		A3.coeffRef(1,3*i) = pow(tf-t.coeff(i-1,0),2)/2 ;
		A3.coeffRef(2,3*i) = tf-t.coeff(i-1,0) ;

		A3.coeffRef(0,3*i+1) = pow(tf-t.coeff(i-1,0),4)/24 ;
		A3.coeffRef(1,3*i+1) = pow(tf-t.coeff(i-1,0),3)/6 ;
		A3.coeffRef(2,3*i+1) = pow(tf-t.coeff(i-1,0),2)/2 ;

		A3.coeffRef(0,3*i+2) = pow(tf-t.coeff(i-1,0),5)/120 ;
		A3.coeffRef(1,3*i+2) = pow(tf-t.coeff(i-1,0),4)/24 ;
		A3.coeffRef(2,3*i+2) = pow(tf-t.coeff(i-1,0),3)/6 ;
	}

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3*n+3,3*n+3);

	A.block(0,0,3*n,3) = A1 ;
	A.block(0,3,3*n,3*n) = A2 ;
	A.block(3*n,0,3,3*n+3) = A3 ;

	/* Calculation Matrix C (coefficient of polynomial function) */

	Eigen::MatrixXd C(3*n+3,1);
	//C = A.inverse()*B;
	C = A.colPivHouseholderQr().solve(B);

	/* Time */

	int NN;
	double N;

	N = tf/smp ;
	NN = round(N) ;

	Eigen::MatrixXd Time = Eigen::MatrixXd::Zero(NN+1,1);

	for (i=1; i<=NN+1; i++){
		Time.coeffRef(i-1,0) = (i-1)*smp;
	}

	/* Time_via */

	Eigen::MatrixXd Time_via = Eigen::MatrixXd::Zero(n,1);

	for (i=1; i<=n; i++){
		Time_via.coeffRef(i-1,0) = round(t.coeff(i-1,0)/smp)+2;
	}

	/* Minimum Jerk Trajectory with Via-points */

	Eigen::MatrixXd Tra_jerk_via = Eigen::MatrixXd::Zero(NN+1,1);

	for (i=1; i<=NN+1; i++){
		Tra_jerk_via.coeffRef(i-1,0) =
				x0 +
				v0*Time.coeff(i-1,0) +
				0.5*a0*pow(Time.coeff(i-1,0),2) +
				C.coeff(0,0)*pow(Time.coeff(i-1,0),3) +
				C.coeff(1,0)*pow(Time.coeff(i-1,0),4) +
				C.coeff(2,0)*pow(Time.coeff(i-1,0),5) ;
	}

	for (i=1; i<=n; i++){
		for (j=Time_via.coeff(i-1,0); j<=NN+1; j++){
			Tra_jerk_via.coeffRef(j-1,0) =
					Tra_jerk_via.coeff(j-1,0) +
					C.coeff(3*i,0)*pow((Time.coeff(j-1,0)-t.coeff(i-1,0)),3)/6 +
					C.coeff(3*i+1,0)*pow((Time.coeff(j-1,0)-t.coeff(i-1,0)),4)/24 +
					C.coeff(3*i+2,0)*pow((Time.coeff(j-1,0)-t.coeff(i-1,0)),5)/120 ;

		}
	}

	return Tra_jerk_via;

}
