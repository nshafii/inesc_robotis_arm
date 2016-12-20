#include "../../INESC_Robotis_Driver/include/INESC_Robotis_Driver/TrajectoryGenerator.hpp"

TrajectoryGenerator::TrajectoryGenerator(const Eigen::Vector3d  p0, const Eigen::Vector3d  p1, double duration)
{
    this->p0 = p0;
    this->p1 = p1;
    this->p2 = Eigen::Vector3d (0.f, 0.f, 0.f);
    this->p3 = Eigen::Vector3d (0.f, 0.f, 0.f);
    this->duration = duration;
    type = "Linear";
};

TrajectoryGenerator::TrajectoryGenerator(const Eigen::Vector3d  p0, const Eigen::Vector3d  p1, const Eigen::Vector3d  p2, double duration)
{
    this->p0 = p0;
    this->p1 = p1;
    this->p2 = p2;
    this->p3 = Eigen::Vector3d (0.f, 0.f, 0.f);
    this->duration = duration;
    type = "Quadratic";
};

TrajectoryGenerator::TrajectoryGenerator(const Eigen::Vector3d  p0, const Eigen::Vector3d  p1, const Eigen::Vector3d  p2, const Eigen::Vector3d  p3, double duration)
{
    this->p0 = p0;
    this->p1 = p1;
    this->p2 = p2;
    this->p3 = p3;
    this->duration = duration;
    type = "Cubic";
};

void TrajectoryGenerator::setDuration(double duration)
{
    this->duration = duration;
}

void TrajectoryGenerator::setLinear(const Eigen::Vector3d  p0, const Eigen::Vector3d  p1, double duration)
{
	this->p0 = p0;
	this->p1 = p1;
	this->duration = duration;
	type = "Linear";
}

void TrajectoryGenerator::setQuadratic(const Eigen::Vector3d  p0, const Eigen::Vector3d  p1, const Eigen::Vector3d  p2, double duration)
{
	this->p0 = p0;
	this->p1 = p1;
	this->p2 = p2;
	this->duration = duration;
	type = "Quadratic";

	/*
	cout << "TrajectoryGenerator Quadratic created:" << endl;
	cout << "\tType = " << this->type << endl;
	cout << "\tP0 = " << this->p0 << endl;
	cout << "\tP1 = " << this->p1 << endl;
	cout << "\tP2 = " << this->p2 << endl;
	cout << "\tDuration = " << this->duration << endl;
	*/
}

void TrajectoryGenerator::setCubic(const Eigen::Vector3d  p0, const Eigen::Vector3d  p1, const Eigen::Vector3d  p2, const Eigen::Vector3d  p3, double duration)
{
	this->p0 = p0;
	this->p1 = p1;
	this->p2 = p2;
	this->p3 = p3;
	this->duration = duration;
	type = "Cubic";

/*
	cout << "TrajectoryGenerator Cubic Modified:" << endl;
	cout << "\tType = " << this->type << endl;
	cout << "\tP0 = " << this->p0 << endl;
	cout << "\tP1 = " << this->p1 << endl;
	cout << "\tP2 = " << this->p2 << endl;
	cout << "\tP3 = " << this->p3 << endl;
	cout << "\tDuration = " << this->duration << endl;
*/
}


double TrajectoryGenerator::getDuration() const
{
    return duration;
}

string TrajectoryGenerator::getType() const
{
    return type;
}



Eigen::Vector3d  TrajectoryGenerator::getLinearPosition(double t) const
{
    t = t/duration;
    if(t>1.f) t=1.f;
    if(t<0.f) t=0.f;
    
    double x = (1-t)*p0.x() + t*p1.x();
    double y = (1-t)*p0.y() + t*p1.y();
    double z = (1-t)*p0.z() + t*p1.z();

    return Eigen::Vector3d (x,y,z);
}


Eigen::Vector3d  TrajectoryGenerator::getQuadraticPosition(double t) const
{
    t = t/duration;
    if(t>1.f) t=1.f;
    if(t<0.f) t=0.f;

    double x = pow((1-t),2)*p0.x() + 2*(1-t)*t*p1.x() + pow(t,2)*p2.x();
    double y = pow((1-t),2)*p0.y() + 2*(1-t)*t*p1.y() + pow(t,2)*p2.y();
    double z = pow((1-t),2)*p0.z() + 2*(1-t)*t*p1.z() + pow(t,2)*p2.z();

    return Eigen::Vector3d(x,y,z);
}



Eigen::Vector3d  TrajectoryGenerator::getCubicPosition(double t) const
{
    t = t/duration;
    if(t>1.f) t=1.f;
    if(t<0.f) t=0.f;
    
    double x = pow((1-t),3)*p0.x() + 3*pow((1-t),2)*t*p1.x() + 3*(1-t)*pow(t,2)*p2.x() + pow(t,3)*p3.x();
    double y = pow((1-t),3)*p0.y() + 3*pow((1-t),2)*t*p1.y() + 3*(1-t)*pow(t,2)*p2.y() + pow(t,3)*p3.y();
    double z = pow((1-t),3)*p0.z() + 3*pow((1-t),2)*t*p1.z() + 3*(1-t)*pow(t,2)*p2.z() + pow(t,3)*p3.z();
    
    return Eigen::Vector3d(x,y,z);
}









