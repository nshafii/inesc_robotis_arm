#ifndef TRAGECTORY_GENERATOR_H
#define TRAGECTORY_GENERATOR_H

#include <string>
#include <cmath>
#include <Eigen/Dense>

using namespace std;

class TrajectoryGenerator {
public:
  TrajectoryGenerator() {
  }
  ;
  TrajectoryGenerator(const Eigen::Vector3d p0, const Eigen::Vector3d p1,
      double duration = 1.f);
  TrajectoryGenerator(const Eigen::Vector3d p0, const Eigen::Vector3d p1,
      const Eigen::Vector3d p2, double duration = 1.f);
  TrajectoryGenerator(const Eigen::Vector3d p0, const Eigen::Vector3d p1,
      const Eigen::Vector3d p2, const Eigen::Vector3d p3,
      double duration = 1.f);

  void setDuration(double duration);
  void setLinear(const Eigen::Vector3d p0, const Eigen::Vector3d p1,
      double duration = 1.f);
  void setQuadratic(const Eigen::Vector3d p0, const Eigen::Vector3d p1,
      const Eigen::Vector3d p2, double duration);
  void setCubic(const Eigen::Vector3d p0, const Eigen::Vector3d p1,
      const Eigen::Vector3d p2, const Eigen::Vector3d p3, double duration);
  double getDuration() const;
  string getType() const;
  Eigen::Vector3d getCubicPosition(double t) const;
  Eigen::Vector3d getQuadraticPosition(double t) const;

  Eigen::Vector3d getLinearPosition(double t) const;

private:
  Eigen::Vector3d p0, p1, p2, p3;
  std::string type;
  double duration;

};

#endif // TRAGECTORY_GENERATOR_H
