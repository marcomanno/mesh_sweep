#pragma once
#include "range.hh"

#include <memory>

namespace Geo
{

struct ITransform
{

  // The lenght of the rotation is sqrt(angle)
  virtual void set_rotation_axis(const VectorD<3>& _axis) = 0;
  virtual const VectorD<3>& get_rotation_axis() const = 0;

  virtual void set_rotation_origin(const VectorD<3>& _orign) = 0;
  virtual const VectorD<3>& get_rotation_origin() const = 0;

  virtual void set_translation(const VectorD<3> & _delta) = 0;
  virtual const VectorD<3>& get_translation() const = 0;

  virtual std::unique_ptr<ITransform> clone() const = 0;

  virtual VectorD<3> operator()(const VectorD<3> & _pos) = 0;

  static std::unique_ptr<ITransform> make();
  static std::unique_ptr<ITransform> interpolate(
    const ITransform& _start, const ITransform& _end, double _par);
  static VectorD<3> rotation_axis(const VectorD<3>& _direction, double angle);
};

struct ITrajectory
{
  virtual ~ITrajectory() {}
  virtual const Interval<double>& range() = 0;
  virtual std::unique_ptr<ITransform> transform(double _par) = 0;
  virtual VectorD<3> transform(double _par,
                               const VectorD<3>& _pos,
                               const VectorD<3>* _dir = nullptr) = 0;

  static std::unique_ptr<ITrajectory>
    make_linear(const Interval<double>& _interv,
      const ITransform& _start,
      const ITransform& _end);

#if 0
  static std::shared_ptr<ITrajectory>
    make_rotation(const Interval<double>& _interv,
                  const VectorD<3>& _ax, const double& _al0, const double& _al1);
  static std::shared_ptr<ITrajectory>
    make_interpolate(const Interval<double>& _interv,
                     const Transform& _tr_beg, const Transform& _tr_end);
  static std::shared_ptr<ITrajectory>
    make_composite(const Interval<double>& _interv,
                   const Transform& _tr_beg, const Transform& _tr_end);
#endif
};


}