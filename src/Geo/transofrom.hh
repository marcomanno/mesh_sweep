#pragma once
#include "range.hh"

#include <memory>

namespace Geo
{

struct Transform
{
  // The lenght of the rotation is sqrt(angle)
  void set_rotation_axis(const VectorD<3>& _axis) { axis_ = _axis;  }
  const VectorD<3>& get_rotation_axis() const { return axis_; }

  void set_rotation_origin(const VectorD<3>& _origin) { origin_ = _origin; }
  const VectorD<3>& get_rotation_origin() const { return origin_; }

  void set_translation(const VectorD<3> & _delta) { delta_ = _delta; }
  const VectorD<3>& get_translation() const { return delta_; }

  VectorD<3> operator()(const VectorD<3> & _pos);

  static Transform interpolate(const Transform& _start, 
    const Transform& _end, double _par, Transform* _d_trnsf = nullptr);
  static VectorD<3> rotation_axis(const VectorD<3>& _direction, double angle);

  Transform& operator+=(const Transform& _oth);
  Transform operator+(const Transform& _oth) const { return (Transform(*this) += _oth); }
  Transform& operator-=(const Transform& _oth);
  Transform operator-(const Transform& _oth) const { return (Transform(*this) -= _oth); }
  Transform& operator*=(double _coeff);
  Transform operator*(double _coeff) const { return (Transform(*this) *= _coeff); }
  Transform& operator/=(double _coeff);
  Transform operator/(double _coeff) const { return (Transform(*this) /= _coeff); }

private:
  VectorD<3> origin_ = {};
  VectorD<3> delta_ = {};
  VectorD<3> axis_ = {};
};

struct ITrajectory
{
  virtual ~ITrajectory() {}
  virtual const Interval<double>& range() = 0;
  virtual Transform transform(double _par, Transform* _d_transf = nullptr) = 0;
  virtual VectorD<3> evaluate(double _par,
                              const VectorD<3>& _pos,
                              VectorD<3>* _dir = nullptr) = 0;

  static std::unique_ptr<ITrajectory>
    make_linear(const Interval<double>& _interv,
      const Transform& _start,
      const Transform& _end);

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

} // namespace Geo

inline Geo::Transform operator*(double _coeff, const Geo::Transform& _trnsf) { return _trnsf * _coeff; }
