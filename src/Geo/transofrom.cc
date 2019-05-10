#include "transofrom.hh"

#ifdef QUATERNION
#include <Eigen/Geometry>
using Quat = Eigen::Quaternion<double>;
#endif

using namespace Geo;

PIMPL_STRUCT_IMPL(Transform)
{
  VectorD<3> origin_ = {};
  VectorD<3> delta_ = {};
  VectorD<3> operator()(const VectorD<3> & _pos);
#ifdef QUATERNION
  Quat rot_;
#else
  VectorD<3> axis_ = {};
#endif
};

PIMPL_STRUCT_METHODS(Transform)

namespace Geo
{

void Transform::set_rotation(const VectorD<3>& _axis, double _angle, const VectorD<3>& _origin)
{
  impl_->origin_ = _origin;
#ifdef QUATERNION
  Eigen::Vector3d axis{ _axis[0], _axis[1], _axis[2] };
  impl_->rot_ = Eigen::AngleAxis(_angle, axis);
  impl_->rot_.normalize();
#else
  impl_->axis_ = _axis;
  normalize(impl_->axis_);
  impl_->axis_ *= sqrt(_angle);
#endif
}

void Transform::set_translation(const VectorD<3>& _delta)
{
  impl_->delta_ = _delta;
}

VectorD<3> Transform::operator()(const VectorD<3>& _pos)
{
  auto p = _pos - impl_->origin_;
  VectorD<3> rot_v{};
#ifdef QUATERNION
  Quat::Vector3 v(p[0], p[1], p[2]);
  auto w = impl_->rot_._transformVector(v);
  rot_v = {w[0], w[1], w[2]}
#else
  auto axis = impl_->axis_;
  auto len = normalize(axis);
  if (len != 0)
  {
    auto alpha = sq(len);
    auto z_dir = (p * axis) * axis;
    auto x_dir = p - z_dir;
    auto y_dir = axis % x_dir;
    rot_v = z_dir + x_dir * cos(alpha) + y_dir * sin(alpha);
  }
#endif
  return impl_->origin_ + impl_->delta_ + rot_v;
}

Transform Transform::interpolate(const Transform& _end, double _par) const
{
  Transform result;
  result.set_translation(::interpolate(impl_->delta_, _end.impl_->delta_, _par));
  result.impl_->origin_ = ::interpolate(impl_->origin_, _end.impl_->origin_, _par);
#ifdef QUATERNION
  result.impl_->rot_ = impl_->rot_.slerp(_par, _end.impl_->rot_);
#else
  result.impl_->axis_ = ::interpolate(impl_->axis_, _end.impl_->axis_, _par);
#endif
  return result;
}

struct Trajectory : public ITrajectory
{
  const Interval<double>& range() override { return range_; }
  Interval<double> range_;
};
#if 0
struct TrajectoryLinear : public Trajectory
{
  Transform transform(double _par) override
  {
    Transform res;
    auto t = (_par - range_[0]) / range_.length();
    res .delta_ = trnsf_.delta_ * (1 - t) + end_pos_;
    res.rotation_ = trnsf_.rotation_;
    return res;
  }
  VectorD<3> transform(double _par,
                       const VectorD<3>& _pos,
                       const VectorD<3>* _dir = nullptr)
  {
    auto trnsf = transform(_par);
    return trnsf(_pos);
  }

  Transform trnsf_;
  VectorD3 end_pos_;
};

std::shared_ptr<ITrajectory>
ITrajectory::make_linear(const Interval<double>& _interv,
                         const VectorD<3>& _start, const VectorD<3>& _end,
                         const VectorD<3>* _rot)
{
  auto res = std::make_shared<TrajectoryLinear>();
  res->range_ = _interv;
  res->end_pos_ = _end;
  res->trnsf_.delta_ = _start;
  if (_rot != nullptr)
    res->trnsf_.rotation_ = *_rot;
  return res;
}

struct TrajectoryRotation : public ITrajectory
{
  TRAJECTORY_METHODS(, override);
};

struct TrajectoryInterpolate : public ITrajectory
{
  TRAJECTORY_METHODS(, override);
};

struct TrajectoryCompose : public ITrajectory
{
  TRAJECTORY_METHODS(, override);
};

#endif

} // nemespace Geo
