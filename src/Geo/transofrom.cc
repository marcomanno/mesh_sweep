#include "transofrom.hh"

#ifdef QUATERNION
#include <Eigen/Geometry>
using Quat = Eigen::Quaternion<double>;
#endif

namespace Geo {

struct Transform : public ITransform
{
  void set_rotation_axis(const VectorD<3>& _axis) override
  {
    axis_ = _axis;
  }
  const VectorD<3>& get_rotation_axis() const override
  {
    return axis_;
  }
  void set_rotation_origin(const VectorD<3>& _origin) override
  {
    origin_ = _origin;
  }
  const VectorD<3>& get_rotation_origin() const override
  {
    return origin_;
  }
  void set_translation(const VectorD<3>& _delta) override
  {
    delta_ = _delta;
  }
  const VectorD<3>& get_translation() const
  {
    return delta_;
  };
  std::unique_ptr<ITransform> clone() const  override
  {
    auto res = std::make_unique<Transform>();
    res->set_rotation_axis(axis_);
    res->set_rotation_origin(origin_);
    res->set_translation(delta_);
    return res;
  }
  VectorD<3> operator()(const VectorD<3>& _pos) override
  {
    auto p = _pos - origin_;
    VectorD<3> rot_v{};
#ifdef QUATERNION
    Quat::Vector3 v(p[0], p[1], p[2]);
    auto w = impl_->rot_._transformVector(v);
    rot_v = { w[0], w[1], w[2] }
#else
    auto axis = axis_;
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
    return origin_ + delta_ + rot_v;
  }

private:
  VectorD<3> origin_ = {};
  VectorD<3> delta_ = {};
#ifdef QUATERNION
  Quat rot_;
#else
  VectorD<3> axis_ = {};
#endif
};

std::unique_ptr<ITransform> ITransform::make()
{
  return std::make_unique<Transform>();
}

std::unique_ptr<ITransform> ITransform::interpolate(const ITransform& _start, const ITransform& _end, double _par)
{
  auto result = std::make_unique<Transform>();
  result->set_translation(Geo::interpolate(_start.get_translation(), _end.get_translation(), _par));
  result->set_rotation_axis(Geo::interpolate(_start.get_rotation_axis(), _end.get_rotation_axis(), _par));
  result->set_rotation_origin(Geo::interpolate(_start.get_rotation_origin(), _end.get_rotation_origin(), _par));
  return result;
}

VectorD<3> ITransform::rotation_axis(const VectorD<3>& _direction, double angle)
{
  if (angle < 1e-12)
    return { };
  auto len = length(_direction);
  if (len < 1e-12)
    throw "Undefined direction";
  return sqrt(angle) / len * _direction;
}


struct Trajectory : public ITrajectory
{
  const Interval<double>& range() override { return range_; }
  Interval<double> range_;
};

struct TrajectoryLinear : public Trajectory
{
  std::unique_ptr<ITransform> transform(double _par) override
  {
    auto t = (_par - range_[0]) / range_.length();
    return ITransform::interpolate(*start_, *end_, t);
  }

  VectorD<3> transform(double _par,
    const VectorD<3>& _pos,
    const VectorD<3>* _dir = nullptr) override
  {
    auto trnsf = transform(_par);
    return (*trnsf)(_pos);
  }

  std::unique_ptr<ITransform> start_, end_;
  Interval<double> range_;
};

std::unique_ptr<ITrajectory>
ITrajectory::make_linear(const Interval<double>& _interv,
  const ITransform& _start, const ITransform& _end)
{
  auto res = std::make_unique<TrajectoryLinear>();
  res->range_ = _interv;
  res->start_ = _start.clone();
  res->end_ = _end.clone();
  return res;
}

} // nemespace Geo
