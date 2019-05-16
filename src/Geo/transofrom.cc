#include "transofrom.hh"

#ifdef QUATERNION
#include <Eigen/Geometry>
using Quat = Eigen::Quaternion<double>;
#endif

namespace Geo {

Transform& Transform::operator+=(const Transform & _oth)
{
  origin_ += _oth.origin_;
  delta_ += _oth.delta_;
  axis_ += _oth.axis_;
  return *this;
}

Transform& Transform::operator-=(const Transform& _oth)
{
  origin_ -= _oth.origin_;
  delta_ -= _oth.delta_;
  axis_ -= _oth.axis_;
  return *this;
}

Transform& Transform::operator*=(double _coeff)
{
  origin_ *= _coeff;
  delta_ *= _coeff;
  axis_ *= _coeff;
  return *this;
}

Transform& Transform::operator/=(double _coeff)
{
  origin_ /= _coeff;
  delta_ /= _coeff;
  axis_ /= _coeff;
  return *this;
}

VectorD<3> Transform::operator()(const VectorD<3>& _pos)
{
  auto p = _pos - origin_;
  VectorD<3> rot_v(p);
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
  return origin_ + delta_ + rot_v;
}

Transform Transform::interpolate(
  const Transform& _start, const Transform& _end, double _par, Transform* _d_trnsf)
{
  if (_d_trnsf != nullptr)
    * _d_trnsf = _end - _start;
  return Geo::interpolate(_start, _end, _par);
}

VectorD<3> Transform::rotation_axis(const VectorD<3>& _direction, double angle)
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
  Transform transform(double _par, Transform* _d_transf = nullptr) override
  {
    auto t = (_par - range_[0]) / range_.length();
    auto result = Transform::interpolate(start_, end_, t, _d_transf);
    if (_d_transf != nullptr)
      * _d_transf /= range_.length();
    return result;
  }

  VectorD<3> evaluate(double _par, const VectorD<3>& _pos, VectorD<3>* _dir = nullptr) override
  {
    Transform d_transf;
    auto trnsf = transform(_par, &d_transf);
    auto new_pos = trnsf(_pos);
    if (_dir != nullptr)
    {
      *_dir = d_transf.get_rotation_origin() + d_transf.get_translation();

      auto& ax = trnsf.get_rotation_axis();
      auto ax_len_sq = length_square(ax);
      if (ax_len_sq > 1e-24)
      {
        auto p = _pos - trnsf.get_rotation_origin();
        auto& dax = d_transf.get_rotation_axis();
        auto ax_len = sqrt(ax_len_sq);

        auto alpha = ax_len_sq;
        auto dalpha = 2. * ax * dax;

        auto ax_n = ax / ax_len;
        auto dax_n = (dax - ((ax * dax) / ax_len_sq) * ax) / ax_len;

        auto tz = (p * ax_n) * ax_n;
        auto dtz = (p * dax_n) * ax_n + (p * ax_n) * dax_n;

        auto tx = p - tz;
        auto dtx = p - dtz;

        auto ty = tx % ax_n;
        auto dty = dtx % ax_n + tx % dax_n;

        double ca = cos(alpha), sa = sin(alpha);
        *_dir += dtz + dtx * ca + dty * sa + dalpha * (ty * ca - tx * sa);
      }
    }
    return new_pos;
  }

  Transform start_, end_;
  Interval<double> range_;
};

std::unique_ptr<ITrajectory>
ITrajectory::make_linear(const Interval<double>& _interv,
  const Transform& _start, const Transform& _end)
{
  auto res = std::make_unique<TrajectoryLinear>();
  res->range_ = _interv;
  res->start_ = _start;
  res->end_ = _end;
  return res;
}

} // nemespace Geo
