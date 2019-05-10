#pragma once
#include "range.hh"

#include <memory>

template <class T> struct Pimpl
{
  Pimpl();
  Pimpl(Pimpl&& _oth)
  {
    if (_oth.impl_ != impl_)
    {
      impl_ = _oth.impl_;
      _oth.impl_ = nullptr;
    }
  }
  Pimpl(const Pimpl& _oth) = delete;
  ~Pimpl() { clear(); }
  Pimpl operator=(Pimpl&& _oth)
  {
    if (_oth.impl_ != impl_)
    {
      clear();
      impl_ = _oth.impl_;
      _oth.impl_ = nullptr;
    }
  }
  Pimpl& operator=(const Pimpl& _oth) = delete;
  struct Impl;
  Impl* impl_ = nullptr;
private:
  void clear();
};

#define PIMPL_STRUCT(myclass) struct myclass : public Pimpl<myclass>
#define PIMPL_STRUCT_IMPL(myclass) template <> struct Pimpl<myclass>::Impl
#define PIMPL_STRUCT_METHODS(myclass)                      \
  template <> Pimpl<myclass>::Pimpl() : impl_(new Impl) {} \
  template <> void Pimpl<myclass>::clear() { delete impl_; }

namespace Geo
{

PIMPL_STRUCT(Transform)
{
  void set_rotation(const VectorD<3> & _axis, double _angle = 0, const VectorD<3> & _origin = {});
  void set_translation(const VectorD<3> & _delta);
  VectorD<3> operator()(const VectorD<3> & _pos);
  Transform interpolate(const Transform & _end, double _par) const;
};

struct ITrajectory
{
  virtual Transform transform(double _par) = 0;
  virtual const Interval<double>& range() = 0;
  virtual VectorD<3> transform(double _par,
                               const VectorD<3>& _pos,
                               const VectorD<3>* _dir = nullptr) = 0;

  static std::shared_ptr<ITrajectory>
    make_linear(const Interval<double>& _interv,
                const VectorD<3>& _start, const VectorD<3>& _end,
                const VectorD<3>* _rot = nullptr);
  static std::shared_ptr<ITrajectory>
    make_rotation(const Interval<double>& _interv,
                  const VectorD<3>& _ax, const double& _al0, const double& _al1);
  static std::shared_ptr<ITrajectory>
    make_interpolate(const Interval<double>& _interv,
                     const Transform& _tr_beg, const Transform& _tr_end);
  static std::shared_ptr<ITrajectory>
    make_composite(const Interval<double>& _interv,
                   const Transform& _tr_beg, const Transform& _tr_end);
};


}