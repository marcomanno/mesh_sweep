
#include "utils.hxx"

#include "catch2/catch.hpp"

#include "Geo/transofrom.hh"
#include "Import/load_obj.hh"
#include "Import/save_obj.hh"
#include "Topology/iterator.hh"


#include <filesystem>

TEST_CASE("Cube", "[Rotations]")
{
  Geo::Transform trns;;
  trns.set_rotation_axis(Geo::Transform::rotation_axis({ 0, 0 ,1 }, M_PI));
  auto body = IO::load_obj(INDIR"/cube.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(body);
  for (auto v : bv)
  {
    Geo::VectorD3 pt;
    v->geom(pt);
    v->set_geom(trns(pt));
  }
  IO::save_obj(out_file("cube.obj").c_str(), body);
}

TEST_CASE("Cube1", "[Rotations]")
{
  std::unique_ptr<Geo::ITrajectory> traj;
  {
    Geo::Transform a;
    a.set_rotation_axis(Geo::Transform::rotation_axis({ 0, 0, 1 }, M_PI / 2));

    Geo::Transform b;
    b.set_rotation_axis(Geo::Transform::rotation_axis({ 0, 1, 0 }, M_PI / 2));

    traj = Geo::ITrajectory::make_linear(Geo::Interval(0., 1.), a, b);
  }

  for (double x = 0; x <= 1; x += 0.125)
  {
    auto body = IO::load_obj(INDIR"/cube.obj");
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(body);
    for (auto v : bv)
    {
      Geo::VectorD3 pt, dir;
      v->geom(pt);
      auto tr_pos = traj->evaluate(x, pt, &dir);
      v->set_geom(tr_pos);
    }
    auto flnm = out_file("cube1_");
    flnm += std::to_string(x);
    flnm += ".obj";
    IO::save_obj(flnm.c_str(), body);
  }
}

TEST_CASE("Cube2", "[Rotations]")
{
  Geo::Transform a;
  Geo::Transform b;
  b.set_rotation_axis(Geo::Transform::rotation_axis({ 1, 0, 0 }, M_PI / 2));

  for (double x = 0; x <= 1; x += 0.125)
  {
    auto trns = Geo::Transform::interpolate(a, b, x);
    auto body = IO::load_obj(INDIR"/cube1.obj");
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(body);
    for (auto v : bv)
    {
      Geo::VectorD3 pt;
      v->geom(pt);
      v->set_geom(trns(pt));
    }
    auto flnm = out_file("cube2_");
    flnm += std::to_string(x);
    flnm += ".obj";
    IO::save_obj(flnm.c_str(), body);
  }
}

TEST_CASE("Cube3", "[Rotations]")
{
  std::unique_ptr<Geo::ITrajectory> traj;
  {
    Geo::Transform a;
    a.set_rotation_axis(Geo::Transform::rotation_axis({ 0, 0, 1 }, 0));

    Geo::Transform b;
    b.set_rotation_axis(Geo::Transform::rotation_axis({ 0, 0, 1 }, 3 * M_PI));
    b.set_translation({0, 0, 100.});

    traj = Geo::ITrajectory::make_linear(Geo::Interval(0., 1.), a, b);
  }

  const int N = 128;
  for (double x = 0; x <= 1; x += 1. / N)
  {
    auto body = IO::load_obj(INDIR"/cube.obj");
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(body);
    for (auto v : bv)
    {
      Geo::VectorD3 pt, dir;
      v->geom(pt);
      auto tr_pos = traj->evaluate(x, pt, &dir);
      auto h = 0.0001;
      auto tr_pos_plus = traj->evaluate(x + h, pt);
      auto tr_pos_minus = traj->evaluate(x - h, pt);
      auto dir_appr = (tr_pos_plus - tr_pos_minus) / (2 * h);
      CHECK(Geo::length(dir - dir_appr) < 1e-5);
      v->set_geom(tr_pos);
    }
    auto flnm = out_file("cube3_");
    flnm += std::to_string(x);
    flnm += ".obj";
    IO::save_obj(flnm.c_str(), body);
  }
}
