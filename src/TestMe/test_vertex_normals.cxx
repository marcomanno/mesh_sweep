
#include "catch2/catch.hpp"

#include "utils.hxx"

#include "Geo/transofrom.hh"
#include "Import/load_obj.hh"
#include "Import/save_obj.hh"
#include "Topology/geom.hh"
#include "Topology/impl.hh"
#include "Topology/iterator.hh"

#include <filesystem>
#include <map>
#include <set>

static Topo::Wrap<Topo::Type::BODY> clone_body(const Topo::Wrap<Topo::Type::BODY> _body)
{
  Topo::Wrap<Topo::Type::BODY> new_body;
  new_body.make<Topo::EE<Topo::Type::BODY>>();
  std::map<Topo::Wrap<Topo::Type::VERTEX>, Topo::Wrap<Topo::Type::VERTEX>> vert_map;
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(_body);
  for (auto v : bv)
  {
    Geo::VectorD3 pos;
    v->geom(pos);
    auto& new_vert = vert_map[v];
    new_vert.make<Topo::EE<Topo::Type::VERTEX>>();
    new_vert->set_geom(pos);
  }
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf(_body);
  for (auto f : bf)
  {
    Topo::Wrap<Topo::Type::FACE> face;
    face.make<Topo::EE<Topo::Type::FACE>>();
    new_body->insert_child(face.get());
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fv(f);
    for (auto v : fv)
      face->insert_child(vert_map[v].get());
  }
  return new_body;
}

static Topo::Wrap<Topo::Type::BODY> transform_and_purge(
  Topo::Wrap<Topo::Type::BODY> _body,
  const Geo::ITrajectory& _traj, double _par,
  int _plus_minus = 0)
{
  auto body = clone_body(_body);
  std::set<Topo::Wrap<Topo::Type::FACE>> faces;
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(body);
  std::vector<Geo::VectorD3> dirs;
  for (auto v : bv)
  {
    Geo::VectorD3 pos;
    v->geom(pos);
    v->set_geom(_traj.evaluate(_par, pos, &dirs.emplace_back()));
  }
  auto dir_it = dirs.begin();
  for (auto v : bv)
  {
    auto normals = Topo::vertex_normals(v);
    int plus_minus = _plus_minus;
    const double eps = 1e-10;
    const auto& dir = *dir_it++;
    for (auto& norm : normals)
    {
      auto dot = norm * dir / Geo::length(dir);
      if (dot > eps)
        plus_minus |= 1;
      else if (dot < -eps)
        plus_minus |= 2;
      else
        plus_minus = 3;
    }
    if (plus_minus != 3)
      continue;
    Topo::Iterator<Topo::Type::VERTEX, Topo::Type::FACE> vf(v);
    for (auto f : vf)
      faces.insert(f);
  }
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf(body);
  for (auto f : bf)
  {
    if (faces.find(f) == faces.end())
      f->remove();
  }
  return body;
}

TEST_CASE("vnormals_000", "[Vertex_normal]")
{
  auto body = IO::load_obj(INDIR"/torus.obj");
  Geo::VectorD3 dir{ 0, 0, 1 };
  std::set<Topo::Wrap<Topo::Type::FACE>> faces;
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(body);
  for (auto v : bv)
  {
    auto normals = Topo::vertex_normals(v);
    int plus_minus = 0;
    for (auto& norm : normals)
    {
      if (norm * dir > 0)
        plus_minus |= 1;
      else
        plus_minus |= 2;
    }
    if (plus_minus == 3)
    {
      Topo::Iterator<Topo::Type::VERTEX, Topo::Type::FACE> vf(v);
      faces.insert(vf.begin(), vf.end());
    }
  }
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf(body);
  for (auto f : bf)
  {
    if (faces.find(f) == faces.end())
      f->remove();
  }
  
  IO::save_obj(out_file("torus").c_str(), body);
}

TEST_CASE("vnormals_001", "[Vertex_normal]")
{
  std::unique_ptr<Geo::ITrajectory> traj;
  {
    Geo::Transform a;
    a.set_rotation_axis(Geo::Transform::rotation_axis({ 0, 0, 1 }, 0));

    Geo::Transform b;
    b.set_rotation_axis(Geo::Transform::rotation_axis({ 0, 0, 1 }, 3 * M_PI));
    b.set_translation({ 0, 0, 100. });

    traj = Geo::ITrajectory::make_linear(Geo::Interval(0., 1.), a, b);
  }
  auto body = IO::load_obj(INDIR"/torus.obj");
  auto compute = [&](double _par, int _portion)
  {
    IO::save_obj(out_file("torus_tr.obj").c_str(),
      transform_and_purge(body, *traj, _par, _portion));
  };
  compute(1., 2);
  compute(0, 1);
  const double step = 1. / 128;
  for (double t = step; t < 1; t += step)
    compute(t, 0);
}