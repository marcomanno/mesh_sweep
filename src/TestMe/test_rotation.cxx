
#include "catch2/catch.hpp"

#include "Geo/transofrom.hh"
#include "Import/load_obj.hh"
#include "Import/save_obj.hh"
#include "Topology/iterator.hh"


#include <filesystem>

static std::string out_file(const char* _flnm);

TEST_CASE("Cube", "[Rotations]")
{
  Geo::Transform trns;
  trns.set_rotation({ 0, 0 ,1 }, M_PI);
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
  Geo::Transform a;
  a.set_rotation({ 0, 0, 1 }, M_PI / 2);

  Geo::Transform b;
  b.set_rotation({ 0, 1, 0 }, M_PI / 2);

  for (double x = 0; x <= 1; x += 0.125)
  {
    Geo::Transform trns = a.interpolate(b, x);
    auto body = IO::load_obj(INDIR"/cube.obj");
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(body);
    for (auto v : bv)
    {
      Geo::VectorD3 pt;
      v->geom(pt);
      v->set_geom(trns(pt));
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
  b.set_rotation({ 1, 0, 0 }, M_PI / 2);

  for (double x = 0; x <= 1; x += 0.125)
  {
    Geo::Transform trns = a.interpolate(b, x);
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

static std::string out_file(const char* _flnm)
{
  namespace fs = std::filesystem;
  if (!fs::is_directory(OUTDIR) || !fs::exists(OUTDIR))
    fs::create_directory(OUTDIR); // create _dir folder
  return std::string(OUTDIR"/") + _flnm;
}
