
#include "catch2/catch.hpp"

#include "Geo/transofrom.hh"
#include "Import/load_obj.hh"
#include "Import/save_obj.hh"
#include "Topology/iterator.hh"


#include <filesystem>

static std::string out_file(const char* _flnm);

TEST_CASE("Cube", "[Rotations]")
{
  auto trns = Geo::ITransform::make();
  trns->set_rotation_axis(Geo::ITransform::rotation_axis({ 0, 0 ,1 }, M_PI));
  auto body = IO::load_obj(INDIR"/cube.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(body);
  for (auto v : bv)
  {
    Geo::VectorD3 pt;
    v->geom(pt);
    v->set_geom((*trns)(pt));
  }
  IO::save_obj(out_file("cube.obj").c_str(), body);
}

TEST_CASE("Cube1", "[Rotations]")
{
  auto a = Geo::ITransform::make();
  a->set_rotation_axis(Geo::ITransform::rotation_axis({ 0, 0, 1 }, M_PI / 2));

  auto b = Geo::ITransform::make();
  b->set_rotation_axis(Geo::ITransform::rotation_axis({ 0, 1, 0 }, M_PI / 2));

  for (double x = 0; x <= 1; x += 0.125)
  {
    auto trns = Geo::ITransform::interpolate(*a, *b, x);
    auto body = IO::load_obj(INDIR"/cube.obj");
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(body);
    for (auto v : bv)
    {
      Geo::VectorD3 pt;
      v->geom(pt);
      v->set_geom((*trns)(pt));
    }
    auto flnm = out_file("cube1_");
    flnm += std::to_string(x);
    flnm += ".obj";
    IO::save_obj(flnm.c_str(), body);
  }
}

TEST_CASE("Cube2", "[Rotations]")
{
  auto a = Geo::ITransform::make();
  auto b = Geo::ITransform::make();
  b->set_rotation_axis(Geo::ITransform::rotation_axis({ 1, 0, 0 }, M_PI / 2));

  for (double x = 0; x <= 1; x += 0.125)
  {
    auto trns = Geo::ITransform::interpolate(*a, *b, x);
    auto body = IO::load_obj(INDIR"/cube1.obj");
    Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(body);
    for (auto v : bv)
    {
      Geo::VectorD3 pt;
      v->geom(pt);
      v->set_geom((*trns)(pt));
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
