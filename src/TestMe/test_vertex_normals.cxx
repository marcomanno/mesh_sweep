
#include "catch2/catch.hpp"

#include "utils.hxx"

#include "Geo/transofrom.hh"
#include "Import/load_obj.hh"
#include "Import/save_obj.hh"
#include "Topology/geom.hh"
#include "Topology/impl.hh"
#include "Topology/iterator.hh"

#include <filesystem>
#include <set>

TEST_CASE("vnormals_000", "[Vertex_normal]")
{
  auto body = IO::load_obj(INDIR"/torus.obj");
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(body);
  Geo::VectorD3 dir{ 0, 0, 1 };
  std::set<Topo::Wrap<Topo::Type::FACE>> faces;
  for (auto v : bv)
  {
    auto normals = vertex_normals(v);
    int plus_minus = 0;
    for (auto& norm : normals)
    {
      if (norm * dir > 0)
        plus_minus |= 1;
      else
        plus_minus |= 2;
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
  
  IO::save_obj(out_file("torus").c_str(), body);
}
