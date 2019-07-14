
#include "convert.hxx"
#include "Topology/impl.hh"
#include "Topology/iterator.hh"

#pragma warning (disable: 4244 4146 4267)

#include <openvdb/openvdb.h>
#include "openvdb/tools/MeshToVolume.h"
#include "openvdb/tools/Composite.h"
#include "openvdb/tools/VolumeToMesh.h"

namespace Levelset {

using Grid = openvdb::FloatGrid;

static Grid::Ptr convert_to_levelset(Topo::Wrap<Topo::Type::BODY> _body)
{
  static bool init = false;
  if (!init)
    openvdb::initialize();
  std::vector<openvdb::Vec3f> points;
  std::vector<openvdb::Vec3I> tris;
  std::map<Topo::Wrap<Topo::Type::VERTEX>, size_t> v_ind;
  Topo::Iterator<Topo::Type::BODY, Topo::Type::VERTEX> bv(_body);
  for (auto v : bv)
  {
    Geo::VectorD3 pos;
    v->geom(pos);
    v_ind[v] = points.size();
    points.emplace_back(pos[0], pos[1], pos[2]);
  }
  Topo::Iterator<Topo::Type::BODY, Topo::Type::FACE> bf(_body);
  for (auto f : bf)
  {
    Topo::Iterator<Topo::Type::FACE, Topo::Type::VERTEX> fe(f);
    if (fe.size() != 3)
      continue;
    tris.emplace_back(v_ind[fe.get(0)], v_ind[fe.get(1)], v_ind[fe.get(2)]);
  }
  openvdb::tools::QuadAndTriangleDataAdapter<openvdb::Vec3f, openvdb::Vec3I> mesh(points, tris);
  return openvdb::tools::meshToVolume<Grid>(mesh, 
    *openvdb::math::Transform::createLinearTransform());
}

struct MeshJoin::Impl
{
  std::vector<Topo::Wrap<Topo::Type::BODY>> meshes_;
  Topo::Wrap<Topo::Type::BODY> compute();
};

MeshJoin::~MeshJoin()
{
  delete impl_;
}

void MeshJoin::add_mesh(Topo::Wrap<Topo::Type::BODY>& _body)
{
  if (!impl_)
    impl_ = new Impl;
  impl_->meshes_.push_back(_body);
}


Topo::Wrap<Topo::Type::BODY> MeshJoin::compute()
{
  return impl_->compute();
}

Topo::Wrap<Topo::Type::BODY> MeshJoin::Impl::compute()
{
  if (meshes_.empty())
    return Topo::Wrap<Topo::Type::BODY>();
  auto it_mesh = meshes_.begin();
  auto grid = convert_to_levelset(*it_mesh);
  while (++it_mesh != meshes_.end())
  {
    auto grid1 = convert_to_levelset(*it_mesh);
    openvdb::tools::csgUnion(*grid, *grid1);
  }
  Topo::Wrap<Topo::Type::BODY> new_body;
  new_body.make<Topo::EE<Topo::Type::BODY>>();

  std::vector<openvdb::math::Vec3s> points;
  std::vector<openvdb::Vec4I> quads;
  openvdb::tools::volumeToMesh(*grid, points, quads, 0.0);
  std::vector<Topo::Wrap<Topo::Type::VERTEX>> verts;
  for (auto& pt : points)
    verts.emplace_back().make<Topo::EE<Topo::Type::VERTEX>>();
  for (auto& f : quads)
  {
    Topo::Wrap<Topo::Type::FACE> face;
    face.make<Topo::EE<Topo::Type::FACE>>();
    new_body->insert_child(face.get());
    for (size_t i = 0; i < 4; ++i)
      face->insert_child(verts[f[i]].get());
  }
  return new_body;
}
} // namespace Levelset
