
#include "geom.hh"
#include "iterator.hh"
#include "Utils/error_handling.hh"

#include <set>

namespace Topo {

Geo::VectorD3 face_normal(Topo::Wrap<Topo::Type::FACE> _face)
{
  Iterator<Type::FACE, Type::LOOP> fl_it(_face);
  THROW_IF(fl_it.size() == 0, "Normal ofnot defined face");
  Iterator<Type::LOOP, Type::VERTEX> lv_it(*fl_it.begin());
  std::vector<Geo::Point> verts;
  for (auto vert : lv_it)
    vert->geom(verts.emplace_back());
  auto poly_face = Geo::IPolygonalFace::make();
  poly_face->add_loop(verts.begin(), verts.end());
  poly_face->compute();
  return poly_face->normal();
}

Geo::VectorD3 coedge_normal(Topo::Wrap<Topo::Type::COEDGE> _coe)
{
  Iterator<Type::COEDGE, Type::FACE> ef_it(_coe);
  return face_normal(ef_it.get(0));
}

Geo::VectorD3 coedge_direction(Topo::Wrap<Topo::Type::COEDGE> _coed)
{
  Geo::Segment seg;
  _coed->geom(seg);
  return seg[1] - seg[0];
}

double edge_angle(Topo::Wrap<Topo::Type::EDGE> _ed)
{
  Iterator<Type::EDGE, Type::COEDGE> ec_it(_ed);
  if (ec_it.size() != 2)
    return 0;
  auto coe_dir = coedge_direction(ec_it.get(0));
  auto norm0 = coedge_normal(ec_it.get(0));
  auto norm1 = coedge_normal(ec_it.get(1));
  return Geo::signed_angle(norm0, norm1, coe_dir);
}

std::vector<Geo::VectorD3> vertex_normals(Wrap<Type::VERTEX> _vert)
{
  struct VertexLoop
  {
    Wrap<Topo::Type::EDGE> edge_;
    Wrap<Topo::Type::FACE> f0_, f1_;
  };
  std::vector<VertexLoop> v_loop;
  Iterator<Type::VERTEX, Type::EDGE> ve_it(_vert);
  std::set<Wrap<Topo::Type::FACE>> faces;
  for (auto ed : ve_it)
  {
    Iterator<Type::EDGE, Type::FACE> ef_it(ed);
    THROW_IF(ef_it.size() != 2, "Expected 2 manifold mesh.");
    v_loop.push_back({ ed, ef_it.get(0), ef_it.get(1) });
  }
  for (size_t i = 0; i < v_loop.size(); ++i)
  {
    auto& ref = v_loop[i];
    for (size_t j = i + 1; j < v_loop.size(); ++j)
    {
      auto& ref1 = v_loop[j];
      if (ref.f1_ != ref1.f0_)
      {
        if (ref.f1_ != ref1.f1_)
          continue;
        std::swap(ref1.f0_, ref1.f1_);
      }
      std::swap(ref1, v_loop[i + 1]);
      break;
    }
  }
  THROW_IF(v_loop.front().f0_ != v_loop.back().f1_, "Expected cycle.");
  std::vector<Geo::VectorD3> normals;
  for (auto& vert_loop : v_loop)
    normals.push_back(face_normal(vert_loop.f1_));
  return normals;
}

namespace PointInFace {
Geo::PointInPolygon::Classification classify(
  const Topo::Wrap<Topo::Type::FACE>& _face, const Geo::Point& _pt)
{
  Topo::Iterator<Topo::Type::FACE, Topo::Type::LOOP> fl_it(_face);
  Geo::PointInPolygon::Classification out_res = Geo::PointInPolygon::Classification::Outside;
  for (auto& loop : fl_it)
  {
    Topo::Iterator<Topo::Type::LOOP, Topo::Type::VERTEX> fv_it(loop);
    std::vector<Geo::Point> polygon(fv_it.size());
    for (int i = 0; i < fv_it.size(); ++i)
      fv_it.get(i)->geom(polygon[i]);
    auto pt_cl = Geo::PointInPolygon::classify(polygon, _pt);
    if (pt_cl == Geo::PointInPolygon::Classification::On)
      return pt_cl;
    if (pt_cl == out_res)
      return Geo::PointInPolygon::Classification::Outside;
    out_res = Geo::PointInPolygon::Classification::Inside;
  }
  return Geo::PointInPolygon::Classification::Inside;
}

Geo::PointInPolygon::Classification classify(
  const VertexChain& _vert_ch, const Geo::Point& _pt)
{
  std::vector<Geo::Point> polygon(_vert_ch.size());
  for (int i = 0; i < _vert_ch.size(); ++i)
    _vert_ch[i].get()->geom(polygon[i]);
  return Geo::PointInPolygon::classify(polygon, _pt);
}
}//namespace PointInFace
}//namespace Topo