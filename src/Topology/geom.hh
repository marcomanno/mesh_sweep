#pragma once
#include "Geo/entity.hh"
#include "Geo/vector.hh"
#include "Geo/point_in_polygon.hh"
#include "Topology/Topology.hh"

namespace Topo {

Geo::VectorD3 face_normal(Topo::Wrap<Topo::Type::FACE> _face);
Geo::VectorD3 coedge_normal(Topo::Wrap<Topo::Type::COEDGE> _coe);
Geo::VectorD3 coedge_direction(Topo::Wrap<Topo::Type::COEDGE> _coed);

// Edge external angle with sign. Return angle < 0 if the edge is concave.
double edge_angle(Topo::Wrap<Topo::Type::EDGE> _ed);

// Geven a vertex return an ordered list of all faces normals around the vertex.
std::vector<Geo::VectorD3> vertex_normals(Topo::Wrap<Topo::Type::VERTEX> _vert);

namespace PointInFace {

Geo::PointInPolygon::Classification classify(
  const Topo::Wrap<Topo::Type::FACE>& _face, const Geo::Point& _pt);

Geo::PointInPolygon::Classification classify(
  const VertexChain& _cert_ch, const Geo::Point& _pt);

}//namespace PointInFace

}//namespace Topo