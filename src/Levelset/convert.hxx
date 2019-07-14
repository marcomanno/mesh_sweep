#pragma once

#include <Topology/topology.hh>

namespace Levelset
{
struct MeshJoin
{
  ~MeshJoin();
  void add_mesh(Topo::Wrap<Topo::Type::BODY>& _body);
  Topo::Wrap<Topo::Type::BODY> compute();
private:
  struct Impl;
  Impl *impl_ = nullptr;
};

} // namespace Levelset
