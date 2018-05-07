// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef GRAPH_GRAPH_HPP_
#define GRAPH_GRAPH_HPP_

// Utilities includes
#include <utilities/graph/graph_base.hpp>
#include <utilities/graph/graph_primitives.hpp>

namespace utl
{
/** \brief Data structure representing an undirected unwheighted graph.
   * Self loops are not allowed.
   */
class Graph : public utl::GraphBase<Vertex, Edge>
{
public:
  /** \brief Empty constructor. */
  Graph() : utl::GraphBase<utl::Vertex, utl::Edge>()
  {
  }

  /** \brief Constructor that preallocates memory for vertices. */
  explicit Graph(const int num_vertices) : utl::GraphBase<utl::Vertex, utl::Edge>(num_vertices)
  {
  }

  /** \brief Destructor. */
  ~Graph() {}
};
}  // namespace utl

#endif  // GRAPH_GRAPH_HPP_
