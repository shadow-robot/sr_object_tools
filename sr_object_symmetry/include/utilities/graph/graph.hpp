/**
 * Copyright 2017 Aleksandrs Ecins
 * Copyright (C) 2018 Shadow Robot Company Ltd
￼ * Licensed under GPLv2+
 * Refer to the LICENSE.txt file included
**/

#ifndef UTILITIES_GRAPH_GRAPH_H
#define UTILITIES_GRAPH_GRAPH_H

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

#endif  // UTILITIES_GRAPH_GRAPH_H
