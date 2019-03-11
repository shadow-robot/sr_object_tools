/**
* Copyright 2019 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

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
