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

#ifndef UTILITIES_GRAPH_GRAPH_ALGORITHMS_H
#define UTILITIES_GRAPH_GRAPH_ALGORITHMS_H

// STD includes
#include <queue>
#include <vector>
// Utilities includes
#include <utilities/graph/graph_base.hpp>
#include <utilities/std_vector.hpp>
#include <utilities/map.hpp>

namespace utl
{
/** \brief Given a set of vertices in a graph return all edges between
   * the input set of vertices and the rest of the vertices in the graph.
   *  \param[in]  graph           graph
   *  \param[in]  cut_vertices    indices of vertices that were cut
   *  \param[in]  cut_edge_graph  a graph only containing the edges belonging to the cut
   */
template <typename VertexT, typename EdgeT>
inline void getCutEdges(const utl::GraphBase<VertexT, EdgeT>& graph, const std::vector<int>& cut_vertices,
                        utl::GraphBase<VertexT, EdgeT>& cut_edge_graph)
{
  cut_edge_graph.clear();

  // Loop over edges
  for (size_t edgeId = 0; edgeId < graph.getNumEdges(); edgeId++)
  {
    // Get the verices of the current edge
    EdgeT edge;
    graph.getEdge(edgeId, edge);
    int vtx1Id = edge.vtx1Id_;
    int vtx2Id = edge.vtx2Id_;

    // Find edge vertices are in the cut vertex set
    std::vector<int> vtx1Loc, vtx2Loc;
    int vtx1Count = utl::vectorFind(cut_vertices, vtx1Id, vtx1Loc);
    int vtx2Count = utl::vectorFind(cut_vertices, vtx2Id, vtx2Loc);

    // Add edge if one vertex is in cut vertices but the other is not
    if ((vtx1Count == 0 && vtx2Count > 0) || (vtx1Count > 0 && vtx2Count == 0))
      cut_edge_graph.addEdge(edge);
  }
}

/** \brief Find connected components in the graph.
   *  \param[in]  graph         graph object
   *  \param[in]  min_cc_size   minimum size of a valid connected component (default 0)
   *  \return    a vector of vectors where each inner vector corresponds to a
   *             connected component and stores the indices of vertices belonging
   *             to it.
   */
template <typename NeighborT, typename EdgeT>
inline utl::Map getConnectedComponents(const utl::GraphBase<NeighborT, EdgeT>& graph, const int min_cc_size = 0)
{
  std::vector<bool> visited(graph.getNumVertices(), false);
  utl::Map CCs;

  for (size_t vtxId = 0; vtxId < graph.getNumVertices(); vtxId++)
  {
    // If node has already been visited - skip
    if (visited[vtxId])
      continue;

    // Run breadth-first search from current vertex
    std::queue<int> vertexQueue;
    std::vector<int> CC;
    vertexQueue.push(vtxId);
    visited[vtxId] = true;

    while (!vertexQueue.empty())
    {
      // Get first vertex from the queue
      int curVtxId = vertexQueue.front();
      vertexQueue.pop();
      CC.push_back(curVtxId);

      // Loop over it's neighbors
      std::vector<int> neighbors;
      graph.getVertexNeighbors(curVtxId, neighbors);

      for (size_t nbrIt = 0; nbrIt < neighbors.size(); nbrIt++)
      {
        int nbrId = neighbors[nbrIt];

        if (!visited[nbrId])
        {
          vertexQueue.push(nbrId);
          visited[nbrId] = true;
        }
      }
    }
    if (static_cast<int>(CC.size()) > min_cc_size)
      CCs.push_back(CC);
  }

  return CCs;
}
}  // namespace utl

#endif  // UTILITIES_GRAPH_GRAPH_ALGORITHMS_H
