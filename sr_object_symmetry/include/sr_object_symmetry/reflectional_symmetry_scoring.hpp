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

#ifndef SR_OBJECT_SYMMETRY_REFLECTIONAL_SYMMETRY_SCORING_H
#define SR_OBJECT_SYMMETRY_REFLECTIONAL_SYMMETRY_SCORING_H

// PCL includes
#include <pcl/search/kdtree.h>

#include <vector>

// Symmetry
#include <sr_object_symmetry/reflectional_symmetry.hpp>

namespace sym
{
/** \brief Calculate how well a symmetry hypothesis fits the individual points
   * of a pointcloud.
   * Two measures are calculated:
   *  1. For points that have a symmetric correspondece the symmetry score is
   * calculated (angle between the normals of the points making a correspondence).
   *  2. For all of the points of the cloud the occlusion score is calculated
   * based on the distance to the closest occluded/occupied cell
   *  \param[in]  cloud_search_tree         a precomputed search tree for the full resolution cloud
   *  \param[in]  cloud_ds                  a downsampled input cloud
   *  \param[in]  symmetry                  input symmetry
   *  \param[out] symmetric_correspondences symmetric correspondences
   *  \param[out] point_symmetry_scores     symmetry scores for points of the cloud that have symmetric correspondences
   *  \param[in]  max_sym_corresp_reflected_distance  maximum distance between the first point of a symmetric
 * correspondence and a reflection of the second point
   *  \param[in]  min_inlier_normal_angle   minimum normal fit angle for an inlier symmetric correspondence
   *  \param[in]  max_inlier_normal_angle   maximum normal fit angle for an inlier symmetric correspondence
   */
template <typename PointT>
inline float reflSymPointSymmetryScores(
    const typename pcl::search::KdTree<PointT>& cloud_search_tree, const pcl::PointCloud<PointT>& cloud_ds,
    //                                       const Eigen::Vector4f &table_plane,
    const std::vector<int>& cloud_boundary_point_ids, const std::vector<int>& cloud_ds_boundary_point_ids,
    const sym::ReflectionalSymmetry& symmetry, pcl::Correspondences& symmetric_correspondences,
    std::vector<float>& point_symmetry_scores, const float max_sym_corresp_reflected_distance = 0.01f,
    const float min_inlier_normal_angle = pcl::deg2rad(10.0f),
    const float max_inlier_normal_angle = pcl::deg2rad(15.0f))
{
  //--------------------------------------------------------------------------
  // Check input

  typename pcl::PointCloud<PointT>::ConstPtr cloud(new pcl::PointCloud<PointT>);
  cloud = cloud_search_tree.getInputCloud();

  if (cloud->size() == 0 || cloud_ds.size() == 0)
  {
    return false;
  }

  symmetric_correspondences.resize(0);
  point_symmetry_scores.resize(0);
  //     Eigen::Vector3f planePoint, planeNormal;
  //     utl::planeCoefficientsToPointNormal<float>(table_plane, planePoint, planeNormal);
  //--------------------------------------------------------------------------
  // Calculate point errors

  // Loop over downsampled points
  for (size_t pointId = 0; pointId < cloud_ds.size(); pointId++)
  {
    // Get point normal
    Eigen::Vector3f srcPoint = cloud_ds.points[pointId].getVector3fMap();
    Eigen::Vector3f srcNormal = cloud_ds.points[pointId].getNormalVector3fMap();

    // Reflect point and normal
    Eigen::Vector3f srcPointReflected = symmetry.reflectPoint(srcPoint);
    Eigen::Vector3f srcNormalRefleclted = symmetry.reflectNormal(srcNormal);

    // Find neighbours in a radius
    std::vector<float> distancesSquared(1);
    std::vector<int> neighbours(1);
    PointT searchPoint;
    searchPoint.getVector3fMap() = srcPointReflected;
    searchPoint.getNormalVector3fMap() = srcNormalRefleclted;
    cloud_search_tree.nearestKSearch(searchPoint, 1, neighbours, distancesSquared);

    // If a point has a symmetric correspondence
    if (distancesSquared[0] <= max_sym_corresp_reflected_distance * max_sym_corresp_reflected_distance)
    {
      Eigen::Vector3f tgtNormal = cloud->points[neighbours[0]].getNormalVector3fMap();

      // If point belongs to segment boundary, we reduce it's score in half, since normals at the boundary of the
      // segment are usually noisy
      if (std::find(cloud_ds_boundary_point_ids.begin(), cloud_ds_boundary_point_ids.end(), pointId) !=
              cloud_ds_boundary_point_ids.end() ||
          std::find(cloud_boundary_point_ids.begin(), cloud_boundary_point_ids.end(), neighbours[0]) !=
              cloud_boundary_point_ids.end())
        continue;

      // Calculate error
      float symmetryScore = sym::getReflSymNormalFitError(srcNormal, tgtNormal, symmetry, true);

      // NOTE: this is to avoid the problem with correspondences between thin walls.
      // Correspondences between thin walls are likely to have normals that are 180 degrees appart.
      if (symmetryScore > M_PI * 3 / 4)
        symmetryScore = M_PI - symmetryScore;

      symmetryScore = (symmetryScore - min_inlier_normal_angle) / (max_inlier_normal_angle - min_inlier_normal_angle);
      symmetryScore = utl::clampValue(symmetryScore, 0.0f, 1.0f);

      // If all checks passed - add correspondence
      symmetric_correspondences.push_back(pcl::Correspondence(pointId, neighbours[0], distancesSquared[0]));
      point_symmetry_scores.push_back(symmetryScore);
    }
  }

  return true;
}

/** \brief Calculate how well a symmetry hypothesis fits the individual points
   * of a pointcloud.
   * Two measures are calculated:
   *  1. For points that have a symmetric correspondece the symmetry score is
   * calculated (angle between the normals of the points making a correspondence).
   *  2. For all of the points of the cloud the occlusion score is calculated
   * based on the distance to the closest occluded/occupied cell
   *  \param[in]  cloud                           input cloud
   *  \param[in]  symmetry                        input symmetry
   *  \param[out] point_perpendicularity_scores   occlusion scores for all points of the cloud
   */
template <typename PointT>
inline bool reflSymPointPerpendicularScores(const pcl::PointCloud<PointT>& cloud,
                                            const sym::ReflectionalSymmetry& symmetry,
                                            std::vector<float>& point_perpendicular_scores,
                                            const float min_perpendicular_angle = pcl::deg2rad(45.0f),
                                            const float max_perpendicular_angle = pcl::deg2rad(80.0f))
{
  //--------------------------------------------------------------------------
  // Check input

  if (cloud.size() == 0)
  {
    return false;
  }

  point_perpendicular_scores.resize(cloud.size());

  //--------------------------------------------------------------------------
  // Calculate point errors

  // Loop over downsampled points
  for (size_t pointId = 0; pointId < cloud.size(); pointId++)
  {
    Eigen::Vector3f pointNormal = cloud.points[pointId].getNormalVector3fMap();
    float angle = utl::lineLineAngle<float>(pointNormal, symmetry.getNormal());
    angle = (angle - min_perpendicular_angle) / (max_perpendicular_angle - min_perpendicular_angle);
    angle = utl::clampValue(angle, 0.0f, 1.0f);
    point_perpendicular_scores[pointId] = angle;
  }

  return true;
}
}  // namespace sym

#endif  // SR_OBJECT_SYMMETRY_REFLECTIONAL_SYMMETRY_SCORING_H
