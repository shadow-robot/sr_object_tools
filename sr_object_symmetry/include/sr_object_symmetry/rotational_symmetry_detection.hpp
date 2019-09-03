/*
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

#ifndef SR_OBJECT_SYMMETRY_ROTATIONAL_SYMMETRY_DETECTION_HPP
#define SR_OBJECT_SYMMETRY_ROTATIONAL_SYMMETRY_DETECTION_HPP

// Symmetry
#include <sr_object_symmetry/rotational_symmetry_detection.h>
#include <sr_object_symmetry/rotational_symmetry_detection_core.hpp>
#include <sr_object_symmetry/rotational_symmetry_scoring.hpp>

#include <vector>

// Utilities
#include <utilities/geometry/geometry.hpp>
#include <utilities/graph/graph_algorithms.hpp>
#include <utilities/pointcloud/pointcloud.hpp>

////////////////////////////////////////////////////////////////////////////////
inline void sym::mergeDuplicateRotSymmetries(const std::vector<sym::RotationalSymmetry>& symmetries,
                                             const Eigen::Vector3f& symmetry_reference_points,
                                             const std::vector<int>& indices, std::vector<int>& merged_sym_ids,
                                             const float max_angle_diff, const float max_distance_diff)
{
  merged_sym_ids.clear();

  // Construct a graph where vertices represent object segments and edges
  // indicate segments that are similar
  utl::Graph symmetryAdjacency(indices.size());

  for (size_t srcIdIt = 0; srcIdIt < indices.size(); srcIdIt++)
  {
    int srcId = indices[srcIdIt];
    sym::RotationalSymmetry srcHypothesis = symmetries[srcId];
    Eigen::Vector3f srcReferencePoint = symmetry_reference_points;

    for (size_t tgtIdIt = srcIdIt + 1; tgtIdIt < indices.size(); tgtIdIt++)
    {
      int tgtId = indices[tgtIdIt];
      sym::RotationalSymmetry tgtHypothesis = symmetries[tgtId];
      Eigen::Vector3f tgtReferencePoint = symmetry_reference_points;

      Eigen::Vector3f referencePoint = (srcReferencePoint + tgtReferencePoint) / 2.0f;
      float angleDiff, distanceDiff;
      srcHypothesis.rotSymDifference(tgtHypothesis, referencePoint, angleDiff, distanceDiff);

      if (angleDiff < max_angle_diff && distanceDiff < max_distance_diff)
        symmetryAdjacency.addEdge(srcIdIt, tgtIdIt);
    }
  }
  // Find all connected components in the graph
  utl::Map hypothesisCCs;
  hypothesisCCs = utl::getConnectedComponents(symmetryAdjacency);

  // Select best hypothesis for each cluster
  merged_sym_ids.resize(hypothesisCCs.size());
  for (size_t clusterId = 0; clusterId < hypothesisCCs.size(); clusterId++)
  {
    float maxSupportSize = -1.0;
    int bestHypId = -1;
    for (size_t hypIt = 0; hypIt < hypothesisCCs[clusterId].size(); hypIt++)
    {
      int hypId = indices[hypothesisCCs[clusterId][hypIt]];
      bestHypId = hypId;
      if (bestHypId == -1)
        std::cout << "[sym::mergeDuplicateRotSymmetries] Could not select best hypothesis. Probably going to crash now."
                  << std::endl;
      merged_sym_ids[clusterId] = bestHypId;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
inline void sym::mergeDuplicateRotSymmetries(const std::vector<sym::RotationalSymmetry>& symmetries,
                                             const Eigen::Vector3f& symmetry_reference_points,
                                             std::vector<int>& merged_sym_ids, const float max_normal_angle_diff,
                                             const float max_distance_diff)
{
  // Create fake indices
  std::vector<int> indices(symmetries.size());
  for (size_t symId = 0; symId < symmetries.size(); symId++)
    indices[symId] = symId;

  // Merge symmetries
  mergeDuplicateRotSymmetries(symmetries, symmetry_reference_points, indices, merged_sym_ids, max_normal_angle_diff,
                              max_distance_diff);
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::RotationalSymmetryDetection<PointT>::RotationalSymmetryDetection()
  : params_(), cloud_(new pcl::PointCloud<PointT>), cloud_no_boundary_(new pcl::PointCloud<PointT>)
{
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::RotationalSymmetryDetection<PointT>::RotationalSymmetryDetection(const sym::RotSymDetectParams& params)
  : params_(params), cloud_(new pcl::PointCloud<PointT>), cloud_no_boundary_(new pcl::PointCloud<PointT>)
{
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::RotationalSymmetryDetection<PointT>::~RotationalSymmetryDetection()
{
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::RotationalSymmetryDetection<PointT>::setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  cloud_ = cloud;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void sym::RotationalSymmetryDetection<PointT>::setInputSymmetries(
    const std::vector<sym::RotationalSymmetry>& symmetries_initial)
{
  symmetries_initial_ = symmetries_initial;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void sym::RotationalSymmetryDetection<PointT>::setParameters(const RotSymDetectParams& params)
{
  params_ = params;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline bool sym::RotationalSymmetryDetection<PointT>::detect()
{
  //--------------------------------------------------------------------------
  // Entry checks

  if (cloud_->size() == 0)
  {
    std::cout << "[sym::RotationalSymmetryDetection::detect] input cloud is empty." << std::endl;
    return false;
  }

  symmetries_refined_.clear();
  symmetry_scores_.clear();
  perpendicular_scores_.clear();
  coverage_scores_.clear();
  point_symmetry_scores_.clear();
  point_perpendicular_scores_.clear();
  symmetry_filtered_ids_.clear();
  symmetry_merged_ids_.clear();

  // Remove boundary points from the pointcloud

  std::vector<int> cloudBoundaryPointIds, cloudNonBoundaryPointIds;
  utl::getCloudBoundary<PointT>(cloud_, 0.01f, cloudBoundaryPointIds, cloudNonBoundaryPointIds);

  std::vector<int> allPointIds(cloud_->size());
  for (size_t pointId = 0; pointId < cloud_->size(); pointId++)
    allPointIds[pointId] = pointId;
  cloud_no_boundary_point_ids_ = utl::vectorDifference(allPointIds, cloudBoundaryPointIds);
  pcl::copyPointCloud<PointT>(*cloud_, cloud_no_boundary_point_ids_, *cloud_no_boundary_);

  // Get the initial symmetries

  if (symmetries_initial_.size() == 0)
  {
    if (!sym::getInitialRotSymmetries<PointT>(cloud_, symmetries_initial_, cloud_mean_))
      return false;
  }
  else
  {
    Eigen::Vector4f cloudMeanTMP;
    pcl::compute3DCentroid<PointT>(*cloud_, cloudMeanTMP);
    cloud_mean_ = cloudMeanTMP.head(3);
  }

  // Refine initial symmetries

  symmetries_refined_.resize(symmetries_initial_.size());
  symmetry_scores_.resize(symmetries_initial_.size());
  perpendicular_scores_.resize(symmetries_initial_.size());
  coverage_scores_.resize(symmetries_initial_.size());
  point_symmetry_scores_.resize(symmetries_initial_.size());
  point_perpendicular_scores_.resize(symmetries_initial_.size());

  std::vector<bool> filteredSymTableTMP(symmetries_refined_.size(), false);
  //     # pragma omp parallel for
  for (size_t symId = 0; symId < symmetries_refined_.size(); symId++)
  {
    // Create optimization object
    sym::RotSymRefineFunctorDiff<PointT> functor;
    functor.cloud_ = cloud_no_boundary_;
    functor.max_fit_angle_ = params_.ref_max_fit_angle;
    Eigen::LevenbergMarquardt<sym::RotSymRefineFunctorDiff<PointT>, float> lm(functor);
    lm.parameters.ftol = 1e-12;
    lm.parameters.maxfev = 800;
    // Refine symmetry
    Eigen::VectorXf x(6);
    x.head(3) = symmetries_initial_[symId].getOrigin();
    x.tail(3) = symmetries_initial_[symId].getDirection();
    lm.minimize(x);
    symmetries_refined_[symId] = sym::RotationalSymmetry(x.head(3), x.tail(3));
    symmetries_refined_[symId].setOriginProjected(cloud_mean_);
    // Score symmetry
    symmetry_scores_[symId] = sym::rotSymCloudSymmetryScore<PointT>(
        *cloud_no_boundary_, symmetries_refined_[symId], point_symmetry_scores_[symId], params_.min_normal_fit_angle,
        params_.max_normal_fit_angle);
    perpendicular_scores_[symId] = sym::rotSymCloudPerpendicularScores<PointT>(
        *cloud_no_boundary_, symmetries_refined_[symId], point_perpendicular_scores_[symId]);
    coverage_scores_[symId] = sym::rotSymCloudCoverageAngle<PointT>(*cloud_, symmetries_refined_[symId]);
    coverage_scores_[symId] /= (M_PI * 2);
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void sym::RotationalSymmetryDetection<PointT>::filter()
{
  symmetry_filtered_ids_.clear();
  for (size_t symId = 0; symId < symmetries_refined_.size(); symId++)
  {
    // Check if it's a good symmetry
    if (symmetry_scores_[symId] < params_.max_symmetry_score &&
        perpendicular_scores_[symId] < params_.max_perpendicular_score &&
        coverage_scores_[symId] > params_.min_coverage_score)
    {
      symmetry_filtered_ids_.push_back(symId);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void sym::RotationalSymmetryDetection<PointT>::getSymmetries(std::vector<sym::RotationalSymmetry>& symmetries,
                                                                    std::vector<int>& symmetry_filtered_ids)
{
  symmetries = symmetries_refined_;
  symmetry_filtered_ids = symmetry_filtered_ids_;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void sym::RotationalSymmetryDetection<PointT>::getScores(std::vector<float>& symmetry_scores,
                                                                std::vector<float>& perpendicular_scores,
                                                                std::vector<float>& coverage_scores)
{
  symmetry_scores = symmetry_scores_;
  perpendicular_scores = perpendicular_scores_;
  coverage_scores = coverage_scores_;
}

#endif  // SR_OBJECT_SYMMETRY_ROTATIONAL_SYMMETRY_DETECTION_HPP
