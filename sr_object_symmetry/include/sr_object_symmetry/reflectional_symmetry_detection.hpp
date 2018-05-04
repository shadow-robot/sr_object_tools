// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.
/*
 * Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
 * Unauthorized copying of the content in this file, via any medium is strictly prohibited.
 *
*/
#ifndef REFLECTIONAL_SYMMETRY_DETECTION_HPP
#define REFLECTIONAL_SYMMETRY_DETECTION_HPP

// OpenMP includes
#include <omp.h>

// Utilities
#include <utilities/graph/bron_kerbosch.hpp>

// Symmetry
#include <sr_object_symmetry/reflectional_symmetry_detection.h>
#include <sr_object_symmetry/reflectional_symmetry_detection_core.hpp>
#include <sr_object_symmetry/reflectional_symmetry_scoring.hpp>

////////////////////////////////////////////////////////////////////////////////
inline void sym::mergeDuplicateReflSymmetries(const std::vector<sym::ReflectionalSymmetry>& symmetries,
                                              const Eigen::Vector3f& symmetry_reference_points,
                                              const std::vector<int>& indices, std::vector<int>& merged_sym_ids,
                                              const float max_normal_angle_diff, const float max_distance_diff,
                                              const float max_reference_point_distance)
{
  merged_sym_ids.clear();

  //----------------------------------------------------------------------------
  // Construct a graph where vertices represent symmetries and edges indicate
  // symmetries that are similar
  utl::Graph symmetryAdjacency(indices.size());

  for (size_t srcIdIt = 0; srcIdIt < indices.size(); srcIdIt++)
  {
    int srcId = indices[srcIdIt];
    sym::ReflectionalSymmetry srcHypothesis = symmetries[srcId];
    Eigen::Vector3f srcReferencePoint = symmetry_reference_points;

    for (size_t tgtIdIt = srcIdIt + 1; tgtIdIt < indices.size(); tgtIdIt++)
    {
      int tgtId = indices[tgtIdIt];
      sym::ReflectionalSymmetry tgtHypothesis = symmetries[tgtId];
      Eigen::Vector3f tgtReferencePoint = symmetry_reference_points;
      // If segments are too far apart, don't merged their symmetries
      if (max_reference_point_distance > 0.0f &&
          utl::pointToPointDistance<float>(srcReferencePoint, tgtReferencePoint) > max_reference_point_distance)
        continue;
      Eigen::Vector3f referencePoint = (srcReferencePoint + tgtReferencePoint) / 2.0f;
      float angleDiff, distanceDiff;
      srcHypothesis.reflSymDifference(tgtHypothesis, referencePoint, angleDiff, distanceDiff);
      if (angleDiff < max_normal_angle_diff && distanceDiff < max_distance_diff)
        symmetryAdjacency.addEdge(srcIdIt, tgtIdIt);
    }
  }

  //----------------------------------------------------------------------------
  // Find the maximal cliques in the graph
  std::list<std::list<int>> hypothesisCliques;
  utl::bronKerbosch(symmetryAdjacency, hypothesisCliques, 1);

  // Find hypothesis clusters that will be merged
  utl::Map hypothesisClusters;
  while (hypothesisCliques.size() > 0)
  {
    // Find the largest clique
    std::list<std::list<int>>::iterator largestCliqueIt;
    int maxCliqueSize = 0;
    for (std::list<std::list<int>>::iterator it = hypothesisCliques.begin(); it != hypothesisCliques.end(); it++)
    {
      if (it->size() > maxCliqueSize)
      {
        largestCliqueIt = it;
        maxCliqueSize = it->size();
      }
    }

    // Add hypotheses from the largest clique to the list of hypothesis clusters
    hypothesisClusters.push_back(std::vector<int>{ std::begin(*largestCliqueIt), std::end(*largestCliqueIt) });
    // Remove hyptheses belonging to the largest clique from existing cliques
    hypothesisCliques.erase(largestCliqueIt);
    for (std::list<std::list<int>>::iterator it = hypothesisCliques.begin(); it != hypothesisCliques.end();)
    {
      for (std::vector<int>::const_iterator elIt = hypothesisClusters.back().begin();
           elIt != hypothesisClusters.back().end(); elIt++)
        it->remove(*elIt);
      if (it->size() == 0)
        it = hypothesisCliques.erase(it);
      else
        it++;
    }
  }

  //----------------------------------------------------------------------------
  // Select best hypothesis for each cluster
  merged_sym_ids.resize(hypothesisClusters.size());

  for (size_t clusterId = 0; clusterId < hypothesisClusters.size(); clusterId++)
  {
    float minScore = std::numeric_limits<float>::max();
    int bestHypId = -1;
    for (size_t hypIt = 0; hypIt < hypothesisClusters[clusterId].size(); hypIt++)
    {
      int hypId = indices[hypothesisClusters[clusterId][hypIt]];
      // if (occlusion_scores[hypId] < minScore)
      //{
      //  minScore = occlusion_scores[hypId];
      bestHypId = hypId;
      //}

      if (bestHypId == -1)
        std::cout << "[sym::mergeDuplicateReflSymmetries] Could not select best hypothesis. Probably going to crash "
                     "now."
                  << std::endl;

      merged_sym_ids[clusterId] = bestHypId;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
inline void sym::mergeDuplicateReflSymmetries(const std::vector<sym::ReflectionalSymmetry>& symmetries,
                                              const Eigen::Vector3f& symmetry_reference_points,
                                              std::vector<int>& merged_sym_ids, const float max_normal_angle_diff,
                                              const float max_distance_diff, const float max_reference_point_distance)
{
  std::vector<int> indices(symmetries.size());
  for (size_t symId = 0; symId < symmetries.size(); symId++)
    indices[symId] = symId;

  sym::mergeDuplicateReflSymmetries(symmetries, symmetry_reference_points, indices, merged_sym_ids,
                                    max_normal_angle_diff, max_distance_diff, max_reference_point_distance);
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::ReflectionalSymmetryDetection<PointT>::ReflectionalSymmetryDetection()
  : params_(), cloud_ds_(new pcl::PointCloud<PointT>)
{
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::ReflectionalSymmetryDetection<PointT>::ReflectionalSymmetryDetection(const sym::ReflSymDetectParams& params)
  : params_(params), cloud_ds_(new pcl::PointCloud<PointT>)
{
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::ReflectionalSymmetryDetection<PointT>::~ReflectionalSymmetryDetection()
{
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::ReflectionalSymmetryDetection<PointT>::setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  cloud_ = cloud;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void sym::ReflectionalSymmetryDetection<PointT>::setInputSymmetries(
    const std::vector<sym::ReflectionalSymmetry>& symmetries_initial)
{
  symmetries_initial_ = symmetries_initial;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void sym::ReflectionalSymmetryDetection<PointT>::setParameters(const ReflSymDetectParams& params)
{
  params_ = params;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline bool sym::ReflectionalSymmetryDetection<PointT>::detect()
{
  //----------------------------------------------------------------------------
  // Initialize computation

  if (!cloud_ || cloud_->size() == 0)
  {
    std::cout << "[sym::ReflectionalSymmetryDetection::detect] input cloud is not set or it is empty." << std::endl;
    return false;
  }

  cloud_mean_ = Eigen::Vector3f::Zero();
  cloud_ds_->clear();
  symmetries_refined_.clear();
  correspondences_.clear();
  point_symmetry_scores_.clear();
  cloud_inlier_scores_.clear();
  corresp_inlier_scores_.clear();
  symmetry_filtered_ids_.clear();
  symmetry_merged_ids_.clear();

  //----------------------------------------------------------------------------
  // Downsample input pointcloud and create a search tree

  // Downsample the cloud
  if (params_.voxel_size <= 0.0f)
    *cloud_ds_ = *cloud_;
  else
  {
    utl::Downsample<PointT> dc;
    dc.setInputCloud(cloud_);
    dc.setDownsampleMethod(utl::Downsample<PointT>::AVERAGE);
    dc.setLeafSize(params_.voxel_size);
    dc.filter(*cloud_ds_);
  }

  // Create a search tree for the input cloud
  pcl::search::KdTree<PointT> cloud_search_tree;
  cloud_search_tree.setInputCloud(cloud_);

  //----------------------------------------------------------------------------
  // Get pointcloud boundary
  std::vector<int> cloudBoundaryPointIds, cloudNonBoundaryPointIds;
  utl::getCloudBoundary<PointT>(cloud_, 0.01f, cloudBoundaryPointIds, cloudNonBoundaryPointIds);

  //----------------------------------------------------------------------------
  // Get initial symmetries
  if (symmetries_initial_.size() == 0)
  {
    if (!sym::getInitialReflSymmetries<PointT>(cloud_, symmetries_initial_, cloud_mean_, params_.num_angle_divisions,
                                               params_.flatness_threshold))
      return false;
  }
  else
  {
    Eigen::Vector4f cloudMeanTMP;
    pcl::compute3DCentroid<PointT>(*cloud_, cloudMeanTMP);
    cloud_mean_ = cloudMeanTMP.head(3);
  }

  //----------------------------------------------------------------------------
  // Refine initial symmetries
  std::cout << "Found Initial symmetries:" << symmetries_initial_.size() << std::endl;
  // Pre-filtering
  if (params_.rot_symmetries.size() > 0)  // rotational symmetriy exist so keep perpendicular ones only
  {
    // pre-filtered symmetries
    std::vector<sym::ReflectionalSymmetry> pre_symmetries;
    for (size_t symId = 0; symId < symmetries_initial_.size(); symId++)
    {
      for (size_t rotId = 0; rotId < params_.rot_symmetries.size(); rotId++)
      {
        Eigen::Vector3f rot_symmetry;
        rot_symmetry = params_.rot_symmetries[rotId];  // only one rotational symmetry
        if (rot_symmetry.dot(symmetries_initial_[symId].getNormal()) < -0.99)
          pre_symmetries.push_back(symmetries_initial_[symId]);
      }
    }
    if (pre_symmetries.size() > 0)  // found perpendicular so resize symmetries
    {
      symmetries_initial_.resize(pre_symmetries.size());
      symmetries_initial_ = pre_symmetries;
      std::cout << "Pre-filtered symmetries:" << symmetries_initial_.size() << std::endl;
    }
  }
  // These vectors are required to enable paralllizing symmetry detection loop
  std::vector<sym::ReflectionalSymmetry> symmetriesTMP(symmetries_initial_.size());
  std::vector<float> cloudInlierScoresTMP(symmetries_initial_.size());
  std::vector<float> correspInlierScoresTMP(symmetries_initial_.size());
  std::vector<std::vector<float>> pointSymmetryScoresTMP(symmetries_initial_.size());
  std::vector<bool> validSymTableTMP(symmetries_initial_.size(), false);
  std::vector<bool> filteredSymTableTMP(symmetries_initial_.size(), false);
  std::vector<pcl::Correspondences> symmetryCorrespTMP(symmetries_initial_.size());

// NOTE: it turns out that putting parralel for statement here
// runs more that twice faster than parallelizing the loop that calls
// reflectional symmetry detection
#pragma omp parallel for
  for (size_t symId = 0; symId < symmetries_initial_.size(); symId++)
  {
    sym::ReflectionalSymmetry curSymmetry;
    pcl::Correspondences curCorrespondences;

    if (!sym::refineReflSymPosition<PointT>(cloud_, cloud_ds_, symmetries_initial_[symId], curSymmetry,
                                            curCorrespondences))
      continue;

    // Refine symmetry global
    if (!sym::refineReflSymGlobal<PointT>(cloud_search_tree, cloud_ds_, cloud_mean_, curSymmetry, curSymmetry,
                                          curCorrespondences, params_.refine_iterations))
      continue;

    // Score symmetry
    std::vector<float> curPointSymmetryScores;
    float curCloudInlierScore, curCorrespInlierScore;

    sym::reflSymPointSymmetryScores<PointT>(cloud_search_tree, *cloud_ds_, std::vector<int>(), std::vector<int>(),
                                            curSymmetry, curCorrespondences, curPointSymmetryScores,
                                            params_.max_correspondence_reflected_distance,
                                            params_.min_inlier_normal_angle, params_.max_inlier_normal_angle);

    float inlierScoreSum = 0;
    for (size_t crspId = 0; crspId < curCorrespondences.size(); crspId++)
      inlierScoreSum += (1.0f - curPointSymmetryScores[crspId]);

    curCloudInlierScore = inlierScoreSum / static_cast<float>(cloud_ds_->size());
    curCorrespInlierScore = inlierScoreSum / static_cast<float>(curCorrespondences.size());

    // Populate temporary result variables
    symmetriesTMP[symId] = curSymmetry;
    cloudInlierScoresTMP[symId] = curCloudInlierScore;
    correspInlierScoresTMP[symId] = curCorrespInlierScore;
    pointSymmetryScoresTMP[symId] = curPointSymmetryScores;
    validSymTableTMP[symId] = true;
    symmetryCorrespTMP[symId] = curCorrespondences;
  }

  // Extract valid symmetries
  for (size_t symIdIt = 0; symIdIt < validSymTableTMP.size(); symIdIt++)
  {
    if (!validSymTableTMP[symIdIt])
      continue;

    symmetries_refined_.push_back(symmetriesTMP[symIdIt]);
    cloud_inlier_scores_.push_back(cloudInlierScoresTMP[symIdIt]);
    corresp_inlier_scores_.push_back(correspInlierScoresTMP[symIdIt]);
    point_symmetry_scores_.push_back(pointSymmetryScoresTMP[symIdIt]);
    correspondences_.push_back(symmetryCorrespTMP[symIdIt]);
  }
  std::cout << "Refined symmetries:" << symmetries_refined_.size() << std::endl;
  return (symmetries_refined_.size() > 0);
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void sym::ReflectionalSymmetryDetection<PointT>::filter()
{
  symmetry_filtered_ids_.clear();

  for (size_t symId = 0; symId < symmetries_refined_.size(); symId++)
  {
    // Check if it's a good symmetry
    if (cloud_inlier_scores_[symId] > params_.min_cloud_inlier_score &&
        corresp_inlier_scores_[symId] > params_.min_corresp_inlier_score)
    {
      symmetry_filtered_ids_.push_back(symId);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void sym::ReflectionalSymmetryDetection<PointT>::getSymmetries(
    std::vector<sym::ReflectionalSymmetry>& symmetries, std::vector<int>& symmetry_filtered_ids)
{
  symmetries = symmetries_refined_;
  symmetry_filtered_ids = symmetry_filtered_ids_;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void sym::ReflectionalSymmetryDetection<PointT>::getScores(std::vector<float>& cloud_inlier_scores,
                                                                  std::vector<float>& corresp_inlier_scores)
{
  cloud_inlier_scores = cloud_inlier_scores_;
  corresp_inlier_scores = corresp_inlier_scores_;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::ReflectionalSymmetryDetection<PointT>::getPointScores(typename pcl::PointCloud<PointT>::Ptr& cloud_ds,
                                                           std::vector<pcl::Correspondences>& correspondences,
                                                           std::vector<std::vector<float>>& point_symmetry_scores)

{
  cloud_ds.reset(new pcl::PointCloud<PointT>);
  *cloud_ds = *cloud_ds_;
  correspondences = correspondences_;
  point_symmetry_scores = point_symmetry_scores_;
}

#endif  // REFLECTIONAL_SYMMETRY_DETECTION_HPP
