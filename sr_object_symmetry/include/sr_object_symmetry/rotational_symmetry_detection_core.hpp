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

#ifndef SR_OBJECT_SYMMETRY_ROTATIONAL_SYMMETRY_DETECTION_CORE_H
#define SR_OBJECT_SYMMETRY_ROTATIONAL_SYMMETRY_DETECTION_CORE_H

#include <algorithm>
#include <vector>

// Symmetry
#include <sr_object_symmetry/rotational_symmetry.hpp>
#include <sr_object_symmetry/refinement_base_functor.hpp>

namespace sym
{
//----------------------------------------------------------------------------
// Initial symmetry generation
//----------------------------------------------------------------------------

/** \brief Get the initial symmetries used for rotational symmetry
   * detection of a pointcloud.
   *  \param[in]  cloud               input cloud
   *  \param[out] symmetries          rotational symmetries
   *  \param[out] cloud_mean          mean of the pointcloud
   *  \return FALSE if input pointcloud has less than three points (can't run PCA)
   */
template <typename PointT>
inline bool getInitialRotSymmetries(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                                    std::vector<sym::RotationalSymmetry>& symmetries, Eigen::Vector3f& cloud_mean)
{
  symmetries.clear();

  // Check that input cloud has sufficient number of points
  if (cloud->size() < 3)
  {
    std::cout << "[sym::getInitialRotSymmetries] input cloud has less than three points. Aborting..." << std::endl;
    return false;
  }

  // Get initial symmetries from the major axes of the input cloud
  pcl::PCA<PointT> pcaSolver;
  pcaSolver.setInputCloud(cloud);
  cloud_mean = pcaSolver.getMean().head(3);

  symmetries.resize(3);
  symmetries[0] = sym::RotationalSymmetry(cloud_mean, pcaSolver.getEigenVectors().col(0));
  symmetries[1] = sym::RotationalSymmetry(cloud_mean, pcaSolver.getEigenVectors().col(1));
  symmetries[2] = sym::RotationalSymmetry(cloud_mean, pcaSolver.getEigenVectors().col(2));

  return true;
}

//----------------------------------------------------------------------------
// Symmetry refinement
//----------------------------------------------------------------------------

/** \brief Given 3D pointcloud with normals and an initial 3D rotational
   * symmetry axis refine the symmetry axis such that it minimizes the error
   * of fit between the symmetry and the points.
   * Symmetry axis is represented as a 6D vector (point on the symmetry axis and
   * a direction vector.
   * \note the final refined symmetry vector may have a non-unit normal.
   */
template <typename PointT>
struct RotSymRefineFunctor : BaseFunctor<float>
{
  /** \brief Empty constructor */
  RotSymRefineFunctor() : max_fit_angle_(1.0f) {}

  /** \brief Compute fitness for each input point.
     *  \param[in]  x symmetry axis
     *  \param[out] fvec error vector
     */
  int operator()(const Eigen::VectorXf& x, Eigen::VectorXf& fvec) const
  {
    // Get current rotational symmetry
    RotationalSymmetry symmetry(x.head(3), x.tail(3));

    // Compute fitness
    for (size_t i = 0; i < this->cloud_->size(); i++)
    {
      float angle = getRotSymFitError(this->cloud_->points[i].getVector3fMap(),
                                      this->cloud_->points[i].getNormalVector3fMap(), symmetry);

      fvec(i) = std::min(angle, max_fit_angle_);
    }
    return 0;
  }

  /** \brief Input cloud. */
  typename pcl::PointCloud<PointT>::ConstPtr cloud_;

  /** \brief Maximum error of fit between a symmetry and a point. */
  float max_fit_angle_;

  /** \brief Dimensionality of the optimization parameter vector. */
  int inputs() const
  {
    return 6;
  }

  /** \brief Number of points. */
  int values() const
  {
    return this->cloud_->size();
  }
};

template <typename PointT>
struct RotSymRefineFunctorDiff : Eigen::NumericalDiff<RotSymRefineFunctor<PointT>>
{
};
}  // namespace sym

#endif  // SR_OBJECT_SYMMETRY_ROTATIONAL_SYMMETRY_DETECTION_CORE_H
