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

/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2010-2011, Willow Garage, Inc.
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder(s) nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef UTILITIES_POINTCLOUD_MESH_TO_CLOUD_H
#define UTILITIES_POINTCLOUD_MESH_TO_CLOUD_H

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <string>

inline double uniform_deviate(int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2,
                                float c3, float r1, float r2, Eigen::Vector3f& p)
{
  float r1sqr = std::sqrt(r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
}

inline void randPSurface(vtkPolyData* polydata, std::vector<double>* cumulativeAreas, double totalArea,
                         Eigen::Vector3f& p, Eigen::Vector3f& n, Eigen::Vector3f& c)
{
  float r = static_cast<float>(uniform_deviate(rand()) * totalArea);  // NOLINT

  std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
  vtkIdType el = vtkIdType(low - cumulativeAreas->begin());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType* ptIds = NULL;
  polydata->GetCellPoints(el, npts, ptIds);
  polydata->GetPoint(ptIds[0], A);
  polydata->GetPoint(ptIds[1], B);
  polydata->GetPoint(ptIds[2], C);
  // OBJ: Vertices are stored in a counter-clockwise order by default
  Eigen::Vector3f v1 = Eigen::Vector3f(A[0], A[1], A[2]) - Eigen::Vector3f(C[0], C[1], C[2]);
  Eigen::Vector3f v2 = Eigen::Vector3f(B[0], B[1], B[2]) - Eigen::Vector3f(C[0], C[1], C[2]);
  n = v1.cross(v2);
  n.normalize();
  float r1 = static_cast<float>(uniform_deviate(rand()));  // NOLINT
  float r2 = static_cast<float>(uniform_deviate(rand()));  // NOLINT
  randomPointTriangle(static_cast<float>(A[0]), static_cast<float>(A[1]), static_cast<float>(A[2]),
                      static_cast<float>(B[0]), static_cast<float>(B[1]), static_cast<float>(B[2]),
                      static_cast<float>(C[0]), static_cast<float>(C[1]), static_cast<float>(C[2]), r1, r2, p);
}

void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples,
                      pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out)
{
  polydata->BuildCells();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);
  size_t i = 0;
  vtkIdType npts = 0, *ptIds = NULL;
  for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); i++)
  {
    polydata->GetPoint(ptIds[0], p1);
    polydata->GetPoint(ptIds[1], p2);
    polydata->GetPoint(ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
    cumulativeAreas[i] = totalArea;
  }

  cloud_out.points.resize(n_samples);
  cloud_out.width = static_cast<pcl::uint32_t>(n_samples);
  cloud_out.height = 1;

  for (i = 0; i < n_samples; i++)
  {
    Eigen::Vector3f p;
    Eigen::Vector3f n;
    Eigen::Vector3f c;
    randPSurface(polydata, &cumulativeAreas, totalArea, p, n, c);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
    cloud_out.points[i].normal_x = n[0];
    cloud_out.points[i].normal_y = n[1];
    cloud_out.points[i].normal_z = n[2];
  }
}

using namespace pcl;  // NOLINT
using namespace pcl::io;  // NOLINT

const float default_leaf_size = 0.01f;

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr convertPlyToCloud(std::string& fileName, int number_samples)
{
  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
  pcl::PolygonMesh mesh;
  if (fileName.find(".stl") != std::string::npos)
    pcl::io::loadPolygonFileSTL(fileName, mesh);
  else if (fileName.find(".ply") != std::string::npos)
    pcl::io::loadPolygonFilePLY(fileName, mesh);
  else if (fileName.find(".obj") != std::string::npos)
    pcl::io::loadPolygonFileOBJ(fileName, mesh);
  pcl::io::mesh2vtk(mesh, polydata1);
  // make sure that the polygons are triangles!
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
#if VTK_MAJOR_VERSION < 6
  triangleFilter->SetInput(polydata1);
#else
  triangleFilter->SetInputData(polydata1);
#endif
  triangleFilter->Update();

  vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
  triangleMapper->Update();
  polydata1 = triangleMapper->GetInput();
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  uniform_sampling(polydata1, number_samples, *cloud_1);
  // Voxelgrid
  VoxelGrid<PointXYZRGBNormal> grid_;
  grid_.setInputCloud(cloud_1);
  grid_.setLeafSize(default_leaf_size, default_leaf_size, default_leaf_size);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  grid_.filter(*voxel_cloud);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // Strip uninitialized colors from cloud:
  pcl::copyPointCloud(*voxel_cloud, *cloud_xyzn);
  return cloud_xyzn;
}

#endif  // UTILITIES_POINTCLOUD_MESH_TO_CLOUD_H
