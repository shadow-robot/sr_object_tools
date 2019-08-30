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

#ifndef UTILITIES_EIGEN_H
#define UTILITIES_EIGEN_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

namespace utl
{
/* \brief Write matrix to a file in binary mode.
    *  \param filename output file name
    *  \param matrix matrix
    *  \note http://stackoverflow.com/questions/25389480/how-to-write-read-an-eigen-matrix-from-binary-file
    */
template <class Matrix>
inline bool writeBinary(const std::string filename, const Matrix& matrix)
{
  std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
  if (out.is_open())
  {
    typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
    out.write(reinterpret_cast<char*>(&rows), sizeof(typename Matrix::Index));
    out.write(reinterpret_cast<char*>(&cols), sizeof(typename Matrix::Index));
    out.write(reinterpret_cast<char*>(matrix.data()), rows * cols * sizeof(typename Matrix::Scalar));
    out.close();
    return true;
  }
  else
  {
    std::cout << "[utl::eigen::writeBinary] Colud not open file '" << filename << "' for writing\n";
    return false;
  }
}

/* \brief Read a matrix from a binary file.
    *  \param filename input file name
    *  \param matrix matrix
    *  \note http://stackoverflow.com/questions/25389480/how-to-write-read-an-eigen-matrix-from-binary-file
    */
template <class Matrix>
inline bool readBinary(const std::string filename, Matrix& matrix)
{
  std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);
  if (!in.is_open())
  {
    std::cout << "[utl::eigen::readBinary] Colud not open file '" << filename << "' for reading." << std::endl;
    return false;
  }

  typename Matrix::Index rows = 0, cols = 0;
  in.read(reinterpret_cast<char*>(&rows), sizeof(typename Matrix::Index));
  in.read(reinterpret_cast<char*>(&cols), sizeof(typename Matrix::Index));
  matrix.resize(rows, cols);
  in.read(reinterpret_cast<char*>(matrix.data()), rows * cols * sizeof(typename Matrix::Scalar));
  in.close();
  return true;
}

/* \brief Write matrix to a file in ASCII mode.
    *  \param filename output file name
    *  \param matrix matrix
    *  \return TRUE if file written successfully
    */
template <class Matrix>
inline bool writeASCII(const std::string filename, const Matrix& matrix)
{
  std::ofstream out(filename.c_str(), std::ios::out);
  if (out.is_open())
  {
    out << matrix << "\n";
    out.close();
    return true;
  }
  else
  {
    std::cout << "[utl::eigen::writeASCII] Colud not open file '" << filename << "' for writing\n";
    return false;
  }
}

/* \brief Read a matrix from an ASCII file.
    *  \param filename input file name
    *  \param matrix matrix
    *  \return TRUE if file read successfully
    *  \note Adapted from http://perso.ensta-paristech.fr/~stulp/dmpbbo/EigenFileIO_8tpp_source.html
    */
template <class Matrix>
inline bool readASCII(const std::string filename, Matrix& matrix)
{
  std::ifstream in(filename.c_str(), std::ios::in);
  if (!in.is_open())
  {
    std::cout << "[utl::eigen::readASCII] Colud not open file '" << filename << "' for reading." << std::endl;
    return false;
  }

  // Read file contents into a vector
  std::string line;
  typename Matrix::Scalar d;

  std::vector<typename Matrix::Scalar> v;
  int n_rows = 0;
  while (getline(in, line))
  {
    ++n_rows;
    std::stringstream input_line(line);
    while (!input_line.eof())
    {
      input_line >> d;
      v.push_back(d);
    }
  }
  in.close();

  // Construct matrix
  int n_cols = v.size() / n_rows;
  matrix = Eigen::Matrix<typename Matrix::Scalar, Eigen::Dynamic, Eigen::Dynamic>(n_rows, n_cols);

  for (int i = 0; i < n_rows; i++)
    for (int j = 0; j < n_cols; j++)
      matrix(i, j) = v[i * n_cols + j];

  return true;
}
}  // namespace utl

#endif  // UTILITIES_EIGEN_H
