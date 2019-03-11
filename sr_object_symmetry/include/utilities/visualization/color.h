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

#ifndef UTILITIES_VISUALIZATION_COLOR_H
#define UTILITIES_VISUALIZATION_COLOR_H

#include <vector>
#include <algorithm>

namespace utl
{
struct Color
{
  double r;
  double g;
  double b;

  Color() : r(-1), g(-1), b(-1)
  {
  }

  Color(const double r, const double g, const double b) : r(r), g(g), b(b)
  {
  }

  std::vector<double> toStdVec()
  {
    std::vector<double> c(3);
    c[0] = r;
    c[1] = g;
    c[2] = b;
    return c;
  }

  bool operator==(const Color& rhs) const
  {
    return (r == rhs.r && b == rhs.b && g == rhs.b);
  }

  bool operator!=(const Color& rhs) const
  {
    return (r != rhs.r && b != rhs.b && g != rhs.b);
  }

  template <typename Scalar>
  Color operator*(const Scalar x)
  {
    Color result;

    result.r = r * static_cast<double>(x);
    result.g = g * static_cast<double>(x);
    result.b = b * static_cast<double>(x);
    result.r = std::max(0.0, std::min(1.0, r));
    result.g = std::max(0.0, std::min(1.0, g));
    result.b = std::max(0.0, std::min(1.0, b));

    return result;
  }
};

typedef std::vector<Color> Colors;
}  // namespace utl

#endif  // UTILITIES_VISUALIZATION_COLOR_H
