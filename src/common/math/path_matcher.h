/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <utility>
#include <vector>

//#include "modules/common/proto/pnc_point.pb.h"
#include "../../../src/proto/protocpp/pnc_point.pb.h"
//#include "../../../modules/common/build/pnc_point.pb.h"
//#include "../../../modules/build/pnc_point.pb.h"

namespace apollo {
namespace common {
namespace math {


  //2.ReferenceLine上匹配点的检索
  //基于离散化处理后的参考线，根据规划起点的定位数据即可找到参考线上对应的最近点，
  //但是这个点并不能称之为匹配点，因为很可能二者连线并不是参考线的法线方向，
  //即，A点与最近点B的连线并不垂直于参考线，当参考线的点不够密集时，是存在较大误差的

  //通过该最近点的前后两个点的连线，求得规划起点的位置在该连线上的垂点，认为是匹配点
  //计算过程很简单，通过向量运算，并得到匹配点C的历程s信息

class PathMatcher {
 public:
  PathMatcher() = delete;

  static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                               const double x, const double y);

  static std::pair<double, double> GetPathFrenetCoordinate(
      const std::vector<PathPoint>& reference_line, const double x,
      const double y);

  static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                               const double s);

 private:
  static PathPoint FindProjectionPoint(const PathPoint& p0, const PathPoint& p1,
                                       const double x, const double y);
};

}  // namespace math
}  // namespace common
}  // namespace apollo
