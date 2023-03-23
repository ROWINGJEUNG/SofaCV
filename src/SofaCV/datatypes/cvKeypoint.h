#ifndef SOFACV_CVKEYPOINT_H
#define SOFACV_CVKEYPOINT_H

#include "SofaCV/SofaCVPlugin.h"

#include <sofa/defaulttype/DataTypeInfo.h>

#include <opencv2/core.hpp>

#include <iostream>

namespace sofacv

{
/**
 * @brief The Keypoint class, Needed to override the stream operators for SOFA
 */
class SOFA_SOFACV_API cvKeypoint : public cv::KeyPoint
{
 public:
  cvKeypoint(const cv::KeyPoint& kp);
  cvKeypoint() : cv::KeyPoint() {}
  cvKeypoint(cv::Point2f _pt, float _size, float _angle = -1,
             float _response = 0, int _octave = 0, int _class_id = -1);
  cvKeypoint(float x, float y, float _size, float _angle = -1,
             float _response = 0, int _octave = 0, int _class_id = -1);

  inline friend std::istream& operator>>(std::istream& in, cvKeypoint& s)
  {
    in >> s.pt.x >> s.pt.y >> s.size >> s.angle >> s.response >> s.octave >>
        s.class_id;
    return in;
  }

  inline friend std::ostream& operator<<(std::ostream& out, const cvKeypoint& s)
  {
    out << s.pt.x << ' ' << s.pt.y << ' ' << s.size << ' ' << s.angle << ' '
        << s.response << ' ' << s.octave << ' ' << s.class_id << ' ';
    return out;
  }
};

}  // namespace sofacv

namespace sofa
{
namespace defaulttype
{
/**
 *  \brief Implementation of SOFA's DataType interface to pass cv::KeyPoint data
 * structures as sofa::Data
 */
template <>
struct DataTypeName<sofacv::cvKeypoint>
{
  static const char* name() { return "cvKeypoint"; }
};

}  // namespace defaulttype
}  // namespace sofa

#endif  // SOFACV_CVKEYPOINT_H
