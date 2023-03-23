#ifndef SOFACV_CVMATUTILS_H
#define SOFACV_CVMATUTILS_H

#include <sofa/defaulttype/Mat.h>
#include <sofa/type/vector.h>

#include <opencv2/core.hpp>

namespace sofacv
{
namespace image
{
/// (c.f. see http://www.stereo3d.com/formats.htm)
enum VideoMode
{
  MONO = 0,
  STEREO_INTERLEAVED = 1,
  STEREO_TOP_BOTTOM = 2,
  STEREO_SIDE_BY_SIDE = 3,
  STEREO_ROTATED_SIDE_BY_SIDE = 4
};

}  // namespace image

/**
 * \brief utility conversions from OpenCV vector / matrix data structures to
 * SOFA & vice versa
 */
namespace matrix
{
template <unsigned int L, unsigned int C, class T = double>
void cvMat2sofaMat(const cv::Mat_<T>& src, sofa::type::Mat<L, C, T>& dst)
{
  assert(src.cols == C && src.rows == L);
  std::memcpy(dst.ptr(), src.ptr(), src.total() * src.elemSize());
}
template <unsigned int L, unsigned int C, class T = double>
void cvMat2sofaMat(const cv::Mat& src, sofa::type::Mat<L, C, T>& dst)
{
  assert(src.cols == C && src.rows == L);
  std::memcpy(dst.ptr(), src.ptr(), src.total() * src.elemSize());
}

template <unsigned int L, unsigned int C, class T = double>
void sofaMat2cvMat(const sofa::type::Mat<L, C, T>& src, cv::Mat_<T>& dst)
{
  dst = cv::Mat_<T>(L, C);
  std::memcpy(dst.ptr(), src.ptr(), L * C * sizeof(T));
}
template <unsigned int L, unsigned int C, class T = double>
void sofaMat2cvMat(const sofa::type::Mat<L, C, T>& src, cv::Mat& dst)
{
  dst = cv::Mat_<T>(L, C);
  std::memcpy(dst.ptr(), src.ptr(), L * C * sizeof(T));
}

template <int N, class T>
void cvMat2sofaVector(const cv::Mat& src, sofa::type::Vec<N, T>& dst)
{
  assert(src.total() == N);
  std::memcpy(dst.ptr(), src.data, src.total() * src.elemSize());
}
template <int N, class T>
void cvMat2sofaVector(const cv::Mat_<T>& src, sofa::type::Vec<N, T>& dst)
{
  assert(src.total() == N);
  std::memcpy(dst.ptr(), src.data, src.total() * src.elemSize());
}
template <unsigned int N, class T>
void cvMat2sofaVector(const cv::Mat& src, sofa::type::Vec<N, T>& dst)
{
  assert(src.total() == N);
  std::memcpy(dst.ptr(), src.data, src.total() * src.elemSize());
}
template <unsigned int N, class T>
void cvMat2sofaVector(const cv::Mat_<T>& src, sofa::type::Vec<N, T>& dst)
{
  assert(src.total() == N);
  std::memcpy(dst.ptr(), src.data, src.total() * src.elemSize());
}

template <class T>
void cvMat2sofaVector(const cv::Mat_<T>& src, sofa::type::vector<T>& dst)
{
  dst.reserve(src.total());
  const T* ptr = reinterpret_cast<T*>(src.data);
  for (size_t i = 0; i < src.total(); ++i)
  {
    dst.push_back(ptr[i]);
  }
}
template <class T>
void cvMat2sofaVector(const cv::Mat& src, sofa::type::vector<T>& dst)
{
  dst.reserve(src.total());
  const T* ptr = reinterpret_cast<T*>(src.data);
  for (size_t i = 0; i < src.total(); ++i)
  {
    dst.push_back(ptr[i]);
  }
}

template <class T>
void sofaVector2cvMat(const sofa::type::vector<T>& src, cv::Mat_<T>& dst)
{
  dst = cv::Mat_<T>(src.size(), 1);
  std::memcpy(dst.ptr(), src.data(), src.size() * sizeof(T));
}

template <class T>
void sofaVector2cvMat(const sofa::type::vector<T>& src, cv::Mat& dst)
{
  dst = cv::Mat_<T>(src.size(), 1);
  std::memcpy(dst.ptr(), src.data(), src.size() * sizeof(T));
}

template <int N, class T>
void sofaVector2cvMat(const sofa::type::Vec<N, T>& src, cv::Mat_<T>& dst)
{
  dst = cv::Mat_<T>(N, 1);
  std::memcpy(dst.ptr(), src.data(), N * sizeof(T));
}
template <int N, class T>
void sofaVector2cvMat(const sofa::type::Vec<N, T>& src, cv::Mat& dst)
{
  dst = cv::Mat_<T>(N, 1);
  std::memcpy(dst.ptr(), src.data(), N * sizeof(T));
}
template <unsigned int N, class T>
void sofaVector2cvMat(const sofa::type::Vec<N, T>& src, cv::Mat_<T>& dst)
{
  dst = cv::Mat_<T>(N, 1);
  std::memcpy(dst.ptr(), src.data(), N * sizeof(T));
}
template <unsigned int N, class T>
void sofaVector2cvMat(const sofa::type::Vec<N, T>& src, cv::Mat& dst)
{
  dst = cv::Mat_<T>(N, 1);
  std::memcpy(dst.ptr(), src.data(), N * sizeof(T));
}

}  // namespace matrix

}  // namespace sofacv

#endif  // SOFACV_CVMATUTILS_H
