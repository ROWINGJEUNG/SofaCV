/******************************************************************************
 *       SOFAOR, SOFA plugin for the Operating Room, development version       *
 *                        (c) 2017 INRIA, MIMESIS Team                         *
 *                                                                             *
 * This program is a free software; you can redistribute it and/or modify it   *
 * under the terms of the GNU Lesser General Public License as published by    *
 * the Free Software Foundation; either version 1.0 of the License, or (at     *
 * your option) any later version.                                             *
 *                                                                             *
 * This program is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details.                                                           *
 *                                                                             *
 * You should have received a copy of the GNU Lesser General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.        *
 *******************************************************************************
 * Authors: Bruno Marques and external contributors (see Authors.txt)          *
 *                                                                             *
 * Contact information: contact-mimesis@inria.fr                               *
 ******************************************************************************/

#ifndef SOFACV_IMAGEFILTER_OLDGUI_H
#define SOFACV_IMAGEFILTER_OLDGUI_H

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include "ImplicitDataEngine.h"
#include "SofaCVPlugin.h"
#include "datatypes/cvMat.h"

namespace sofacv
{
template <class T>
class ScalarSliderManager;

template <class T, class U>
class CustomSliderManager;

template <unsigned int N, class T>
class VecSliderManager;

class DSM;

/**
 *  \brief provides a Debug UI mechanism for image filters
 *
 * Implementation good rules: @see ImplicitDataEngine for additional impl rules
 *
 * void init()
 * {
 *    // if your filter needs parameter tuning
 *    registerData(&d_data, minVal, maxVal, step)
 *
 *    // if your filter requires manual interaction (e.g. Segmenter2D)
 *    activateMouseCallback();
 *
 *    ImageFilter::init(); // Always call at the end of init()
 * }
 *
 * void applyFilter(const cv::Mat& in, cv::Mat& out, bool **debug**)
 * {
 *   // code here your filter's behavior
 * }
 *
 * // optional
 * void reinit()
 * {
 *   ImageFilter::reinit();
 * }
 *
 * void update()
 * {
 *   ImageFilter::update();
 * }
 *
 */
class ImageFilter : public ImplicitDataEngine
{
  static void _mouseCallback(int e, int x, int y, int f, void* d);

 public:
  SOFA_CLASS(ImageFilter, ImplicitDataEngine);

  /**
   * @brief ImageFilter
   * @param outputImage set to false if you don't want this filter to output an
   * image useful for filters
   * such as FeatureDetectors / matchers, where we want to visualize the
   * resulting image in
   * the debug window, but we don't need the debug output for further
   * processing)
   */
  ImageFilter();
  virtual ~ImageFilter();

  virtual void init() override;
  virtual void doUpdate() override;
  virtual void reinit() override;

  /**
   * Implement the filter's behavior in this method;
   * @param[in] in input image matrix
   * @param[out] output image matrix to modify
   * @param debug true if applyFilter is called from the mouse callback or from
   * a modified slider in the Debug UI
   */
  virtual void applyFilter(const cv::Mat& in, cv::Mat& out,
                           bool debug = false) = 0;

  /// Creates the Debug UI window if not created yet, sets its registered
  /// trackbars, and activates the necessary callbacks
  void refreshDebugWindow();

  sofa::Data<cvMat> d_img;                ///< [INPUT] image to process
  sofa::Data<cvMat> d_img_out;            ///< [OUTPUT] processed image
  sofa::Data<bool> d_displayDebugWindow;  ///< toggles the Debug UI
  sofa::Data<bool> d_isActive;  ///< Whether the filter is performed or not
  sofa::Data<bool> d_outputImage;

  /// activates the mouse callback when mouse interaction is performed on the
  /// image's debug UI
  void activateMouseCallback();
  /**
   *
   *
   * \fn void registerData(sofa::Data<bool>* data)
   *
   * \brief Pass data to @registerData to bind them to ImageFilter's debug UI
   *
   * @param[in] data the data to bind to a slider
   *
   */
  void registerData(sofa::Data<bool>* data);

  /// @see void registerData(sofa::Data<bool>* data)
  void registerData(sofa::Data<sofa::helper::OptionsGroup>* data);

  /**
   * \fn void registerData(sofa::Data<uchar>* data, uchar min, uchar max, uchar\
   * step)
   *
   * \brief Pass data to @registerData to bind them to ImageFilter's debug UI
   *
   * @param[in] data the data to bind to a slider
   * @param[in] min the minimum value of the data
   * @param[in] max the maximum value of the data
   * @param[in] step the discretized interval for the slider
   *
   */
  void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
                    uchar step = 1);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<int>* data, int min, int max, int step = 1);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<unsigned>* data, unsigned min, unsigned max,
                    unsigned step = 1);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<double>* data, double min, double max,
                    double step = 1);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<float>* data, float min, float max,
                    float step = 1.0f);

  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2u>* data, unsigned min,
                    unsigned max, unsigned step = 1);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3u>* data, unsigned min,
                    unsigned max, unsigned step = 1);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4u>* data, unsigned min,
                    unsigned max, unsigned step = 1);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2i>* data, int min,
                    int max, int step = 1);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3i>* data, int min,
                    int max, int step = 1);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4i>* data, int min,
                    int max, int step = 1);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2f>* data, float min,
                    float max, float step = 1.0f);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3f>* data, float min,
                    float max, float step = 1.0f);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4f>* data, float min,
                    float max, float step = 1.0f);

  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2d>* data, double min,
                    double max, double step = 1.0);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3d>* data, double min,
                    double max, double step = 1.0);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4d>* data, double min,
                    double max, double step = 1.0);

  /// unregisters all data passed through @see registerData() for the Debug UI
  void unregisterAllData();

 protected:
  /// mouse callback (if activated through @see activateMouseCallback()
  virtual void mouseCallback(int, int, int, int) {}

 private:
  static int m_window_uid;
  bool m_isMouseCallbackActive;
  cv::Mat m_debugImage;
  void reinitDebugWindow();
  sofa::core::DataTracker m_displayDebugDataTracker;

  std::vector<DSM*> m_params;
  const std::string m_win_name;
};

}  // namespace sofacv
#endif  // SOFACV_IMAGEFILTER_OLDGUI_H
