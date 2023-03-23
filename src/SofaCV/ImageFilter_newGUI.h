#ifndef SOFACV_IMAGEFILTER_NEWGUI_H
#define SOFACV_IMAGEFILTER_NEWGUI_H

#include "SofaCVPlugin.h"
#include "datatypes/cvMat.h"
#include "ImplicitDataEngine.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <opencv2/highgui.hpp>

namespace sofacv
{
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
class SOFA_SOFACV_API ImageFilter : public ImplicitDataEngine
{
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
  virtual ~ImageFilter() override;

  void init() override;
  virtual void doUpdate() override;
  void reinit() override;
  void bindGlTexture(const std::string& imageString);
  virtual void drawFullFrame();

  /**
   * Implement the filter's behavior in this method;
   * @param[in] in input image matrix
   * @param[out] output image matrix to modify
   * @param debug true if applyFilter is called from the mouse callback or from
   * a modified slider in the Debug UI
   */
  virtual void applyFilter(const cv::Mat& /*in*/, cv::Mat& /*out*/,
                           bool /*debug*/ = false) = 0;

  sofa::Data<cvMat> d_img;      ///< [INPUT] image to process
  sofa::Data<cvMat> d_img_out;  ///< [OUTPUT] processed image
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
  void registerData(sofa::Data<uchar>* data, uchar min, uchar max, uchar step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<int>* data, int min, int max, int step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<unsigned>* data, unsigned min, unsigned max,
                    unsigned step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<double>* data, double min, double max,
                    double step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<float>* data, float min, float max, float step);

  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2u>* data, unsigned min,
                    unsigned max, unsigned step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3u>* data, unsigned min,
                    unsigned max, unsigned step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4u>* data, unsigned min,
                    unsigned max, unsigned step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2i>* data, int min,
                    int max, int step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3i>* data, int min,
                    int max, int step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4i>* data, int min,
                    int max, int step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2f>* data, float min,
                    float max, float step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3f>* data, float min,
                    float max, float step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4f>* data, float min,
                    float max, float step);

  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2d>* data, double min,
                    double max, double step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3d>* data, double min,
                    double max, double step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4d>* data, double min,
                    double max, double step);

  /// unregisters all data passed through @see registerData() for the Debug UI
  void unregisterAllData();

  /// mouse callback (if activated through @see activateMouseCallback()

  bool hasMouseCallback() { return m_isMouseCallbackActive; }
  void call_MouseCallback(int et, int b, int m, int x, int y)
  {
    switch (et)
    {
      case 0:
        et = cv::EVENT_MOUSEMOVE;
        break;
      case 1:
      {
        switch (b)
        {
          case 1:
            et = cv::EVENT_LBUTTONDOWN;
            break;
          case 2:
            et = cv::EVENT_RBUTTONDOWN;
            break;
          case 4:
            et = cv::EVENT_MBUTTONDOWN;
            break;
          default:
            et = cv::EVENT_LBUTTONDOWN;
        }
      }
      break;
      case 2:
      {
        switch (b)
        {
          case 1:
            et = cv::EVENT_LBUTTONUP;
            break;
          case 2:
            et = cv::EVENT_RBUTTONUP;
            break;
          case 4:
            et = cv::EVENT_MBUTTONUP;
            break;
          default:
            et = cv::EVENT_LBUTTONUP;
        }
      }
      break;
    }

    int modifier = 0;
    if (m & 1)
        modifier += cv::EVENT_FLAG_SHIFTKEY;
    if (m & 2)
        modifier += cv::EVENT_FLAG_CTRLKEY;
    if (m & 4)
        modifier += cv::EVENT_FLAG_ALTKEY;

    mouseCallback(et, x, y, modifier);
  }

 protected:
  virtual void mouseCallback(int, int, int, int) {}

 private:
  static unsigned m_window_uid;
  bool m_isMouseCallbackActive;
  cv::Mat m_debugImage;
  void reinitDebugWindow();
};

}  // namespace sofacv
#endif  // SOFACV_IMAGEFILTER_NEWGUI_H
