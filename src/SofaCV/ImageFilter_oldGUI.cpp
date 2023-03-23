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


#include "ImageFilter_oldGUI.h"
#include "DataSliderMgr.inl"

#include <sofa/gl/DrawToolGL.h>

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/gl/RAII.h>
#include <sofa/helper/system/gl.h>

#include <opencv2/highgui.hpp>
#include<opencv2/highgui/highgui_c.h>

namespace sofacv
{
    int ImageFilter::m_window_uid = 0;

    void ImageFilter::_mouseCallback(int e, int x, int y, int f, void *d)
    {
        reinterpret_cast<ImageFilter*>(d)->mouseCallback(e, x, y, f);
    }

    ImageFilter::ImageFilter()
        : d_img(initData(
                    &d_img, cvMat(), "img",
                    "Input image, that will undergo changes through the filter.", false)),
          d_img_out(initData(&d_img_out, "img_out",
                             "Output image, holding the filter's result", false)),
          d_displayDebugWindow(initData(&d_displayDebugWindow, false, "Debug",
                                        "Display a debug window to see in live "
                                        "the changes applied to the filter")),
          d_isActive(initData(&d_isActive, true, "isActive",
                              "if false, does not call applyFilter")),
          d_outputImage(
              initData(&d_outputImage, true, "outputImage",
                       "whether or not the computed image will be set in img_out")),
          m_win_name(std::to_string(m_window_uid) + "_" + getClassName())
    {
      addAlias(&d_img_out, "img1_out");
      f_listening.setValue(true);
      m_window_uid++;
    }

ImageFilter::~ImageFilter() {}
void ImageFilter::init()
{
  m_displayDebugDataTracker.trackData(d_displayDebugWindow);
  addInput(&d_img);
  addOutput(&d_img_out);
  update();
}

void ImageFilter::doUpdate()
{
  sofa::helper::AdvancedTimer::stepBegin("Image Filters");

  if (!d_isActive.getValue())
  {
    // filter inactive, out = in
    d_img_out.setValue(d_img.getValue());

    if (d_displayDebugWindow.getValue())
    {
      cv::imshow(m_win_name, d_img_out.getValue());
      //      cv::waitKey(1);
    }
    sofa::helper::AdvancedTimer::stepEnd("Image Filters");
    return;
  }
  m_debugImage = d_img.getValue().zeros(
      d_img.getValue().rows, d_img.getValue().cols, d_img.getValue().type());
  applyFilter(d_img.getValue(), m_debugImage);
  if (!d_outputImage.getValue())
  {
    d_img_out.setValue(d_img.getValue());
  }
  else
  {
    d_img_out.setValue(m_debugImage);
  }
  if (d_displayDebugWindow.getValue() && !m_debugImage.empty())
  {
    cv::imshow(m_win_name, m_debugImage);
  }
  sofa::helper::AdvancedTimer::stepEnd(("Image Filters"));
}

void ImageFilter::reinit()
{
  if (m_displayDebugDataTracker.hasChanged() && d_isActive.getValue())
    refreshDebugWindow();

  //  // to set needsRefresh to true
  //  ImplicitDataEngine::reinit();
}

void ImageFilter::reinitDebugWindow()
{
  if (!d_displayDebugWindow.getValue())
  {
    if (cvGetWindowHandle(m_win_name.c_str()) && m_isMouseCallbackActive)
      cv::setMouseCallback(m_win_name, NULL, NULL);
    if (cvGetWindowHandle(m_win_name.c_str())) cv::destroyWindow(m_win_name);
    return;
  }

  // WITH QT:
  cv::namedWindow(m_win_name,
                  CV_WINDOW_NORMAL | CV_WINDOW_AUTOSIZE | CV_GUI_EXPANDED);
  for (DSM* dmgr : m_params) dmgr->createSlider(m_win_name);
  if (m_isMouseCallbackActive && cvGetWindowHandle(m_win_name.c_str()))
    cv::setMouseCallback(m_win_name, &ImageFilter::_mouseCallback, this);
}

void ImageFilter::refreshDebugWindow()
{
  reinitDebugWindow();
  if (!d_displayDebugWindow.getValue() ||
      !cvGetWindowHandle(m_win_name.c_str()))
    return;
  applyFilter(d_img.getValue(), m_debugImage, true);
  if (m_debugImage.empty()) return;

  cv::imshow(m_win_name, m_debugImage);
}

void ImageFilter::activateMouseCallback()
{
  m_isMouseCallbackActive = !m_isMouseCallbackActive;
}

void ImageFilter::unregisterAllData() { m_params.clear(); }

void ImageFilter::registerData(sofa::Data<bool>* data)
{
  m_params.push_back(new ScalarSliderManager<bool>(data));
}

void ImageFilter::registerData(sofa::Data<sofa::helper::OptionsGroup>* data)
{
  m_params.push_back(
      new CustomSliderManager<sofa::helper::OptionsGroup, int>(data));
}

void ImageFilter::registerData(sofa::Data<int>* data, int min, int max,
                               int step)
{
  m_params.push_back(new ScalarSliderManager<int>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<unsigned>* data, unsigned min,
                               unsigned max, unsigned step)
{
  m_params.push_back(new ScalarSliderManager<unsigned>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<double>* data, double min, double max,
                               double step)
{
  m_params.push_back(new ScalarSliderManager<double>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<float>* data, float min, float max,
                               float step)
{
  m_params.push_back(new ScalarSliderManager<float>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2u>* data,
                               unsigned min, unsigned max, unsigned step)
{
  m_params.push_back(new VecSliderManager<2, unsigned>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3u>* data,
                               unsigned min, unsigned max, unsigned step)
{
  m_params.push_back(new VecSliderManager<3, unsigned>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4u>* data,
                               unsigned min, unsigned max, unsigned step)
{
  m_params.push_back(new VecSliderManager<4, unsigned>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2i>* data,
                               int min, int max, int step)
{
  m_params.push_back(new VecSliderManager<2, int>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3i>* data,
                               int min, int max, int step)
{
  m_params.push_back(new VecSliderManager<3, int>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4i>* data,
                               int min, int max, int step)
{
  m_params.push_back(new VecSliderManager<4, int>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2f>* data,
                               float min, float max, float step)
{
  m_params.push_back(new VecSliderManager<2, float>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3f>* data,
                               float min, float max, float step)
{
  m_params.push_back(new VecSliderManager<3, float>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4f>* data,
                               float min, float max, float step)
{
  m_params.push_back(new VecSliderManager<4, float>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2d>* data,
                               double min, double max, double step)
{
  m_params.push_back(new VecSliderManager<2, double>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3d>* data,
                               double min, double max, double step)
{
  m_params.push_back(new VecSliderManager<3, double>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4d>* data,
                               double min, double max, double step)
{
  m_params.push_back(new VecSliderManager<4, double>(data, min, max, step));
}

}  // namespace sofacv
