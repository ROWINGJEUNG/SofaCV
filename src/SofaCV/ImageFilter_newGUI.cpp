#include "ImageFilter_newGUI.h"
#include <opencv2/highgui.hpp>

#include <sofa/core/visual/DrawToolGL.h>

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/gl/RAII.h>
#include <sofa/helper/system/gl.h>

namespace sofacv
{
SOFA_DECL_CLASS(ImageFilter)

unsigned ImageFilter::m_window_uid = 0;

ImageFilter::ImageFilter()
    : d_img(initData(
          &d_img, cvMat(), "img",
          "Input image, that will undergo changes through the filter.", false)),
      d_img_out(initData(&d_img_out, "img_out",
                         "Output image, holding the filter's result", false)),
      d_isActive(initData(&d_isActive, true, "isActive",
                          "if false, does not call applyFilter")),
      d_outputImage(
          initData(&d_outputImage, true, "outputImage",
                   "whether or not the computed image will be set in img_out"))
{
  addAlias(&d_img_out, "img1_out");
  f_listening.setValue(true);
}

ImageFilter::~ImageFilter() {}
void ImageFilter::init()
{
  if (getClassName() == "ImageFilter")
    msg_error(getName())
        << "Cannot instantiate an abstract component of type ImageFilter";
  addInput(&d_isActive, true);
  addInput(&d_outputImage, true);
  addInput(&d_img);
  addOutput(&d_img_out);
}

void ImageFilter::doUpdate()
{
  sofa::helper::AdvancedTimer::stepBegin("Image Filters");

  if (!d_isActive.getValue())
  {
    // filter inactive, out = in
    d_img_out.setValue(d_img.getValue());

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
  sofa::helper::AdvancedTimer::stepEnd(("Image Filters"));
}

void ImageFilter::reinit()
{
  std::cout << getName() << " Reinit()" << std::endl;
//  if (d_isActive.getValue()) m_debugImage = d_img_out.getValue().clone();
}

void ImageFilter::bindGlTexture(const std::string& imageString)
{
  glEnable(GL_TEXTURE_2D);  // enable the texture
  glDisable(GL_LIGHTING);   // disable the light

  glBindTexture(GL_TEXTURE_2D, 0);  // texture bind

  unsigned internalFormat = GL_RGB;
  unsigned format = GL_BGR_EXT;
  unsigned type = GL_UNSIGNED_BYTE;

  switch (m_debugImage.channels())
  {
    case 1:  // grayscale
      internalFormat = GL_LUMINANCE;
      format = GL_RED;
      break;
    case 3:  // RGB / BGR
      internalFormat = GL_RGB;

      format = GL_BGR_EXT;
      break;
    case 4:  // RGBA / BGRA
      internalFormat = GL_RGBA;
      format = GL_BGRA_EXT;
      break;
  }
  switch (m_debugImage.type())
  {
    case CV_8U:
      type = GL_UNSIGNED_BYTE;
      break;
    case CV_8S:
      type = GL_BYTE;
      break;
    case CV_16U:
      type = GL_UNSIGNED_SHORT;
      break;
    case CV_16S:
      type = GL_SHORT;
      break;
    case CV_32S:
      type = GL_INT;
      break;
    case CV_32F:
      type = GL_FLOAT;
      break;
    default:
      type = GL_UNSIGNED_BYTE;
      break;
  }
  glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, m_debugImage.cols,
               m_debugImage.rows, 0, format, type, imageString.c_str());
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

void ImageFilter::drawFullFrame()
{
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthMask(GL_FALSE);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, 1, 0, 1, -1, 1);  // orthogonal view
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  std::stringstream imageString;
  imageString.write((const char*)m_debugImage.data,
                    m_debugImage.total() * m_debugImage.elemSize());

  bindGlTexture(imageString.str());

  glBegin(GL_QUADS);
  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  glTexCoord2f(0, 0);
  glVertex2f(0, 0);
  glTexCoord2f(1, 0);
  glVertex2f(1, 0);
  glTexCoord2f(1, 1);
  glVertex2f(1, 1);
  glTexCoord2f(0, 1);
  glVertex2f(0, 1);
  glEnd();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);     // enable light
  glDisable(GL_TEXTURE_2D);  // disable texture 2D
                             // glDepthMask (GL_TRUE);		// enable zBuffer
  glDisable(GL_BLEND);
  glDepthMask(GL_TRUE);
}

void ImageFilter::activateMouseCallback()
{
  m_isMouseCallbackActive = !m_isMouseCallbackActive;
}

void ImageFilter::unregisterAllData() {}

void ImageFilter::registerData(sofa::Data<bool>* /*data*/) {}

void ImageFilter::registerData(sofa::Data<sofa::helper::OptionsGroup>* /*data*/) {}

void ImageFilter::registerData(sofa::Data<int>* /*data*/, int /*min*/, int /*max*/,
                               int /*step*/)
{
}
void ImageFilter::registerData(sofa::Data<unsigned>* /*data*/, unsigned /*min*/,
                               unsigned /*max*/, unsigned /*step*/)
{
}
void ImageFilter::registerData(sofa::Data<double>* /*data*/, double /*min*/, double /*max*/,
                               double /*step*/)
{
}
void ImageFilter::registerData(sofa::Data<float>* /*data*/, float /*min*/, float /*max*/,
                               float /*step*/)
{
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2u>* /*data*/,
                               unsigned /*min*/, unsigned /*max*/, unsigned /*step*/)
{
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3u>* /*data*/,
                               unsigned /*min*/, unsigned /*max*/, unsigned /*step*/)
{
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4u>* /*data*/,
                               unsigned /*min*/, unsigned /*max*/, unsigned /*step*/)
{
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2i>* /*data*/,
                               int /*min*/, int /*max*/, int /*step*/)
{
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3i>* /*data*/,
                               int /*min*/, int /*max*/, int /*step*/)
{
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4i>* /*data*/,
                               int /*min*/, int /*max*/, int /*step*/)
{
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2f>* /*data*/,
                               float /*min*/, float /*max*/, float /*step*/)
{
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3f>* /*data*/,
                               float /*min*/, float /*max*/, float /*step*/)
{
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4f>* /*data*/,
                               float /*min*/, float /*max*/, float /*step*/)
{
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2d>* /*data*/,
                               double /*min*/, double /*max*/, double /*step*/)
{
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3d>* /*data*/,
                               double /*min*/, double /*max*/, double /*step*/)
{
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4d>* /*data*/,
                               double /*min*/, double /*max*/, double /*step*/)
{
}

}  // namespace sofacv
