#include "ImageFilterDisplay.h"
#include <GL/glew.h>

#include <SofaQtQuickGUI/SofaScene.h>

#include <QtGui/QOpenGLFramebufferObject>

namespace sofacv
{
namespace gui
{
FilterRenderer::FilterRenderer(const ImageFilterDisplay *imageFilterDisplay)
    : m_imageFilterDisplay(imageFilterDisplay)
{
}

void FilterRenderer::render()
{
  if (!m_imageFilterDisplay) return;
  if (!m_imageFilterDisplay->imageFilterModel()) return;

  glUseProgram(0);
  glClearColor(0, 0, 1, 1);
  glClear(GL_COLOR_BUFFER_BIT);

  this->m_imageFilterDisplay->imageFilterModel()->display();

  update();
}

QOpenGLFramebufferObject *FilterRenderer::createFramebufferObject(
    const QSize &size)
{
  QOpenGLFramebufferObjectFormat format;
  format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
  format.setSamples(4);

  return new QOpenGLFramebufferObject(size, format);
}

/////////////
QQuickFramebufferObject::Renderer *ImageFilterDisplay::createRenderer() const
{
  return new FilterRenderer(this);
}

ImageFilterModel *ImageFilterDisplay::imageFilterModel() const
{
  return m_imageFilterModel;
}

void ImageFilterDisplay::sendMouseEvent(int eventType, int buttonFlags,
                                        int modifierFlags, int mousex,
                                        int mousey)
{
    if (!imageFilterModel() || !imageFilterModel()->imageFilter())
        return;
  double ry =
      imageFilterModel()->imageFilter()->d_img.getValue().rows / this->height();
  double rx =
      imageFilterModel()->imageFilter()->d_img.getValue().cols / this->width();

  if (this->imageFilterModel()->imageFilter()->hasMouseCallback())
  {
    this->imageFilterModel()->imageFilter()->call_MouseCallback(
        eventType, buttonFlags, modifierFlags, int(mousex * rx), int(mousey * ry));
  }
}

void ImageFilterDisplay::setImageFilterModel(ImageFilterModel *imageFilterModel)
{
  m_imageFilterModel = imageFilterModel;
}

}  // namespace gui
}  // namespace sofacv
