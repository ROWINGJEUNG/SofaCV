#ifndef SOFACV_GUI_IMAGEFILTERDISPLAY_H
#define SOFACV_GUI_IMAGEFILTERDISPLAY_H

#include "SofaCVPlugin.h"
#include "ImageFilter.h"
#include "gui/ImageFilterModel.h"

#include <SofaQtQuickGUI/SofaComponent.h>
#include <SofaQtQuickGUI/SofaQtQuickGUI.h>

#include <QObject>
#include <QtQuick/QQuickFramebufferObject>

#include <math.h>

namespace sofacv
{
namespace gui
{
class ImageFilterDisplay;

class SOFA_SOFACV_API FilterRenderer
    : public QQuickFramebufferObject::Renderer
{
 public:
  FilterRenderer(
      const ImageFilterDisplay* imageFilterDisplay);
  void render();
  QOpenGLFramebufferObject* createFramebufferObject(const QSize& size);

 private:
  const ImageFilterDisplay* m_imageFilterDisplay;
};

class SOFA_SOFACV_API ImageFilterDisplay : public QQuickFramebufferObject
{
  Q_OBJECT

  Q_PROPERTY(ImageFilterModel* imageFilterModel READ imageFilterModel WRITE
                 setImageFilterModel)  // NOTIFY imageFilterModelChanged)
 public:
  Renderer* createRenderer() const;

  ImageFilterModel* imageFilterModel() const;

 public slots:
  void sendMouseEvent(int eventType, int buttonFlags, int modifierFlags,
                      int mousex, int mousey);

 protected:
  void setImageFilterModel(ImageFilterModel* imageFilterModel);

 private:
  ImageFilterModel* m_imageFilterModel;
};

}  // namespace gui
}  // namespace sofacv

#endif  // SOFACV_GUI_IMAGEFILTERDISPLAY_H
