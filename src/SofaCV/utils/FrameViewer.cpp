#include "FrameViewer.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/gl/DrawToolGL.h>

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/gl/RAII.h>
#include <sofa/helper/system/gl.h>
#include <sofa/simulation/AnimateBeginEvent.h>

// draw 안에 변수로 들어가는 visualparams를 사용하려면 이 해더를 include 해야한다.
#include <sofa/core/visual/VisualParams.h>


namespace sofacv
{
namespace utils
{
SOFA_DECL_CLASS(FrameViewer)

int FrameViewerClass = sofa::core::RegisterObject(
                           "debug component to project images in OpenGL using "
                           "the a projection matrix")
                           .add<FrameViewer>();

FrameViewer::FrameViewer()
    : d_frame(initData(&d_frame, "img", "frame to display in opencv window")),
      d_reconFrame(initData(&d_reconFrame, "3DPoints", "3D points from realsense")),
      d_corners(initData(&d_corners, "corners",
                         "3D world coordinates of the image corners")),
      d_mode(initData(&d_mode, "mode",
                      "viewer mode (PERSPECTIVE, ORTHO, HIDDEN)")),
      d_alpha(initData(&d_alpha, 1.0f, "alpha", "alpha value for the texture"))
{
  sofa::helper::OptionsGroup *t = d_mode.beginEdit();
  t->setNames(3, "PERSPECTIVE", "ORTHO", "HIDDEN");
  t->setSelectedItem(0);
  d_mode.endEdit();
}

FrameViewer::~FrameViewer() {}
void FrameViewer::init()
{
  addInput(&d_frame);
  addInput(&d_reconFrame);
  addInput(&d_corners);

  update();
}

void FrameViewer::reinit() {}

void FrameViewer::bindGlTexture(const std::string &imageString)
{
  glEnable(GL_TEXTURE_2D);  // enable the texture
  glDisable(GL_LIGHTING);   // disable the light

  glBindTexture(GL_TEXTURE_2D, 0);  // texture bind

  unsigned internalFormat = GL_RGB;
  unsigned format = GL_BGR_EXT;
  unsigned type = GL_UNSIGNED_BYTE;

  switch (d_frame.getValue().channels())
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
  switch (d_frame.getValue().depth())
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
  glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, d_frame.getValue().cols,
               d_frame.getValue().rows, 0, format, type, imageString.c_str());
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

// Render from the viewpoint of the opengl's context
void FrameViewer::perspectiveDraw()
{
  std::stringstream imageString;
  imageString.write((const char *)d_frame.getValue().data,
                    d_frame.getValue().total() * d_frame.getValue().elemSize());

  bindGlTexture(imageString.str());

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //	glDepthMask(GL_FALSE);

  sofa::helper::vector<sofa::defaulttype::Vector3> p = d_corners.getValue();

  glBegin(GL_QUADS);
  glColor4f(1.0f, 1.0f, 1.0f, d_alpha.getValue());
  glTexCoord2f(0, 0);
  glVertex3f(p[0][0], p[0][1], p[0][2]);
  glTexCoord2f(1, 0);
  glVertex3f(p[1][0], p[1][1], p[1][2]);
  glTexCoord2f(1, 1);
  glVertex3f(p[2][0], p[2][1], p[2][2]);
  glTexCoord2f(0, 1);
  glVertex3f(p[3][0], p[3][1], p[3][2]);
  glEnd();

  // glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);     // enable light
  glDisable(GL_TEXTURE_2D);  // disable texture 2D
                             // glDepthMask (GL_TRUE);		// enable zBuffer
  glDisable(GL_BLEND);
  //	glDepthMask(GL_TRUE);
}

void FrameViewer::orthoDraw()
{
  std::stringstream imageString;
  imageString.write((const char *)d_frame.getValue().data,
                    d_frame.getValue().total() * d_frame.getValue().elemSize());

  bindGlTexture(imageString.str());

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//  glDepthMask(GL_FALSE);

  glMatrixMode(GL_PROJECTION);  // init the projection matrix
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, 1, 0, 1, -1, 1);  // orthogonal view
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glBegin(GL_QUADS);
  glColor4f(1.0f, 1.0f, 1.0f, d_alpha.getValue());
  glTexCoord2f(0, 1);
  glVertex2f(0, 0);
  glTexCoord2f(1, 1);
  glVertex2f(1, 0);
  glTexCoord2f(1, 0);
  glVertex2f(1, 1);
  glTexCoord2f(0, 0);
  glVertex2f(0, 1);
  glEnd();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

//  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);     // enable light
  glDisable(GL_TEXTURE_2D);  // disable texture 2D
                             // glDepthMask (GL_TRUE);		// enable zBuffer
  glDisable(GL_BLEND);
  glDepthMask(GL_TRUE);
}

// SOFA framework안에 모델 시각화를 위해 사용
// SOFA 내부 타이머가 draw 함수를 자동으로 실행시킴
void FrameViewer::draw(const sofa::core::visual::VisualParams* vparams)
{
  if (d_frame.getValue().data)
  {
      switch (d_mode.getValue().getSelectedId())
      {
      case 0:
          perspectiveDraw();
          break;
      case 1:
          orthoDraw();
          break;
      default:
          break;
      }
  }

  vparams->drawTool()->saveLastState();
}

void FrameViewer::computeBBox(const sofa::core::ExecParams *, bool)
{
  if (d_mode.getValue().getSelectedId() != 0) return;

  const sofa::helper::vector<sofa::defaulttype::Vector3> &x =
      d_corners.getValue();
  if (x.empty()) return;

  double minBBox[3] = {std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max()};
  double maxBBox[3] = {-std::numeric_limits<double>::max(),
                       -std::numeric_limits<double>::max(),
                       -std::numeric_limits<double>::max()};

  for (unsigned int i = 0; i < x.size(); i++)
  {
    const sofa::defaulttype::Vector3 &p = x[i];
    for (int c = 0; c < 3; c++)
    {
      if (p[c] > maxBBox[c]) maxBBox[c] = p[c];
      if (p[c] < minBBox[c]) minBBox[c] = p[c];
    }
  }
  this->f_bbox.setValue(sofa::defaulttype::TBoundingBox<double>(minBBox, maxBBox));
}

}  // namespace utils
}  // namespace sofacv
