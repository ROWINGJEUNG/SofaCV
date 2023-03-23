#include "PCViewer.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/visual/DrawTool.h>
#include <sofa/helper/system/gl.h>

namespace sofacv
{
namespace utils
{
SOFA_DECL_CLASS(PCViewer)

int PCViewerClass =
    sofa::core::RegisterObject("component to visualize pointclouds in Sofa")
        .add<PCViewer>();

PCViewer::PCViewer()
    : d_size(initData(&d_size, 1.0, "size", "point size")),
      d_positions(initData(&d_positions, "points", "point cloud's positions")),
      d_colors(initData(&d_colors, "colors", "point cloud's point colors")),
      d_color(initData(&d_color, "color", "point cloud's point color"))
{
  f_listening.setValue(true);
}

PCViewer::~PCViewer() {}
void PCViewer::init()
{
  addInput(&d_positions);
  addInput(&d_colors);
  update();
}

void PCViewer::doUpdate()
{
  m_positions = d_positions.getValue();
  const sofa::helper::vector<Vec3b>& c = d_colors.getValue();
  m_colors.resize(c.size());
  for (size_t i = 0; i < c.size(); ++i)
    m_colors[i].set(c[i][2] / 255.0f, c[i][1] / 255.0f, c[i][0] / 255.0f, 1.0f);
}

void PCViewer::computeBBox(const sofa::core::ExecParams* , bool)
{
  if (m_positions.size() == 0) return;

  double minBBox[3] = {std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max()};
  double maxBBox[3] = {-std::numeric_limits<double>::max(),
                       -std::numeric_limits<double>::max(),
                       -std::numeric_limits<double>::max()};

  for (unsigned int i = 0; i < m_positions.size(); i++)
  {
    const sofa::defaulttype::Vector3& p = m_positions[i];
    for (int c = 0; c < 3; c++)
    {
      if (p[c] > maxBBox[c]) maxBBox[c] = p[c];
      if (p[c] < minBBox[c]) minBBox[c] = p[c];
    }
  }
  this->f_bbox.setValue(sofa::defaulttype::TBoundingBox<double>(minBBox, maxBBox));
}

void PCViewer::draw(const sofa::core::visual::VisualParams*)
{
  update();
  glPointSize(d_size.getValue());
  glDisable(GL_LIGHTING);
  glBegin(GL_POINTS);

  if (d_color.isSet())
  {
    const Vec3b& color = d_color.getValue();
    for (size_t i = 0; i < m_positions.size(); ++i)
    {
      Vector3 pt = m_positions[i];
      glColor3f(color[0] / 256.0f, color[1] / 256.0f, color[2] / 256.0f);
      glVertex3f(pt.x(), pt.y(), pt.z());
    }
  }
  else if (d_colors.isSet())
  {
    const sofa::helper::vector<Vec3b>& colors = d_colors.getValue();
    for (size_t i = 0; i < m_positions.size(); ++i)
    {
      Vector3 pt = m_positions[i];
      glColor4f(colors[i][2] / 256.0f, colors[i][1] / 256.0f,
                colors[i][0] / 256.0f, 1.0f);
      glVertex3f(pt.x(), pt.y(), pt.z());
    }
  }
  else
  {
    for (size_t i = 0; i < m_positions.size(); ++i)
    {
      Vector3 pt = m_positions[i];
      glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
      glVertex3f(pt.x(), pt.y(), pt.z());
    }
  }
  glEnd();
  glPointSize(1);
  glEnable(GL_LIGHTING);

  //  core::visual::DrawToolGL dt;
  //  if (m_positions.size() != m_colors.size())
  //    dt.drawPoints(m_positions, d_size.getValue(),
  //                  defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
  //  else
  //    dt.drawPoints(m_positions, d_size.getValue(), m_colors);
}

void PCViewer::reinit() { update(); }
}  // namespace utils
}  // namespace sofacv
