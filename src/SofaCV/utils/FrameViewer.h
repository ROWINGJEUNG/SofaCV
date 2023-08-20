#ifndef SOFACV_COMMON_FRAMEVIEWER_H
#define SOFACV_COMMON_FRAMEVIEWER_H

#include "SofaCV/SofaCVPlugin.h"
#include "SofaCV/ImplicitDataEngine.h"
#include "SofaCV/datatypes/cvMat.h"

#include <sofa/component/visual/VisualModelImpl.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/type/Quat.h>

namespace sofacv
{
namespace utils
{
class SOFA_SOFACV_API FrameViewer : virtual public ImplicitDataEngine
{
 public:
    SOFA_CLASS(FrameViewer, ImplicitDataEngine);

 public:
  FrameViewer();
  virtual ~FrameViewer();

  void init() override;
  void reinit() override;

  // SOFA framework�ȿ� �� �ð�ȭ�� ���� ���
  // SOFA ���� Ÿ�̸Ӱ� draw �Լ��� �ڵ����� �����Ŵ
  void draw(const sofa::core::visual::VisualParams* vparams) override;
  void computeBBox(const sofa::core::ExecParams* params, bool) override;

  // Ŭ���� ���� ������ ������Ʈ�� ��� -> �� Ŭ������ ������Ʈ �ʿ��� ������ ����
  void doUpdate() override { }

    sofa::Data<cvMat> d_frame;
	sofa::Data<cvMat> d_reconFrame;
	sofa::Data<sofa::type::vector<sofa::type::Vector3> > d_corners;
	sofa::Data<sofa::helper::OptionsGroup> d_mode;
	sofa::Data<float> d_alpha;

 private:
	void perspectiveDraw();
	void orthoDraw();
	void bindGlTexture(const std::string& imageString);
};

}  // namespace utils
}  // namespace sofaor
#endif  // SOFACV_COMMON_FRAMEVIEWER_H
