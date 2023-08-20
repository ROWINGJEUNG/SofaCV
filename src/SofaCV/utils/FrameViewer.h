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

  // SOFA framework안에 모델 시각화를 위해 사용
  // SOFA 내부 타이머가 draw 함수를 자동으로 실행시킴
  void draw(const sofa::core::visual::VisualParams* vparams) override;
  void computeBBox(const sofa::core::ExecParams* params, bool) override;

  // 클래스 내부 데이터 업데이트에 사용 -> 이 클래스는 업데이트 필요한 데이터 없음
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
