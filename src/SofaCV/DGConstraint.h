#ifndef SOFACV_DGCONSTRAINT_H
#define SOFACV_DGCONSTRAINT_H

// 필요한 forcefield 해더 가져옴
#include <sofa/component/mechanicalload/ConstantForceField.h>

// data type 정의하기 위한 헤더 가져옴
#include <sofa/defaulttype/VecTypes.h>
using sofa::defaulttype::Vec3Types;
#include <sofa/defaulttype/RigidTypes.h>
using sofa::defaulttype::Rigid3Types;

// SOFACV 관련 해더 가져옴
#include "SofaCV/SofaCVPlugin.h"
#include "SofaCV/ImplicitDataEngine.h"
#include "SofaCV/datatypes/cvMat.h"

// 기타 필요 헤더 가져옴
#include <sofa/component/visual/VisualModelImpl.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/type/Quat.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/component/constraint/projective/FixedConstraint.h>

// event 처리 위한 헤더
#include <sofa/core/objectmodel/KeypressedEvent.h>

// std 헤더들
#include <mutex>
#include <iostream>
#include <math.h>
#include <cassert>
#include <numeric>

namespace sofacv
{
	class SOFA_SOFACV_API DGConstraintForceField : public sofa::component::mechanicalload::ConstantForceField<Vec3Types>, virtual public ImplicitDataEngine
	{
	public:
		SOFA_CLASS(DGConstraintForceField, ConstantForceField<Vec3Types>);

		DGConstraintForceField(sofa::core::behavior::MechanicalState<Vec3Types>* mParent = nullptr);           // 생성자 정의
		virtual ~DGConstraintForceField();  // 소멸자 정의

		// 클래스 내부 필요 변수들 정의 (doUpdate 실행을 이 함수 안에서 한다)
		void init() override;

		// ICP의 결과를 적용할 MechanicalObject 설정
		void setPathMstateParent(const std::string& o) { mstateParent.setPath(o); }

		// Drillguide 시각화에 사용할 MechanicalObject
		void setPathMObjectDrill(const std::string& o) { mObjectDrill.setPath(o); }

		// Drillguide 시각화에 사용할 MechanicalObject
		sofa::core::objectmodel::SingleLink<DGConstraintForceField, sofa::core::behavior::MechanicalState<Vec3Types>,
			sofa::core::objectmodel::BaseLink::FLAG_STOREPATH | sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK> mObjectDrill;

		// SOFA framework안에 모델 시각화를 위해 사용 (SOFA 내부 타이머가 draw 함수를 자동으로 실행시킴)
		// 시간에 따라 모델에 가해지는 힘의 방향을 변경하기 위해 사용
		void draw(const sofa::core::visual::VisualParams* vparams) override;

		// 일정 시간마다 동작하는 timer 설정
		void useScheduler(bool);
		void doUpdate() override;

		// 외력이 변경되었을 때 호출되는 함수
		void doUpdateInternal() override {}
				
		sofa::Data<cvMat> d_reconFrame;          // depath map data
		sofa::Data<int> d_reconPointNum;         // depth map을 구성하는 점들의 개수
		sofa::Data<int> d_depthModelSimilarity;  // depth map과 3D model사이 inliner 점 개수 -> femur에 가하는 힘 제어위해 사용
		sofa::Data<bool> d_deformableRegi;       // if true, do deformable registration
		sofa::Data<bool> d_visualizeDepth;       // if true, visualize current depth map
		sofa::Data<bool> d_visualizeForce;       // if true, visualize current external forces

	protected:
		// ICP의 결과를 적용할 MechanicalObject
		sofa::core::objectmodel::SingleLink<DGConstraintForceField, sofa::core::behavior::MechanicalState<Vec3Types>,
			sofa::core::objectmodel::BaseLink::FLAG_STOREPATH|sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK> mstateParent;

		// Timer 설정에 필요한 함수 및 변수들
		bool m_hasScheduler{ false };  /// If this grabber is controlled by a scheduler
		virtual void handleEvent(sofa::core::objectmodel::Event* e) override;

		// initial registration 여부를 확인하기 위한 변수
		bool m_initialRegistration{ false };
		bool m_programInitialization{ false };

		// realSense에서 발송된 depth map data
		cv::Mat realSenseData;

		double dilDirection1;
		double dilDirection2;
		double dilDirection3;
		double dilPosition1;
		double dilPosition2;
		double dilPosition3;
		};
}
#endif