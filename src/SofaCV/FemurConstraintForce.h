#ifndef SOFACV_FEMURCONSTRAINTFORCE_H
#define SOFACV_FEMURCONSTRAINTFORCE_H

// 필요한 forcefield 해더 가져옴
#include <SofaBoundaryCondition/ConstantForceField.h>

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
#include <SofaBaseVisual/VisualModelImpl.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/Quater.h>
#include <sofa/helper/AdvancedTimer.h>
#include <SofaBoundaryCondition/FixedConstraint.h>

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
	class SOFA_SOFACV_API FemurConstraintForce : public sofa::component::forcefield::ConstantForceField<Vec3Types>, virtual public ImplicitDataEngine
	{
	public:
		SOFA_CLASS(FemurConstraintForce, ConstantForceField<Vec3Types>);

		FemurConstraintForce(sofa::core::behavior::MechanicalState<Vec3Types>* mParent = nullptr);           // 생성자 정의
		virtual ~FemurConstraintForce();  // 소멸자 정의

		// 클래스 내부 필요 변수들 정의 (doUpdate 실행을 이 함수 안에서 한다)
		void init() override;

		// SOFA framework안에 모델 시각화를 위해 사용 (SOFA 내부 타이머가 draw 함수를 자동으로 실행시킴)
		// 시간에 따라 모델에 가해지는 힘의 방향을 변경하기 위해 사용
		void draw(const sofa::core::visual::VisualParams* vparams) override;

		// 일정 시간마다 동작하는 timer 설정
		void useScheduler(bool);
		void doUpdate() override;

		// 외력이 변경되었을 때 호출되는 함수
		void doUpdateInternal() override {}

		sofa::Data<bool> d_deformableRegi;       // if true, do deformable registration
		sofa::Data<bool> d_visualizeForce;       // if true, visualize current external forces
		sofa::Data<int> d_depthModelSimilarity;  // depth map과 3D model사이 inliner 점 개수 -> DGConstraint에서 받아옴

	protected:
		double forceWeightPercentage1{ 1.0 };
		double forceWeightPercentage2{ 1.0 };
		double forceWeight{ 100.0 };
		int pre_depthModelSimilarity { 0 };
		// Timer 설정에 필요한 함수 및 변수들
		bool m_hasScheduler{ false };  /// If this grabber is controlled by a scheduler
		virtual void handleEvent(sofa::core::objectmodel::Event* e) override;
	};
}
#endif