#ifndef SOFACV_DGCONSTRAINT_H
#define SOFACV_DGCONSTRAINT_H

// �ʿ��� forcefield �ش� ������
#include <SofaBoundaryCondition/ConstantForceField.h>

// data type �����ϱ� ���� ��� ������
#include <sofa/defaulttype/VecTypes.h>
using sofa::defaulttype::Vec3Types;
#include <sofa/defaulttype/RigidTypes.h>
using sofa::defaulttype::Rigid3Types;

// SOFACV ���� �ش� ������
#include "SofaCV/SofaCVPlugin.h"
#include "SofaCV/ImplicitDataEngine.h"
#include "SofaCV/datatypes/cvMat.h"

// ��Ÿ �ʿ� ��� ������
#include <SofaBaseVisual/VisualModelImpl.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/Quater.h>
#include <sofa/helper/AdvancedTimer.h>
#include <SofaBoundaryCondition/FixedConstraint.h>

// event ó�� ���� ���
#include <sofa/core/objectmodel/KeypressedEvent.h>

// std �����
#include <mutex>
#include <iostream>
#include <math.h>
#include <cassert>
#include <numeric>

namespace sofacv
{
	class SOFA_SOFACV_API DGConstraintForceField : public sofa::component::forcefield::ConstantForceField<Vec3Types>, virtual public ImplicitDataEngine
	{
	public:
		SOFA_CLASS(DGConstraintForceField, ConstantForceField<Vec3Types>);

		DGConstraintForceField(sofa::core::behavior::MechanicalState<Vec3Types>* mParent = nullptr);           // ������ ����
		virtual ~DGConstraintForceField();  // �Ҹ��� ����

		// Ŭ���� ���� �ʿ� ������ ���� (doUpdate ������ �� �Լ� �ȿ��� �Ѵ�)
		void init() override;

		// ICP�� ����� ������ MechanicalObject ����
		void setPathMstateParent(const std::string& o) { mstateParent.setPath(o); }

		//// ICP�� �ʿ��� �����͸� ������ MechanicalObject ����
		//void setPathMObjectICPTarget(const std::string& o) { mObjectICPTarget.setPath(o); }

		// Drillguide �ð�ȭ�� ����� MechanicalObject
		void setPathMObjectDrill(const std::string& o) { mObjectDrill.setPath(o); }

		// Drillguide �ð�ȭ�� ����� MechanicalObject
		sofa::core::objectmodel::SingleLink<DGConstraintForceField, sofa::core::behavior::MechanicalState<Vec3Types>,
			sofa::core::objectmodel::BaseLink::FLAG_STOREPATH | sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK> mObjectDrill;

		// SOFA framework�ȿ� �� �ð�ȭ�� ���� ��� (SOFA ���� Ÿ�̸Ӱ� draw �Լ��� �ڵ����� �����Ŵ)
		// �ð��� ���� �𵨿� �������� ���� ������ �����ϱ� ���� ���
		void draw(const sofa::core::visual::VisualParams* vparams) override;

		// ���� �ð����� �����ϴ� timer ����
		void useScheduler(bool);
		void doUpdate() override;

		// �ܷ��� ����Ǿ��� �� ȣ��Ǵ� �Լ�
		void doUpdateInternal() override {}
				
		sofa::Data<cvMat> d_reconFrame;      // depath map data
		sofa::Data<int> d_reconPointNum;     // depth map�� �����ϴ� ������ ����
		sofa::Data<bool> d_deformableRegi;   // if true, do deformable registration
		sofa::Data<bool> d_visualizeDepth;   // if true, visualize current depth map
		sofa::Data<bool> d_visualizeForce;   // if true, visualize current external forces

	protected:
		// ICP�� ����� ������ MechanicalObject
		sofa::core::objectmodel::SingleLink<DGConstraintForceField, sofa::core::behavior::MechanicalState<Vec3Types>,
			sofa::core::objectmodel::BaseLink::FLAG_STOREPATH|sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK> mstateParent;

		//// ICP�� �ʿ��� �����͸� ������ MechanicalObject
		//sofa::core::objectmodel::SingleLink<DGConstraintForceField, sofa::core::behavior::MechanicalState<Vec3Types>,
		//	sofa::core::objectmodel::BaseLink::FLAG_STOREPATH | sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK> mObjectICPTarget;

		// Timer ������ �ʿ��� �Լ� �� ������
		bool m_hasScheduler{ false };  /// If this grabber is controlled by a scheduler
		virtual void handleEvent(sofa::core::objectmodel::Event* e) override;

		// initial registration ���θ� Ȯ���ϱ� ���� ����
		bool m_initialRegistration{ false };

		// realSense���� �߼۵� depth map data
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