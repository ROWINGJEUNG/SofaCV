#ifndef SOFACV_FEMURCONSTRAINTFORCE_H
#define SOFACV_FEMURCONSTRAINTFORCE_H

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
	class SOFA_SOFACV_API FemurConstraintForce : public sofa::component::forcefield::ConstantForceField<Vec3Types>, virtual public ImplicitDataEngine
	{
	public:
		SOFA_CLASS(FemurConstraintForce, ConstantForceField<Vec3Types>);

		FemurConstraintForce(sofa::core::behavior::MechanicalState<Vec3Types>* mParent = nullptr);           // ������ ����
		virtual ~FemurConstraintForce();  // �Ҹ��� ����

		// Ŭ���� ���� �ʿ� ������ ���� (doUpdate ������ �� �Լ� �ȿ��� �Ѵ�)
		void init() override;

		// SOFA framework�ȿ� �� �ð�ȭ�� ���� ��� (SOFA ���� Ÿ�̸Ӱ� draw �Լ��� �ڵ����� �����Ŵ)
		// �ð��� ���� �𵨿� �������� ���� ������ �����ϱ� ���� ���
		void draw(const sofa::core::visual::VisualParams* vparams) override;

		// ���� �ð����� �����ϴ� timer ����
		void useScheduler(bool);
		void doUpdate() override;

		// �ܷ��� ����Ǿ��� �� ȣ��Ǵ� �Լ�
		void doUpdateInternal() override {}

		sofa::Data<bool> d_deformableRegi;       // if true, do deformable registration
		sofa::Data<bool> d_visualizeForce;       // if true, visualize current external forces
		sofa::Data<int> d_depthModelSimilarity;  // depth map�� 3D model���� inliner �� ���� -> DGConstraint���� �޾ƿ�

	protected:
		double forceWeightPercentage1{ 1.0 };
		double forceWeightPercentage2{ 1.0 };
		double forceWeight{ 100.0 };
		int pre_depthModelSimilarity { 0 };
		// Timer ������ �ʿ��� �Լ� �� ������
		bool m_hasScheduler{ false };  /// If this grabber is controlled by a scheduler
		virtual void handleEvent(sofa::core::objectmodel::Event* e) override;
	};
}
#endif