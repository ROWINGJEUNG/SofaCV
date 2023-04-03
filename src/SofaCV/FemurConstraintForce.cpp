#include "FemurConstraintForce.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/gl/DrawToolGL.h>

#include <sofa/gl/RAII.h>
//#include <sofa/helper/system/gl.h>
#include <sofa/gl/gl.h>
#include <sofa/simulation/AnimateBeginEvent.h>

// draw �ȿ� ������ ���� visualparams�� ����Ϸ��� �� �ش��� include �ؾ��Ѵ�.
#include <sofa/core/visual/VisualParams.h>

// ICP�� ���� ����� �о�´�
#include "ICP/icpPointToPoint.h"
#include "PCA_jdg.h"

namespace sofacv
{
    SOFA_DECL_CLASS(FemurConstraintForce)

    int FemurConstraintForceClass = sofa::core::RegisterObject(
                                    "debug component to project images in OpenGL using "
                                    "the a projection matrix")
                                    .add<FemurConstraintForce>();

    FemurConstraintForce::FemurConstraintForce(sofa::core::behavior::MechanicalState<Vec3Types>* mParent) :
        d_deformableRegi(initData(&d_deformableRegi, false, "d_deformableRegi", "triggering deformable registration")),
        d_visualizeForce(initData(&d_visualizeForce, true, "d_visualizeForce", "triggering visualization of external forces")),
        d_depthModelSimilarity(initData(&d_depthModelSimilarity, 0, "depthModelSimilarity", "Num of close points bet. depthmap and 3D surface model")),
        m_hasScheduler(false)
    {
        f_listening.setValue(true);
    }

    FemurConstraintForce::~FemurConstraintForce() {}

    // �ʱ�ȭ ���� init �Լ� ����
    void FemurConstraintForce::init()
    {
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);

        if (l_topology.empty())
        {
            msg_info() << "link to Topology container should be set to ensure right behavior. First Topology found in current context will be used.";
            l_topology.set(this->getContext()->getMeshTopologyLink());
        }

        // temprory pointer to topology
        sofa::core::topology::BaseMeshTopology* _topology = l_topology.get();

        if (_topology)
        {
            msg_info() << "Topology path used: '" << l_topology.getLinkedPath() << "'";

            // Initialize functions and parameters for topology data and handler
            d_indices.createTopologyHandler(_topology);
            d_indices.registerTopologicalData();

            m_systemSize = _topology->getNbPoints();
        }
        else
        {
            msg_info() << "No topology component found at path: " << l_topology.getLinkedPath() << ", nor in current context: " << this->getContext()->name;
            sofa::core::behavior::BaseMechanicalState* state = this->getContext()->getMechanicalState();
            m_systemSize = state->getSize();
        }

        const VecIndex& indices = d_indices.getValue();
        auto indicesSize = indices.size();

        if (d_indices.isSet() && indicesSize != 0)
        {
            // check size of vector indices
            if (indicesSize > m_systemSize)
            {
                msg_error() << "Size mismatch: indices > system size";
                this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
                return;
            }
            // check each indice of the vector
            for (sofa::Size i = 0; i < indicesSize; i++)
            {
                if (indices[i] > m_systemSize)
                {
                    msg_error() << "Indices incorrect: indice[" << i << "] = " << indices[i] << " exceeds system size";
                    this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
                    return;
                }
            }
        }
        else
        {
            // initialize with all indices
            VecIndex& indicesEdit = *d_indices.beginEdit();
            indicesEdit.clear();
            indicesEdit.resize(m_systemSize);
            std::iota(std::begin(indicesEdit), std::end(indicesEdit), 0);
            d_indices.endEdit();
        }

        if (d_forces.isSet())
        {
            const VecDeriv& forces = d_forces.getValue();
            if (checkForces(forces))
            {
                computeForceFromForceVector();
            }
            else
            {
                msg_error() << " Invalid given vector forces";
                this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
                return;
            }
            msg_info() << "Input vector forces is used for initialization";
        }
        else if (d_force.isSet())
        {
            const Deriv& force = d_force.getValue();
            if (checkForce(force))
            {
                computeForceFromSingleForce();
            }
            else
            {
                msg_error() << " Invalid given force";
                this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
                return;
            }
            msg_info() << "Input force is used for initialization";
        }
        else if (d_totalForce.isSet())
        {
            const Deriv& totalForce = d_totalForce.getValue();
            if (checkForce(totalForce))
            {
                computeForceFromTotalForce();
            }
            else
            {
                msg_error() << " Invalid given totalForce";
                this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
                return;
            }
            msg_info() << "Input totalForce is used for initialization";
        }

        // init from ForceField
        Inherit::init();

        // add to tracker
        this->trackInternalData(d_indices);
        this->trackInternalData(d_forces);
        this->trackInternalData(d_force);
        this->trackInternalData(d_totalForce);

        // if all init passes, component is valid
        this->d_componentState.setValue(sofa::core::objectmodel::ComponentState::Valid);

        update();
    }

    void FemurConstraintForce::handleEvent(sofa::core::objectmodel::Event* e)
    {
        if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
        {
            if (!m_hasScheduler)
                update();
            return;
        }
        ImplicitDataEngine::handleEvent(e);
    }

    void FemurConstraintForce::useScheduler(bool hasScheduler)
    {
        m_hasScheduler = hasScheduler;
    }

    // �ùķ��̼��� ����� ������ ȣ��Ǵ� Ÿ�̸� (�� �˰����� �̰��� �����ϸ� �ȴ�.)
    void FemurConstraintForce::doUpdate()
    {
        sofa::helper::AdvancedTimer::stepBegin("FemurConstraintForce");

        // registration ���� ���
        int forceTrigger = d_depthModelSimilarity.getValue();

        if (d_deformableRegi.getValue())
        {
            if (forceTrigger > 140)   // �̹� ����� ������ ���
            {
                sofa::helper::vector<sofa::type::Vec3d> totalForceVec;
                sofa::core::topology::BaseMeshTopology::VerticesAroundVertex totalIndices;

                totalForceVec.push_back({ 0, 0, 0 });
                totalIndices.push_back(0);

                d_indices.setValue(totalIndices);
                d_forces.setValue(totalForceVec);
            }
            else  // ������ ��� �ʿ��� ��
            {
                // knee surface�� �����ϴ� �� ���� position (ICP�� target���� ����, Surface)
                const VecCoord& ICPTargetPosition = this->mstate->read(sofa::core::ConstVecCoordId::position())->getValue();

                long modelSize = (long)ICPTargetPosition.size();   // knee surface ���� �����ϴ� �� ����
                std::vector<std::vector<double>> PCATargetMat;
                std::vector<double> PCATargetMatX;
                std::vector<double> PCATargetMatY;
                std::vector<double> PCATargetMatZ;

                for (int mechanicalInput = 0; mechanicalInput < modelSize; ++mechanicalInput)
                {
                    Real xTest = 0.0, yTest = 0.0, zTest = 0.0;
                    DataTypes::get(xTest, yTest, zTest, ICPTargetPosition[mechanicalInput]);
                    PCATargetMatX.push_back(xTest);
                    PCATargetMatY.push_back(yTest);
                    PCATargetMatZ.push_back(zTest);
                }
                PCATargetMat.push_back(PCATargetMatX);
                PCATargetMat.push_back(PCATargetMatY);
                PCATargetMat.push_back(PCATargetMatZ);

                std::vector<double> w1;
                std::vector<double> w2;
                std::vector<double> w3;
                double evalue1, evalue2, evalue3;
                pcapc12(PCATargetMat, &w1, &w2, &w3, evalue1, evalue2, evalue3);

                Real PCAbase = sqrt(w3[0] * w3[0] + w3[1] * w3[1] + w3[2] * w3[2]);
                Real forceX = w3[0] / PCAbase;
                Real forceY = w3[1] / PCAbase;
                Real forceZ = w3[2] / PCAbase;

                // �� ���� �ۿ�Ǵ� �ܷ��� �����ϴ� vector ����
                sofa::helper::vector<sofa::type::Vec3d> totalForceVec;
                sofa::core::topology::BaseMeshTopology::VerticesAroundVertex totalIndices;

                if (forceX > 0)  // PCA vector�� ������ �׻� �����ϰ� �����ϱ� ���� ���
                    forceWeightPercentage1 *= -1;

                Real xForce = forceX * 15 * forceWeightPercentage1 * forceWeight;
                Real yForce = forceY * 15 * forceWeightPercentage1 * forceWeight;
                Real zForce = forceZ * 15 * forceWeightPercentage1 * forceWeight;

                Real xForce2 = -forceX * 60 * forceWeightPercentage1 * forceWeight;
                Real yForce2 = -forceY * 60 * forceWeightPercentage1 * forceWeight;
                Real zForce2 = -forceZ * 60 * forceWeightPercentage1 * forceWeight;

                if (forceX > 0)  // PCA vector�� ������ �׻� �����ϰ� �����ϱ� ���� ���
                    forceWeightPercentage1 *= -1;

                totalForceVec.push_back({ xForce, yForce, zForce });
                totalForceVec.push_back({ xForce2, yForce2, zForce2 });
                totalIndices.push_back(0);
                totalIndices.push_back(46);

                d_indices.setValue(totalIndices);
                d_forces.setValue(totalForceVec);
            }
        }
        sofa::helper::AdvancedTimer::stepEnd("FemurConstraintForce");

    }


    // SOFA framework���� �� �ð�ȭ�� ���� ��� (SOFA ���� Ÿ�̸Ӱ� draw �Լ��� �ڵ����� ����)
    void FemurConstraintForce::draw(const sofa::core::visual::VisualParams* vparams)
    {
        // display option ���� (�ʿ���)
        vparams->drawTool()->saveLastState();

        // �𵨿� �������� ���� ũ��� ������ �ð�ȭ�ϴ� �κ�
        if (d_visualizeForce.getValue())
        {
            // arrow size�� 0�� ��� �ٷ� return ����
            const SReal aSC = d_showArrowSize.getValue();  // ȭ��ǥ ũ��
            if ((!vparams->displayFlags().getShowForceFields() && (aSC == 0.0)) || (aSC < 0.0)) return;

            // �ǽð����� ���� Mechanical Model�� �� �� ��ǥ�� �޾ƿ��� �κ� (���� ���� �׸� �� ���, Femur)
            const VecCoord& mstatePosition = this->mstate->read(sofa::core::ConstVecCoordId::position())->getValue();

            const VecIndex& indices = d_indices.getValue();
            const VecDeriv& f = d_forces.getValue();

            vparams->drawTool()->setLightingEnabled(true);

            for (unsigned int i = 0; i < indices.size(); i++)
            {
                Real xx = 0.0, xy = 0.0, xz = 0.0, fx = 0.0, fy = 0.0, fz = 0.0;

                if (!d_indexFromEnd.getValue())
                {
                    if (indices[i] < mstatePosition.size())
                    {
                        DataTypes::get(xx, xy, xz, mstatePosition[indices[i]]);
                    }
                    else
                    {
                        msg_error() << "Draw: error in indices values";
                    }
                }
                else
                {
                    if ((mstatePosition.size() - indices[i] - 1) < mstatePosition.size() && (mstatePosition.size() - indices[i] - 1) >= 0)
                    {
                        DataTypes::get(xx, xy, xz, mstatePosition[mstatePosition.size() - indices[i] - 1]);
                    }
                    else
                    {
                        msg_error() << "Draw: error in indices values";
                    }
                }

                DataTypes::get(fx, fy, fz, f[i]);

                sofa::defaulttype::Vector3 p1(xx, xy, xz);  // ȭ��ǥ ������

                sofa::defaulttype::Vector3 p2(aSC * fx + xx, aSC * fy + xy, aSC * fz + xz);  //ȭ��ǥ ����

                float norm = static_cast<float>((p2 - p1).norm());  // ȭ��ǥ ����

                // ȭ��ǥ�� �׸���
                vparams->drawTool()->drawArrow(p1, p2, norm / 20.0f, sofa::helper::types::RGBAColor(1.0f, 0.35f, 0.35f, 1.0f));
            }
        }
        vparams->drawTool()->restoreLastState();
    }
}