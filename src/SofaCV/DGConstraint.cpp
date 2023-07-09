#include "DGConstraint.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/gl/DrawToolGL.h>

#include <sofa/gl/RAII.h>
//#include <sofa/helper/system/gl.h>
#include <sofa/gl/gl.h>
#include <sofa/simulation/AnimateBeginEvent.h>

// draw 안에 변수로 들어가는 visualparams를 사용하려면 이 해더를 include 해야한다.
#include <sofa/core/visual/VisualParams.h>

// ICP를 위한 모듈을 읽어온다
#include "ICP/icpPointToPoint.h"

namespace sofacv
{
    SOFA_DECL_CLASS(DGConstraintForceField)

    int DGConstraintForceFieldClass = sofa::core::RegisterObject(
                                        "debug component to project images in OpenGL using "
                                        "the a projection matrix")
                                        .add<DGConstraintForceField>();

    DGConstraintForceField::DGConstraintForceField(sofa::core::behavior::MechanicalState<Vec3Types>* mParent) :
        d_reconFrame(initData(&d_reconFrame, "3DPoints", "3D points from realsense", false, true)),
        d_reconPointNum(initData(&d_reconPointNum, 0, "reconPointNum", "number of current 3D cloud points")),
        d_depthModelSimilarity(initData(&d_depthModelSimilarity, 0, "depthModelSimilarity", "Num of close points bet. depthmap and 3D surface model")),
        d_deformableRegi(initData(&d_deformableRegi, false, "d_deformableRegi", "triggering deformable registration")),
        d_visualizeDepth(initData(&d_visualizeDepth, false, "d_visualizeDepth", "triggering visualization of depth map")),
        d_visualizeForce(initData(&d_visualizeForce, false, "d_visualizeForce", "triggering visualization of external forces")),

        mstateParent(initLink("mtateParent", "MechanicalState which move using the result of ICP"), mParent),
        //mObjectICPTarget(initLink("ICPTargetLink", "MechanicalState for ICP (target)"), mParent),
        mObjectDrill(initLink("DrillguidetLink", "MechanicalState for drilguide"), mParent),
        m_hasScheduler(false),
        m_initialRegistration(false),
        dilDirection1(0), dilDirection2(0), dilDirection3(0), dilPosition1(0), dilPosition2(0), dilPosition3(0)
    {
        f_listening.setValue(true);
    }

    DGConstraintForceField::~DGConstraintForceField() {}

    // 초기화 위한 init 함수 정의
    void DGConstraintForceField::init()
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

        // depth map data를 받을 수 있도록 함
        addInput(&d_reconFrame);
        addInput(&d_reconPointNum);
        addInput(&d_deformableRegi);
        addInput(&d_visualizeDepth);

        update();
    }

    void DGConstraintForceField::handleEvent(sofa::core::objectmodel::Event* e)
    {
        if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
        {
            if (!m_hasScheduler)
                update();
            return;
        }
        ImplicitDataEngine::handleEvent(e);
    }

    void DGConstraintForceField::useScheduler(bool hasScheduler)
    {
        m_hasScheduler = hasScheduler;
    }

    // 시뮬레이션이 진행될 때마다 호출되는 타이머 (내 알고리즘을 이곳에 구현하면 된다.)
    void DGConstraintForceField::doUpdate()
    {
        sofa::helper::AdvancedTimer::stepBegin("ConstraintForceField");
        
        // registration 관련 기능
        if (d_deformableRegi.getValue())
        {
            // 실시간 depth map을 받아오는 부분
            realSenseData = d_reconFrame.getValue();

            // knee surface를 구성하는 각 점의 position (ICP의 target으로 사용됨, Surface)
            const VecCoord& ICPTargetPosition = this->mstate->read(sofa::core::ConstVecCoordId::position())->getValue();

            long pointNum = d_reconPointNum.getValue();        // depth map을 구성하는 점 개수
            long modelSize = (long)ICPTargetPosition.size();   // knee surface 모델을 구성하는 점 개수

            // ICP에는 5개 이상의 점이 필요함. depth map을 구성하는 점 수가 이보다 적다면 에러 발생
            // 프로그램이 막 시작한 단계에서는 ICP를 건너 뛰어야 올바르게 동작합
            if (pointNum < 5 || m_programInitialization == false)
            {
                std::cout << "depth map point is less than 5" << std::endl;
                d_visualizeForce.setValue(false);
                m_programInitialization = true;
                return;
            }
            else
            {
                d_visualizeForce.setValue(true);
            }

            // ICP가 수행되는 두 point cloud 'list'
            double* M = (double*)calloc((unsigned long long)3 * pointNum, sizeof(double));   // 동적할당, 8바이트 값으로 캐스팅 수행
            double* T = (double*)calloc((unsigned long long)3 * modelSize, sizeof(double));  // 동적할당, 8바이트 값으로 캐스팅 수행

            // realsense data를 list에 넣는다
            for (int dataInput = 0; dataInput < pointNum; ++dataInput)
            {
                cv::Vec3d point = realSenseData.at<cv::Vec3d>(cv::Point(dataInput, 0));
                M[dataInput * 3 + 0] = point[0];
                M[dataInput * 3 + 1] = point[1];
                M[dataInput * 3 + 2] = point[2];
            }

            // surface data를 list에 넣는다
            double zMean = 0;  // z 축에 대한 평균 값
            for (int mechanicalInput = 0; mechanicalInput < modelSize; ++mechanicalInput)
            {
                Real xTest = 0.0, yTest = 0.0, zTest = 0.0;
                DataTypes::get(xTest, yTest, zTest, ICPTargetPosition[mechanicalInput]);
                T[mechanicalInput * 3 + 0] = xTest;
                T[mechanicalInput * 3 + 1] = yTest;
                T[mechanicalInput * 3 + 2] = zTest;
                zMean += zTest;
            }
            zMean /= modelSize;

            Matrix R = Matrix::eye(3);
            Matrix t(3, 1);

            // run point-to-point ICP
            IcpPointToPoint icp(M, pointNum, 3);

            if (!m_initialRegistration)  // 초기정합이 필요한 경우
            {
                icp.fit(T, modelSize, R, t, -1);  // (-1 = no outlier threshold)
                m_initialRegistration = true;
            }
            else                        // 초기정합이 완료 된 후
            {
                icp.fit(T, modelSize, R, t, 120);  // 120 mm 거리 안의 점들에 대해서만 ICP 적용
            }

            // depth map에 대해 지정된 범위 안에 존재하는 knee surface 점들 
            // 범위를 0.1 ~ 20 으로 한번 변경해 볼가?
            std::vector<int32_t> sourceInliers = icp.getRanges(T, modelSize, R, t, 0.1, 10);
            // 각 knee surface 점에 대해 가장 가까운 depth map 위 점
            std::vector<int32_t> targetNearestPointsIdx = icp.getNearestIdxs(T, modelSize, R, t);

            // 각 점에 작용되는 외력을 저장하는 vector 선언
            sofa::helper::vector<sofa::type::Vec3d> totalForceVec;
            sofa::core::topology::BaseMeshTopology::VerticesAroundVertex totalIndices;

            // knee surface의 각 점에 적용되는 힘을 계산하기 위한 밑준비
            std::vector<float> sourcePoint(3);
            std::vector<float> targetPoint(3);
            // extract matrix and translation vector
            double r00 = R.val[0][0]; double r01 = R.val[0][1]; double r02 = R.val[0][2];
            double r10 = R.val[1][0]; double r11 = R.val[1][1]; double r12 = R.val[1][2];
            double r20 = R.val[2][0]; double r21 = R.val[2][1]; double r22 = R.val[2][2];
            double t0 = t.val[0][0]; double t1 = t.val[1][0]; double t2 = t.val[2][0];
            
            // source의 각 점에 적용되는 힘을 계산한다.
            std::cout << "number of close points: " << sourceInliers.size() << "/" << modelSize << std::endl;
            d_depthModelSimilarity.setValue(sourceInliers.size());  // 외부 class로 값을 넘겨주기 위한 코드

            for (int i = 0; i< sourceInliers.size(); i++)
            {
                int inlierNum = sourceInliers[i];
                sourcePoint[0] = (float)(r00 * T[inlierNum * 3 + 0] + r01 * T[inlierNum * 3 + 1] + r02 * T[inlierNum * 3 + 2] + t0);
                sourcePoint[1] = (float)(r10 * T[inlierNum * 3 + 0] + r11 * T[inlierNum * 3 + 1] + r12 * T[inlierNum * 3 + 2] + t1);
                sourcePoint[2] = (float)(r20 * T[inlierNum * 3 + 0] + r21 * T[inlierNum * 3 + 1] + r22 * T[inlierNum * 3 + 2] + t2);

                if (sourcePoint[2] < zMean)  // 모델의 앞쪽 반에만 알고리즘이 적용될 수 있도록 함
                {
                    targetPoint[0] = (float)M[targetNearestPointsIdx[inlierNum] * 3 + 0];
                    targetPoint[1] = (float)M[targetNearestPointsIdx[inlierNum] * 3 + 1];
                    targetPoint[2] = (float)M[targetNearestPointsIdx[inlierNum] * 3 + 2];

                    // source와 target의 nearest point 사이 거리를 힘으로써 적용한다 7 100 7 일 때 80
                    Real xForce = ((Real)targetPoint[0] - sourcePoint[0]) * 100;  // 나중에 특정 weight를 곱하면크기를 비례해서 키울 수 있음
                    Real yForce = ((Real)targetPoint[1] - sourcePoint[1]) * 100;  // Real형으로 type casting 필요
                    Real zForce = ((Real)targetPoint[2] - sourcePoint[2]) * 100;

                    totalForceVec.push_back({ xForce, yForce, zForce });
                    totalIndices.push_back(inlierNum);
                }
            }

            d_indices.setValue(totalIndices);
            d_forces.setValue(totalForceVec);

            // free memory
            free(M);
            free(T);

            // ICP 결과 이용하여 모델 (FEM+Collision+Visual 모두 동시) 움직이는 부분          
            sofa::type::Vec<3U, Real> Iox = sofa::type::Vec<3U, Real>(R.val[0][0], R.val[1][0], R.val[2][0]);
            sofa::type::Vec<3U, Real> Ioy = sofa::type::Vec<3U, Real>(R.val[0][1], R.val[1][1], R.val[2][1]);
            sofa::type::Vec<3U, Real> Ioz = sofa::type::Vec<3U, Real>(R.val[0][2], R.val[1][2], R.val[2][2]);
            sofa::defaulttype::Quaternion q = sofa::helper::Quater<SReal>::createQuaterFromFrame(Iox, Ioy, Ioz);
            sofa::helper::WriteAccessor<sofa::core::objectmodel::Data<VecCoord>> x_wA = this->mstateParent->write(sofa::core::VecCoordId::position());

            for (unsigned int i = 0; i < x_wA.size(); i++)
            {
                sofa::defaulttype::Vec<3, Real> pos;
                DataTypes::get(pos[0], pos[1], pos[2], x_wA[i]);

                // rotation 적용
                sofa::defaulttype::Vec<3, Real> newposition = q.rotate(pos);
                // translation 적용
                DataTypes::set(x_wA[i], newposition[0] + t.val[0][0], newposition[1] + t.val[1][0] - 2, newposition[2] + t.val[2][0]);
            }

            // 무릎 surface model이 굽혀졌다가 다시 펴지는 문제를 해결해야 함
            // 시뮬레이션을 멈추는 방법을 찾거나, 힘의 평형점을 찾아서 계속 힘을 줘야 함 (어떻게 해결해야 할가...)
        }
        sofa::helper::AdvancedTimer::stepEnd("ConstraintForceField");

     }


    // SOFA framework에서 모델 시각화를 위해 사용 (SOFA 내부 타이머가 draw 함수를 자동으로 실행)
    void DGConstraintForceField::draw(const sofa::core::visual::VisualParams* vparams)
    {     
        // display option 설정 (필요함)
        vparams->drawTool()->saveLastState();
        
        // depth map visualizatiopn을 수행한다
        if (d_visualizeDepth.getValue())
        {
            // 실시간 depth map을 받아오는 부분
            realSenseData = d_reconFrame.getValue();

            int iterNum = realSenseData.size().width;
            std::vector<sofa::defaulttype::Vector3> points;

            long i = 0;
            for (i; i < iterNum; ++i)
            {
                cv::Vec3d point = realSenseData.at<cv::Vec3d>(cv::Point(i, 0));
                points.push_back(sofa::defaulttype::Vector3(point[0], point[1], point[2]));
            }
            vparams->drawTool()->drawSpheres(points, 1, sofa::helper::types::RGBAColor(1.0f, 0.35f, 0.35f, 1.0f));
        }

        // 모델에 가해지는 힘의 크기와 방향을 시각화하는 부분
        if (d_visualizeForce.getValue())
        {
            // arrow size가 0인 경우 바로 return 수행
            const SReal aSC = d_showArrowSize.getValue();  // 화살표 크기
            if ((!vparams->displayFlags().getShowForceFields() && (aSC == 0.0)) || (aSC < 0.0)) return;

            // 실시간으로 현재 Mechanical Model의 각 점 좌표를 받아오는 부분 (힘의 방향 그릴 때 사용, Femur)
            const VecCoord& mstatePosition = this->mstate->read(sofa::core::ConstVecCoordId::position())->getValue();
            
            const VecIndex& indices = d_indices.getValue();
            const VecDeriv& f = d_forces.getValue();

            vparams->drawTool()->setLightingEnabled(true);

            // drilling position을 시각화하는 부분 (임시로 만든 기능, 추후 개선 필요함)
            //sofa::defaulttype::Vector3 p1(dilPosition1, dilPosition2, dilPosition3);
            //sofa::defaulttype::Vector3 p2(30 * dilDirection1 + dilPosition1, 30 * dilDirection2 + dilPosition2, 30 * dilDirection3 + dilPosition3);

            //float norm = static_cast<float>((p2 - p1).norm());
            //vparams->drawTool()->drawArrow(p2, p1, norm / 20.0f, sofa::helper::types::RGBAColor(0, 1.0f, 0, 1.0f));
            // drilling position을 시각화하는 부분 끝 (임시로 만든 기능)

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

                sofa::defaulttype::Vector3 p1(xx, xy, xz);  // 화살표 시작점
                        
                sofa::defaulttype::Vector3 p2(aSC * fx + xx, aSC * fy + xy, aSC * fz + xz);  //화살표 끝점

                float norm = static_cast<float>((p2 - p1).norm());  // 화살표 굵기

                // 화살표를 그린다
                vparams->drawTool()->drawArrow(p1, p2, norm / 20.0f, sofa::helper::types::RGBAColor(1.0f, 0.35f, 0.35f, 1.0f));
            }
        }
        vparams->drawTool()->restoreLastState();
    }
}