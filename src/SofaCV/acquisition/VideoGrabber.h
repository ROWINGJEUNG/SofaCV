/******************************************************************************
 *       SOFAOR, SOFA plugin for the Operating Room, development version       *
 *                        (c) 2017 INRIA, MIMESIS Team                         *
 *                                                                             *
 * This program is a free software; you can redistribute it and/or modify it   *
 * under the terms of the GNU Lesser General Public License as published by    *
 * the Free Software Foundation; either version 1.0 of the License, or (at     *
 * your option) any later version.                                             *
 *                                                                             *
 * This program is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details.                                                           *
 *                                                                             *
 * You should have received a copy of the GNU Lesser General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.        *
 *******************************************************************************
 * Authors: Bruno Marques and external contributors (see Authors.txt)          *
 *                                                                             *
 * Contact information: contact-mimesis@inria.fr                               *
 ******************************************************************************/

#ifndef SOFACV_ACQUISITION_VIDEOGRABBER_H
#define SOFACV_ACQUISITION_VIDEOGRABBER_H

#include "SofaCV/ImplicitDataEngine.h"
#include "SofaCV/SofaCVPlugin.h"

#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include "BaseFrameGrabber.h"

// data type 정의하기 위한 헤더 가져옴
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
using sofa::defaulttype::Vec3Types;
using Coord3 = sofa::defaulttype::Vector3;
using VecCoord3 = sofa::helper::vector<Coord3>;

// 기타 필요 SOFA 헤더 가져옴
#include <SofaBaseVisual/VisualModelImpl.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/Quater.h>
#include <sofa/helper/AdvancedTimer.h>
#include <SofaBoundaryCondition/FixedConstraint.h>

#include <opencv2/opencv.hpp>

#include <mutex>
#include <thread>
#include <string>

// OTS 관련 헤더
#include <DtTracker/Tracker.h>
#include <DtCameraInterface/BaslerCamera.h>
#include <DtCameraInterface/StereoCamera.h>
#include <DtBlobDetection/BlobDetection.h>

// Realsense 관련 헤더
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

namespace sofacv
{
namespace acquisition
{
/**
 * \brief Video module
 */
namespace video
{
class SOFA_SOFACV_API VideoGrabber : public BaseFrameGrabber
{
 public:
  SOFA_CLASS(VideoGrabber, BaseFrameGrabber);

 public:
  sofa::Data<int> d_camIdx;     // [INTPUT] Index of the capturing device to open
  sofa::Data<bool> d_paused;    // [INTPUT] Toggle play / pause
  sofa::Data<sofa::defaulttype::Vec2i> d_resolution;  // [property] To set specific camera grabbing resolution

  //tracker 관련 sofa-gui 변수
  sofa::Data<bool> d_pauseTracker;                     // Toggle tracker play / pause
  sofa::Data<sofa::defaulttype::Mat4x4d> d_cameraPos;  // tracker to camera pos
  sofa::Data<double> d_cameraPosErr;                   // tracker to camera pos error
  sofa::Data<sofa::defaulttype::Mat4x4d> d_patientPos; // tracker to patient pos
  sofa::Data<double> d_patientPosErr;                  // tracker to patient pos error
  sofa::Data<sofa::defaulttype::Mat4x4d> d_boardPos;   // tracker to board pos
  sofa::Data<double> d_boardPosErr;                    // tracker to board pos error
  sofa::Data<sofa::defaulttype::Mat4x4d> d_toolPos;    // tracker to tool pos
  sofa::Data<double> d_toolPosErr;                     // tracker to tool pos error

  // 현재 frame에서 조건에 부합하는 recon point 개수
  sofa::Data<int> d_reconPointNum;
  sofa::Data<bool> d_deformableRegi;  // deformable registration 진행 여부
  sofa::Data<bool> d_visualizeDepth;  // depth map을 구성하는 point 시각화 여부

  // virtual depth map을 받아오기 위한 변수
  sofa::Data<VecCoord3> d_vDepthMap;

 public:
  VideoGrabber(sofa::core::behavior::MechanicalState<Vec3Types>* mParent = nullptr);
  virtual ~VideoGrabber() override;

  void init() override;      // class 초기화에 사용하면 됨
  void doUpdate() override;  // SOFA framework 안에서 사용되는 타이머
  void reinit() override;
  void cleanup() override;

  void PausedChanged();
  void PauseTrackerChanged();
  void CamIdxChanged();

  void grabFrame();     // 다음 frame을 camera에서 읽는 함수
  sofa::defaulttype::Mat4x4d armaToSofaMat(const arma::mat& mat);  // arma matrix를 sofa 4*4 matrix로 변환

  // 가상의 depth map을 받아오기 위한 링크 설정
  //void setVirtualDepthLink(const std::string& o) { mVirtualDepthState.setPath(o); }

 private:
  int m_camIdx;            // 사용하는 camera의 인덱스 (cv camera 사용할 때만 필요)
  bool m_paused;           // camera의 업데이트를 일시 중지하는 트리거
  bool m_pauseTracker;     // marker tracking을 일시 중지하는 트리거
  std::mutex m;

  // realsense 관련 class 변수
  rs2::frameset frameset;   // Realsense의 다음 frame을 받아오는 변수
  rs2::colorizer colorize;  // 얻어진 depthmap을 채색할 때 사용하는 함수
  rs2::align m_RSalign;     // depth 정보를 RGB 이미지에 대해 align함
  rs2::pipeline m_RSpipe;   // Realsense pipeline 정의
  rs2::config m_RSconfig;   // Realsense 관련 설정
  rs2::context m_ctx;       // 현재 연결된 Realsense의 기기 정보
  rs2::pointcloud m_pc;     // depth map에서 복원된 3D point cloud
  rs2::points m_points;     // point cloud에서 point의 위치 정보만 따로 분리한 것
  std::string stSerial;     // 현재 연결되 Realsense의 시리얼 번호

  cv::Mat image;
  cv::Mat reconPoints;

//protected:
//	sofa::core::objectmodel::SingleLink<VideoGrabber, sofa::core::behavior::MechanicalState<Vec3Types>,
//		sofa::core::objectmodel::BaseLink::FLAG_STOREPATH | sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK> mVirtualDepthState;
};

}  // namespace video
}  // namespace acquisition
}  // namespace sofacv

#endif  // SOFACV_ACQUISITION_VIDEOGRABBER_H
