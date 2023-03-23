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

#include <opencv2/opencv.hpp>

#include <mutex>
#include <thread>
#include <string>

// OTS ���� ���
#include <DtTracker/Tracker.h>
#include <DtCameraInterface/BaslerCamera.h>
#include <DtCameraInterface/StereoCamera.h>
#include <DtBlobDetection/BlobDetection.h>

// Realsense ���� ���
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

  //tracker ���� sofa-gui ����
  sofa::Data<bool> d_pauseTracker;                     // Toggle tracker play / pause
  sofa::Data<sofa::defaulttype::Mat4x4d> d_cameraPos;  // tracker to camera pos
  sofa::Data<double> d_cameraPosErr;                   // tracker to camera pos error
  sofa::Data<sofa::defaulttype::Mat4x4d> d_patientPos; // tracker to patient pos
  sofa::Data<double> d_patientPosErr;                  // tracker to patient pos error
  sofa::Data<sofa::defaulttype::Mat4x4d> d_boardPos;   // tracker to board pos
  sofa::Data<double> d_boardPosErr;                    // tracker to board pos error
  sofa::Data<sofa::defaulttype::Mat4x4d> d_toolPos;    // tracker to tool pos
  sofa::Data<double> d_toolPosErr;                     // tracker to tool pos error

  // ���� frame���� ���ǿ� �����ϴ� recon point ����
  sofa::Data<int> d_reconPointNum;
  sofa::Data<bool> d_deformableRegi;  // deformable registration ���� ����
  sofa::Data<bool> d_visualizeDepth;  // depth map�� �����ϴ� point �ð�ȭ ����

 public:
  VideoGrabber();
  virtual ~VideoGrabber() override;

  void init() override;      // class �ʱ�ȭ�� ����ϸ� ��
  void doUpdate() override;  // SOFA framework �ȿ��� ���Ǵ� Ÿ�̸�
  void reinit() override;
  void cleanup() override;

 private:
  int m_camIdx;            // ����ϴ� camera�� �ε��� (cv camera ����� ���� �ʿ�)
  bool m_paused;           // camera�� ������Ʈ�� �Ͻ� �����ϴ� Ʈ����
  bool m_pauseTracker;     // marker tracking�� �Ͻ� �����ϴ� Ʈ����
  std::mutex m;

  // realsense ���� class ����
  rs2::frameset frameset;   // Realsense�� ���� frame�� �޾ƿ��� ����
  rs2::colorizer colorize;  // ����� depthmap�� ä���� �� ����ϴ� �Լ�
  rs2::align m_RSalign;     // depth ������ RGB �̹����� ���� align��
  rs2::pipeline m_RSpipe;   // Realsense pipeline ����
  rs2::config m_RSconfig;   // Realsense ���� ����
  rs2::context m_ctx;       // ���� ����� Realsense�� ��� ����
  rs2::pointcloud m_pc;     // depth map���� ������ 3D point cloud
  rs2::points m_points;     // point cloud���� point�� ��ġ ������ ���� �и��� ��
  std::string stSerial;     // ���� ����� Realsense�� �ø��� ��ȣ

  cv::Mat image;
  cv::Mat depth_mat;
  cv::Mat reconPoints;

 public:
  void PausedChanged();
  void PauseTrackerChanged();
  void CamIdxChanged();

  void grabFrame();     // ���� frame�� camera���� �д� �Լ�
  sofa::defaulttype::Mat4x4d armaToSofaMat(const arma::mat& mat);  // arma matrix�� sofa 4*4 matrix�� ��ȯ
};

}  // namespace video
}  // namespace acquisition
}  // namespace sofacv

#endif  // SOFACV_ACQUISITION_VIDEOGRABBER_H
