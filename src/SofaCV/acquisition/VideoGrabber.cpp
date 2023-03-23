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

#include "VideoGrabber.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <omp.h>
#include <SofaCV/ICP/matrix.h>

namespace sofacv
{
namespace acquisition
{
namespace video
{
SOFA_DECL_CLASS(VideoGrabber)

static int VideoGrabberClass =
    sofa::core::RegisterObject("OpenCV-based component reading mono and stereo videos").add<VideoGrabber>();

VideoGrabber::VideoGrabber()
    : BaseFrameGrabber(),
      // �ʱ� ī�޶� in
      d_camIdx(initData(&d_camIdx, 0, "cam_index", "camera device index")),
      d_paused(initData(&d_paused, false, "pause_camera", "if true, stops grabbing")),
      d_resolution(initData(&d_resolution, sofa::defaulttype::Vec2i(1280, 720),
                            "resolution", "grabbing resolution to set")),

    d_pauseTracker(initData(&d_pauseTracker, false, "pause_tracker", "if true, pause tracker")),
    d_cameraPos(initData(&d_cameraPos, "camera_Pos", "tracker to camera pos")),
    d_cameraPosErr(initData(&d_cameraPosErr, 0.0, "camera_Pos_Err", "tracker to camera pos error")),
    d_patientPos(initData(&d_patientPos, "patient_Pos", "tracker to patient pos")),
    d_patientPosErr(initData(&d_patientPosErr, 0.0, "patient_Pos_Err", "tracker to patient pos error")),
    d_boardPos(initData(&d_boardPos, "board_Pos", "tracker to board pos")),
    d_boardPosErr(initData(&d_boardPosErr, 0.0, "board_Pos_Err", "tracker to board pos error")),
    d_toolPos(initData(&d_toolPos, "tool_Pos", "tracker to board pos")),
    d_toolPosErr(initData(&d_toolPosErr, 0.0, "tool_Pos_Err", "tracker to tool pos error")),
    m_RSalign(RS2_STREAM_COLOR),
    d_reconPointNum(initData(&d_reconPointNum, 0, "reconPointNum", "number of current 3D cloud points")),
    d_deformableRegi(initData(&d_deformableRegi, false, "d_deformableRegi", "triggering deformable registration")),
    d_visualizeDepth(initData(&d_visualizeDepth, false, "d_visualizeDepth", "triggering visualization of depth map"))
{
  f_listening.setValue(true);

  this->addAlias(&d_reconPointNum, "reconPointNum");
}

void VideoGrabber::init()
{
  m_camIdx = d_camIdx.getValue();                // m_camIdx �ʱ�ȭ
  m_paused = d_paused.getValue();                // m_paused �ʱ�ȭ
  m_pauseTracker = d_pauseTracker.getValue();    // m_pauseTracker �ʱ�ȭ

  // Realsense �ʱ�ȭ ���� �ڵ�
  for (auto&& dev : m_ctx.query_devices())  // ���� ����� Realsense ����̽� Ư��
  {
      stSerial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      std::cout << "Connected device : " << stSerial << std::endl;
  }
  m_RSconfig.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
  m_RSconfig.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
  m_RSconfig.enable_device(stSerial);
  m_RSpipe.start(m_RSconfig);
  d_fps.setValue(30);  // BaseFrameGrabber�� ���ǵ� d_fps ���� (output)

  sofa::defaulttype::Vec2i dims;
  dims[0] = static_cast<int>(1280);
  dims[1] = static_cast<int>(720);
  d_dimensions.setValue(dims);  // BaseFrameGrabber�� ���ǵ� d_dimensions ���� (output)

  for (int s = 0; s < 10; s++) m_RSpipe.wait_for_frames();  // �ڵ� ���� ���� ������ ����
  // Realsense �ʱ�ȭ ���� �ڵ� ��

  addInput(&d_camIdx);
  addInput(&d_paused);

  addInput(&d_pauseTracker);
  addInput(&d_cameraPos);
  addInput(&d_cameraPosErr);
  addInput(&d_patientPos);
  addInput(&d_patientPosErr);
  addInput(&d_boardPos);
  addInput(&d_boardPosErr);
  addInput(&d_toolPos);
  addInput(&d_toolPosErr);

  addOutput(&d_reconPointNum);
  addOutput(&d_deformableRegi);
  addOutput(&d_visualizeDepth);

  BaseFrameGrabber::init();
  update();
}

// ����� ī�޶󿡼� ���� �������� �޾ƿ´�
void VideoGrabber::grabFrame()
{
  if (m_paused)  // m_paused�� True�� frame update�� �����. ���� �� ����� �������� �ʴ� �� ����
    return;

  frameset = m_RSpipe.wait_for_frames();
  frameset = m_RSalign.process(frameset);

  rs2::depth_frame depth = frameset.get_depth_frame();
  rs2::video_frame color = frameset.get_color_frame();

  image = cv::Mat(cv::Size(1280, 720), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
  cv::cvtColor(image, image, cv::COLOR_RGB2BGR);  // RealSense�� �� �����Ǿ� ����

  colorize.set_option(RS2_OPTION_COLOR_SCHEME, 2);
  rs2::frame bw_depth = depth.apply_filter(colorize);
  //depth_mat = cv::Mat(cv::Size(1280, 720), CV_8UC3, (void*)bw_depth.get_data(), cv::Mat::AUTO_STEP);
  //cv::imshow("testWin", depth_mat);
  //cv::waitKey(10);
 
  // �� point�� ��ġ ������ �а� Mat ��Ŀ� ���� �ִ´� -> ���ο� �ڷ��� ���� �ʿ� ���� ������ ���� ����
  m_pc.map_to(color);
  m_points = m_pc.calculate(depth);
  int pointSize = m_points.size();

  const rs2::vertex* vertices = m_points.get_vertices();
  const rs2::texture_coordinate* Texture_Coord = m_points.get_texture_coordinates();
  const auto New_Texture = reinterpret_cast<const uint8_t*>(color.get_data());  // get texture data
  int width = color.get_width();
  int height = color.get_height();
  int bytesPerPixel = color.get_bytes_per_pixel();
  int strideInBytes = color.get_stride_in_bytes();

  int numPoint = 0;
  reconPoints = cv::Mat::zeros(1, pointSize, CV_64FC3);

//#pragma omp parallel for private(i) default(none) shared(vertices, reconPoints, Texture_Coord, New_Texture) reduction(+:numPoint)
  for (int i = 0; i < pointSize; ++i)
  {
      double _z = double(vertices[i].z) * 1000;

      if (_z > 300.0 && _z < 500.0)  // ���� ���� outlier ����
      {
          // Normals to Texture Coordinates conversion
          int x_value = std::min(std::max(int(Texture_Coord[i].u * width + .5f), 0), width - 1);
          int y_value = std::min(std::max(int(Texture_Coord[i].v * height + .5f), 0), height - 1);

          int bytes = x_value * bytesPerPixel;    // Get # of bytes per pixel
          int strides = y_value * strideInBytes;  // Get line width in bytes
          int Text_Index = (bytes + strides);

          int NT1 = New_Texture[Text_Index];      // RGB color of current point
          int NT2 = New_Texture[Text_Index + 1];  // RGB color of current point
          int NT3 = New_Texture[Text_Index + 2];  // RGB color of current point

          // color�� �������� �ƴ� ��츸 point�� 3D coordinate�� �����Ѵ�.
          if (NT1 > 50 && NT2 > 50 && NT3 > 50)
          {
              double _x = double(vertices[i].x) * 1000;
              double _y = double(vertices[i].y) * 1000;
              reconPoints.at<cv::Vec3d>(cv::Point(numPoint, 0)) = cv::Vec3d(_x, _y, _z);
              numPoint += 1;
          }
      }
  }

  // �̹��� ���� ������Ʈ
  m.lock();
  if (!image.empty())
  {
      d_frame1.cleanDirty();
      image.copyTo(*d_frame1.beginEdit());
      d_frame1.endEdit();

      image.release();
  }

  // 3D recon point ���� ������Ʈ (10,000�� �̻��� 3D point ���� ���� �۵�) 
  if (numPoint>10000)
  {
      int& finalNumPoint = *d_reconPointNum.beginEdit();
      finalNumPoint = numPoint;
      d_reconPointNum.endEdit();

      d_reconFrame.cleanDirty();
      reconPoints(cv::Rect(0, 0, numPoint, 1)).copyTo(*d_reconFrame.beginEdit());
      d_reconFrame.endEdit();

      reconPoints.release();
  }
  m.unlock();

  // image plane�� 3D world coordinate������ ��ġ�� �˱� ���� ���, realsense�� origin ��.
  //rs2::video_stream_profile colorPropile = color.get_profile().as<rs2::video_stream_profile>();
  //auto intrinsicParam = colorPropile.get_intrinsics();
  //double screenSizeX = intrinsicParam.width;
  //double screenSizeY = intrinsicParam.height;

  //double cx = (screenSizeX / 2.) - intrinsicParam.ppx;
  //double cy = (screenSizeY / 2.) - intrinsicParam.ppy;
  //double focalLength = (intrinsicParam.fx + intrinsicParam.fy) / 2.;
  //double planeRatio = screenSizeX / screenSizeY;

  //double focal0 = cx;
  //double focal1 = cy;
  //double focal2 = focalLength;

  //double viewup0 = 0.;
  //double viewup1 = -1.;
  //double viewup2 = 0.;

  //// ī�޶��� �þ� �����ϴ� �κ��� �ʿ���

  //// image plane�� 3D���� ��ġ�� �����ϴ� �κ�
  //double F = sqrt(focal0 * focal0 + focal1 * focal1 + focal2 * focal2);
  //double fovx = 69.4;  // realsense Ȩ�������� �����ͽ�Ʈ���� ������ �� (69.4, 42.5)
  //double viewAngle = fovx / 2.;
  //double degreeToRadian = 3.141592 / 180.;
  //double h = tan(viewAngle * degreeToRadian / 2) * F;
  //double point0[3] = { -h * planeRatio, -h, -F };
  //double point1[3] = { h * planeRatio, -h, -F };
  //double point2[3] = { -h * planeRatio, h, -F };

  // ���� ��ҵ��� ����� ���Ǿ����� Ȯ���ϱ� ���� �κе�
  //std::cout << "focal length: " << focalLength << std::endl;
  //std::cout << "cx: " << cx << std::endl;
  //std::cout << "cy: " << cy << std::endl;
  //std::cout << "F: " << F << std::endl;
  //std::cout << point0[0] << " " << point0[1] << " " << point0[2] << std::endl;
  //std::cout << point1[0] << " " << point1[1] << " " << point1[2] << std::endl;
  //std::cout << point2[0] << " " << point2[1] << " " << point2[2] << std::endl;
}

void VideoGrabber::reinit()
{
  // paused toggle ��ư�� ������ ��
  if (m_paused != d_paused.getValue())
  {
      PausedChanged();
  }
  // pauseTracker toggle ��ư�� ������ ��
  if (m_pauseTracker != d_pauseTracker.getValue())
  {
      PauseTrackerChanged();
  }
  // ī�޶� ��Ʈ ����Ǿ��� ��
  if (m_camIdx != d_camIdx.getValue())
  {
      CamIdxChanged();
  }
}

// SOFA framework �ȿ��� ���Ǵ� Ÿ�̸� (stepBegin�� stepEnd ���� �ൿ�� �ݺ�)
void VideoGrabber::doUpdate()
{
  sofa::helper::AdvancedTimer::stepBegin("RetrieveFrame");
  grabFrame();
  sofa::helper::AdvancedTimer::stepEnd("RetrieveFrame");
}

// ī�޶� ������ �Ͻ������� �����.
void VideoGrabber::PausedChanged()
{
  if (d_paused.getValue())
  {
      std::cout << "video paused!" << std::endl;
  }
  else
  {
      std::cout << "video restarted!" << std::endl;
  }
  m_paused = d_paused.getValue();
}

// OTS tracking�� �Ͻ������� �����.
void VideoGrabber::PauseTrackerChanged()
{
    if (d_pauseTracker.getValue())
    {
        std::cout << "OTS paused!" << std::endl;
    }
    else
    {
        std::cout << "OTS restarted!" << std::endl;
    }
    m_pauseTracker = d_pauseTracker.getValue();
}

// ī�޶� ��Ʈ�� ����Ǿ��� ��
void VideoGrabber::CamIdxChanged()
{
  cleanup();
  init();
}

void VideoGrabber::cleanup()
{
}

// arma::mat -> sofa::defaulttype::Mat4x4d
sofa::defaulttype::Mat4x4d VideoGrabber::armaToSofaMat(const arma::mat& mat)
{
    sofa::defaulttype::Mat4x4d tempCameraPose;
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            tempCameraPose.elems[row][col] = mat.at(row, col);
        }
    }
    return tempCameraPose;
}

VideoGrabber::~VideoGrabber() {}

}  // namespace video
}  // namespace acquisition
}  // namespace sofacv

// ���� gui�� ǥ���ϱ� ���� �Ʒ��� ���� ����ؾ� ��.
//d_cameraPos.setValue(to_sofaMat(tempHomogenious));  // camera ��ġ���� ������ gui�� �ݿ�
//d_cameraPosErr.setValue(tempError);                 // camera ��ġ���� ���� ������ gui�� �ݿ�