#include "BaseFrameGrabber.h"

#include <sofa/core/ObjectFactory.h>

namespace sofacv
{
namespace acquisition
{
    SOFA_DECL_CLASS(BaseFrameGrabber)

        // int BaseFrameGrabberClass =
        //		sofa::core::RegisterObject(
        //        "OpenCV-based component reading mono and stereo videos")
        //        .add<BaseFrameGrabber>();

        BaseFrameGrabber::BaseFrameGrabber()
        : d_videoMode(initData(&d_videoMode, "video_mode",
            "determines if video is mono / stereo and if "
            "stereo, determines the Image format")),
        d_fullFrame(initData(&d_fullFrame, "fullFrame", "full frame retrieved", false, true)),

        // SOFA framework 안에서 시각화 되는 frame (VideoGrabber에서 프레임 정보 보냄)
        d_frame1(initData(&d_frame1, "img1", "  left frame (if stereo)", false, true)),
        d_frame2(initData(&d_frame2, "img2", "right frame (if stereo)", false, true)),

        // realSence에서 읽은 3D point data (VideoGrabber에서 값을 보냄)
        d_reconFrame(initData(&d_reconFrame, "3DPoints", "3D points from realsense", false, true)),

      d_fps(initData(&d_fps, "fps", "  video stream's fps", true, true)),
      d_dimensions(initData(&d_dimensions, "dimensions", "video stream's dimensions", true, true))
{
  this->addAlias(&d_fullFrame, "stereoFrame");
  this->addAlias(&d_fullFrame, "stereoFrame_out");
  this->addAlias(&d_fullFrame, "fullFrame_out");

  this->addAlias(&d_frame1, "img");
  this->addAlias(&d_frame1, "img_out");
  this->addAlias(&d_frame1, "img1_out");

  this->addAlias(&d_frame2, "img2_out");

  this->addAlias(&d_reconFrame, "3DPoints_out");

  sofa::helper::OptionsGroup* videoMode = d_videoMode.beginEdit();
  videoMode->setNames(5, "MONO", "STEREO_INTERLEAVED", "STEREO_TOP_BOTTOM",
                      "STEREO_SIDE_BY_SIDE", "STEREO_ROTATED_SIDE_BY_SIDE");
  videoMode->setSelectedItem(0);
  d_videoMode.endEdit();
}

BaseFrameGrabber::~BaseFrameGrabber() {}

void BaseFrameGrabber::init()
{
    addOutput(&d_fullFrame);
    addOutput(&d_frame1);
    addOutput(&d_frame2);
    addOutput(&d_reconFrame);   // FrameViewer class와 연결할 수 있도록 외부로 포트 열어둠

    addOutput(&d_dimensions);
    addOutput(&d_fps);
    setDirtyValue();
}

void BaseFrameGrabber::split_deinterleaved(const cvMat& src, cvMat& dstL,
                                           cvMat& dstR)
{
    // First put odd lines on the left & pair on right of the same image
  cv::Mat tmp(src.rows / 2, src.cols * 2, src.type(), src.data);
  // Then split vertically
  cv::Rect rectL(0, 0, tmp.cols / 2, tmp.rows);
  cv::Rect rectR(tmp.cols / 2, 0, tmp.cols / 2, tmp.rows);
  dstL = tmp(rectL).clone();
  dstR = tmp(rectR).clone();
}

void BaseFrameGrabber::split_top_bottom(const cvMat& src, cvMat& dstL,
                                        cvMat& dstR)
{
  // take first half of the rows
  dstL = src(cv::Range(0, src.rows / 2), cv::Range::all()).clone();
  // take second half of the rows
  dstR = src(cv::Range(src.rows / 2, src.rows), cv::Range::all()).clone();
}

void BaseFrameGrabber::split_side_by_side(const cvMat& src, cvMat& dstL,
                                          cvMat& dstR)
{
  // take first half of the cols
  dstL = src(cv::Range::all(), cv::Range(0, src.cols / 2)).clone();
  // take second half of the cols
  dstR = src(cv::Range::all(), cv::Range(src.cols / 2, src.cols)).clone();
}

void BaseFrameGrabber::split_rotated_side_by_side(const cvMat& src, cvMat& dstL,
                                                  cvMat& dstR)
{
  // same as side_by_side
  dstL = src(cv::Range::all(), cv::Range(0, src.cols / 2)).clone();
  dstR = src(cv::Range::all(), cv::Range(src.cols / 2, src.cols)).clone();

  // Then rotate:
  // transpose + flip around Y = 90° CW
  cv::transpose(dstL, dstL);
  cv::flip(dstL, dstL, 1);

  // transpose + flip around X = 90° CCW
  cv::transpose(dstR, dstR);
  cv::flip(dstR, dstR, 0);
}

void BaseFrameGrabber::splitFrames(const cvMat& src, cvMat& dstL, cvMat& dstR,
                                   image::VideoMode mode)
{
  switch (mode)
  {
    case image::STEREO_INTERLEAVED:
      split_deinterleaved(src, dstL, dstR);
      break;
    case image::STEREO_TOP_BOTTOM:
      split_top_bottom(src, dstL, dstR);
      break;
    case image::STEREO_SIDE_BY_SIDE:
      split_side_by_side(src, dstL, dstR);
      break;
    case image::STEREO_ROTATED_SIDE_BY_SIDE:
      split_rotated_side_by_side(src, dstL, dstR);
      break;
    case image::MONO:
      dstL = src;
      break;
  }
}

}  // namespace acquisition
}  // namespace sofacv
