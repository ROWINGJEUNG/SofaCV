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

#include <SofaCV/acquisition/BaseFrameGrabber.h>
using sofacv::acquisition::BaseFrameGrabber;
#include <SofaTest/Sofa_test.h>

#include <SofaSimulationGraph/DAGSimulation.h>
#include <sofa/simulation/Simulation.h>
using sofa::core::objectmodel::BaseObject;
using sofa::simulation::Simulation;
using sofa::simulation::Node;

#include <SofaSimulationCommon/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML;
using sofa::core::ExecParams;

namespace sofa
{
struct TestFrameGrabber : public BaseFrameGrabber
{
  cv::Mat src;
  sofacv::cvMat dst1, dst2;

  TestFrameGrabber()
  {
    src = (cv::Mat_<double>(4, 2) << 1, 2, 3, 4, 5, 6, 7, 8);
  }

  void init() override {}
  void doUpdate() override {}

  void testSplitDeinterleaved()
  {
    this->splitFrames(src, dst1, dst2,
                      sofacv::image::STEREO_INTERLEAVED);

    ASSERT_FALSE(dst2.at<uchar>(0, 0) != src.at<uchar>(1, 0));
    ASSERT_FALSE(dst1.at<uchar>(0, 0) != src.at<uchar>(0, 0));
    ASSERT_FALSE(dst2.at<uchar>(1, 0) != src.at<uchar>(3, 0));
    ASSERT_FALSE(dst1.at<uchar>(1, 0) != src.at<uchar>(2, 0));

    ASSERT_FALSE(dst2.at<uchar>(0, 1) != src.at<uchar>(1, 1));
    ASSERT_FALSE(dst1.at<uchar>(0, 1) != src.at<uchar>(0, 1));
    ASSERT_FALSE(dst2.at<uchar>(1, 1) != src.at<uchar>(3, 1));
    ASSERT_FALSE(dst1.at<uchar>(1, 1) != src.at<uchar>(2, 1));
  }
  void testSplitTopBottom()
  {
    this->splitFrames(src, dst1, dst2,
                      sofacv::image::STEREO_TOP_BOTTOM);

    ASSERT_FALSE(dst1.at<uchar>(0, 0) != src.at<uchar>(0, 0));
    ASSERT_FALSE(dst1.at<uchar>(0, 1) != src.at<uchar>(0, 1));
    ASSERT_FALSE(dst1.at<uchar>(1, 0) != src.at<uchar>(1, 0));
    ASSERT_FALSE(dst1.at<uchar>(1, 1) != src.at<uchar>(1, 1));

    ASSERT_FALSE(dst2.at<uchar>(0, 0) != src.at<uchar>(2, 0));
    ASSERT_FALSE(dst2.at<uchar>(0, 1) != src.at<uchar>(2, 1));
    ASSERT_FALSE(dst2.at<uchar>(1, 0) != src.at<uchar>(3, 0));
    ASSERT_FALSE(dst2.at<uchar>(1, 1) != src.at<uchar>(3, 1));
  }
  void testSplitSideBySide()
  {
    this->splitFrames(src, dst1, dst2,
                      sofacv::image::STEREO_SIDE_BY_SIDE);

    ASSERT_FALSE(dst1.rows != 4 || dst1.cols != 1);
    ASSERT_FALSE(dst1.at<uchar>(0, 0) != src.at<uchar>(0, 0));
    ASSERT_FALSE(dst1.at<uchar>(1, 0) != src.at<uchar>(1, 0));
    ASSERT_FALSE(dst1.at<uchar>(2, 0) != src.at<uchar>(2, 0));
    ASSERT_FALSE(dst1.at<uchar>(3, 0) != src.at<uchar>(3, 0));

    ASSERT_FALSE(dst2.rows != 4 || dst2.cols != 1);
    ASSERT_FALSE(dst2.at<uchar>(0, 0) != src.at<uchar>(0, 1));
    ASSERT_FALSE(dst2.at<uchar>(1, 0) != src.at<uchar>(1, 1));
    ASSERT_FALSE(dst2.at<uchar>(2, 0) != src.at<uchar>(2, 1));
    ASSERT_FALSE(dst2.at<uchar>(3, 0) != src.at<uchar>(3, 1));
  }
  void testSplitRotatedSideBySide()
  {
    this->splitFrames(src, dst1, dst2,
                      sofacv::image::STEREO_ROTATED_SIDE_BY_SIDE);

    ASSERT_FALSE(dst1.rows != 1 || dst1.cols != 4);
    ASSERT_FALSE(dst1.at<uchar>(0, 3) != src.at<uchar>(0, 0));
    ASSERT_FALSE(dst1.at<uchar>(0, 2) != src.at<uchar>(1, 0));
    ASSERT_FALSE(dst1.at<uchar>(0, 1) != src.at<uchar>(2, 0));
    ASSERT_FALSE(dst1.at<uchar>(0, 0) != src.at<uchar>(3, 0));

    ASSERT_FALSE(dst2.rows != 1 || dst2.cols != 4);
    ASSERT_FALSE(dst2.at<uchar>(0, 0) != src.at<uchar>(0, 1));
    ASSERT_FALSE(dst2.at<uchar>(0, 1) != src.at<uchar>(1, 1));
    ASSERT_FALSE(dst2.at<uchar>(0, 2) != src.at<uchar>(2, 1));
    ASSERT_FALSE(dst2.at<uchar>(0, 3) != src.at<uchar>(3, 1));
  }
};

struct BaseFrameGrabber_test : public sofa::BaseTest
{
  void testStereoSplitting()
  {
    TestFrameGrabber test;
    test.testSplitDeinterleaved();
    test.testSplitTopBottom();
    test.testSplitSideBySide();
    test.testSplitRotatedSideBySide();
  }

  void SetUp() {}

  void TearDown() {}
};

/// Checks whether the video frames are split as expected
TEST_F(BaseFrameGrabber_test, testStereoSplitting)
{
  this->testStereoSplitting();
}

}  // namespace sofa
