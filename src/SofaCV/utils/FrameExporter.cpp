#include "FrameExporter.h"
#include <opencv2/imgcodecs.hpp>

namespace sofacv
{
namespace utils
{

FrameExporter::FrameExporter()
    : d_filePath(initData(&d_filePath, "filePath", "absolute path to the output image file")),
      d_fileName(initData(&d_fileName, "fileName", "output image file name")),
      d_img(initData(&d_img, "img", "image to export")),
      d_nSteps(initData(&d_nSteps, (unsigned)1, "nSteps",
                        "number of steps between each export (0 means no "
                        "export during animation")),
      d_exportType(initData(&d_exportType, "exportType",
                            "specify when export should happen (BEGIN, END, "
                            "STEP). if STEP, specify export frequency with "
                            "nSteps. Default is END")),
      d_activate(initData(&d_activate, true, "active",
                          "if false, nothing will be exported")),
      m_stepCounter(0)
{
  sofa::helper::OptionsGroup* t = d_exportType.beginEdit();
  t->setNames(3, "BEGIN", "END", "STEP");
  t->setSelectedItem("END");
  d_exportType.endEdit();
}

void FrameExporter::init()
{
  m_stepCounter = 0;
  addInput(&d_img);
  update();
}

void FrameExporter::doUpdate()
{
  ++m_stepCounter;

  cv::Mat img;

  if (d_img.getValue().empty()) return;
  if (d_img.getValue().type() == CV_32FC1)
  {
    msg_warning("FrameExporter::export()")
        << "CV_32F matrices will be normalized into a CV_8U matrix. Consider "
           "converting first to optimize performances";
    cv::normalize(d_img.getValue(), img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  }
  else
    img = d_img.getValue();

  std::vector<int> qualityType;
  qualityType.push_back(cv::IMWRITE_JPEG_QUALITY);
  qualityType.push_back(90);

  switch (d_exportType.getValue().getSelectedId())
  {
    case 0:  // BEGIN
      if (m_stepCounter == 2)
      {
        cv::imwrite(d_filePath.getValue() + d_fileName.getValue(), img, qualityType);
      }
      break;
    case 2:  // STEP
      if (d_nSteps.getValue() == 0) return;
      if (m_stepCounter % d_nSteps.getValue() == 0)
      {
        cv::imwrite(d_filePath.getValue() + std::to_string(m_stepCounter) + "_" + d_fileName.getValue(),
                    img, qualityType);
      }
      break;
    default:
      break;
  }
}

void FrameExporter::cleanup()
{
  std::cout << "cleanup called on exporter" << std::endl;
  if (d_exportType.getValue().getSelectedId() == 1)  // END
  {
    std::vector<int> qualityType;
    qualityType.push_back(cv::IMWRITE_JPEG_QUALITY);
    qualityType.push_back(90);

    cv::Mat img;
    if (d_img.getValue().type() == CV_32FC1)
    {
      msg_warning("FrameExporter::export()")
          << "CV_32F matrices will be normalized into a CV_8U matrix. "
             "Consider "
             "converting first to optimize performances";
      cv::normalize(d_img.getValue(), img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    }
    else
      img = d_img.getValue();
    cv::imwrite(d_fileName.getValue(), img, qualityType);
  }
}

} // namespace utils
} // namespace sofacv
