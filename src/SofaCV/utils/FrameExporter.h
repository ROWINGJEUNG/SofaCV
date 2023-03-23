#ifndef SOFACV_UTILS_FRAMEEXPORTER_H
#define SOFACV_UTILS_FRAMEEXPORTER_H

#include "SofaCV/SofaCVPlugin.h"
#include "SofaCV/datatypes/cvMat.h"
#include "SofaCV/ImplicitDataEngine.h"

#include <sofa/helper/OptionsGroup.h>

#include <opencv2/core.hpp>

namespace sofacv
{
namespace utils
{
/**
 * @brief The FrameExporter class
 *
 * Export an input image (img) into [fileName] either at BEGIN, END or at a
 * specific frequency of the simulation (STEP).
 * Can be activated / deactivated manually with [active]
 */
class SOFA_SOFACV_API FrameExporter : public ImplicitDataEngine
{
  sofa::Data<std::string> d_filePath;  ///< name of the file to write into
  sofa::Data<std::string> d_fileName;  ///< name of the file to write into
  sofa::Data<cvMat> d_img;     ///< [INPUT] image to write in fileName
  sofa::Data<unsigned>
      d_nSteps;  ///< if exportType == STEP, frequency at which to export
  sofa::Data<sofa::helper::OptionsGroup> d_exportType;  ///< whether the image
                                                        /// should be written at
  /// begin or end of the
  /// simulation, or
  /// repeatedly every N
  /// steps
  sofa::Data<bool>
      d_activate;  ///< set to false to manually activate through GUI

 public:
  SOFA_CLASS(FrameExporter, ImplicitDataEngine);

  FrameExporter();

  virtual ~FrameExporter() override {}
  void init() override;

  void reinit() override {}

  void doUpdate() override;

  void cleanup() override;

  virtual void handleEvent(sofa::core::objectmodel::Event* e) override
  {
    if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
    {
      update(); /// always call update() for grabbers. It's
                /// decided internally whether or not they should do something
      return;
    }
    ImplicitDataEngine::handleEvent(e);
  }

 private:
  unsigned m_stepCounter;
};

SOFA_DECL_CLASS(FrameExporter)

int FrameExporterClass =
    sofa::core::RegisterObject(
        "component to export Opencv images as a file on your system")
        .add<FrameExporter>();

}  // namespace utils
}  // namespace sofacv
#endif  // SOFACV_UTILS_FRAMEEXPORTER_H
