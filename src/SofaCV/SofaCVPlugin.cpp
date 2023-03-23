#include "SofaCVPlugin.h"
#include <sofa/config.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/config.h>

#ifdef QT_PLUGIN
#include <gui/ImageFilterDisplay.h>
#include <gui/ImageFilterModel.h>
#include <QApplication>
#include <QDebug>
#include <QQuickPaintedItem>
const int versionMajor = 1;
const int versionMinor = 0;

static void initResources() { Q_INIT_RESOURCE(SofaCV_qml); }
#endif  // QT_PLUGIN

#ifdef SOFA_HAVE_SOFAPYTHON
#include <SofaPython/PythonFactory.h>
#include "python/Binding_cvMatData.h"
#endif
#include "SofaCV.h"

namespace sofacv
{
/**
 * \brief Main namespace for SofaCV
 */
}  // namespace sofacv

namespace sofa
{
namespace component
{
// Here are just several convenient functions to help user to know what contains
// the plugin
extern "C"
{
  SOFA_SOFACV_API void initExternalModule();
  SOFA_SOFACV_API const char* getModuleName();
  SOFA_SOFACV_API const char* getModuleVersion();
  SOFA_SOFACV_API const char* getModuleLicense();
  SOFA_SOFACV_API const char* getModuleDescription();
  SOFA_SOFACV_API const char* getModuleComponentList();
}

void initExternalModule()
{
  static bool first = true;
  if (first)
  {
    first = false;
    sofa::helper::system::DataRepository.addLastPath(SOFACV_RESOURCES_PATH);
#ifdef QT_PLUGIN
    initResources();

    qmlRegisterType<sofacv::gui::ImageFilterDisplay>(
        "ImageFilterDisplay", versionMajor, versionMinor, "ImageFilterDisplay");
    qmlRegisterType<sofacv::gui::ImageFilterModel>(
        "ImageFilterModel", versionMajor, versionMinor, "ImageFilterModel");
    qmlRegisterType<sofacv::gui::ImageFilterModelList>(
        "ImageFilterModelList", versionMajor, versionMinor,
        "ImageFilterModelList");
#endif  // QT_PLUGIN
#ifdef SOFA_HAVE_SOFAPYTHON
    if (PythonFactory::s_sofaPythonModule)
    {
      simulation::PythonEnvironment::gil lock(__func__);

      // adding new bindings for Data<Mat>
      SP_ADD_CLASS_IN_FACTORY(cvMatData, sofa::Data<sofacv::cvMat>)
    }
#endif
  }
}

const char* getModuleName() { return "SofaCV"; }
const char* getModuleVersion() { return SofaCV_VERSION; }
const char* getModuleLicense() { return ""; }
const char* getModuleDescription()
{
  return "SofaCV plugin, containing data structures and utility "
         "functions for computer vision";
}

const char* getModuleComponentList() { return ""; }

}  // namespace component
}  // namespace sofa
