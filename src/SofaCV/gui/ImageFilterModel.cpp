#include "ImageFilterModel.h"

#include <SofaQtQuickGUI/SofaScene.h>

namespace sofacv
{
namespace gui
{
using namespace sofa;
ImageFilterModel::ImageFilterModel(QObject* parent)
    : QObject(parent), m_sofaComponent(0)
{
  connect(this, &ImageFilterModel::sofaComponentChanged, this,
          &ImageFilterModel::handleSofaDataChange);
}

qtquick::SofaComponent* ImageFilterModel::sofaComponent() const
{
  return m_sofaComponent;
}

void ImageFilterModel::setSofaComponent(qtquick::SofaComponent* sofaComponent)
{
  if (sofaComponent == m_sofaComponent) return;

  m_sofaComponent = sofaComponent;
  if (sofaComponent)
    m_sofaComponent = new qtquick::SofaComponent(*sofaComponent);

  sofaComponentChanged();
}

void ImageFilterModel::handleSofaDataChange()
{
  if (!m_sofaComponent) return;

  core::objectmodel::Base* baseComponent = m_sofaComponent->base();
  if (!baseComponent) return;

  // Here, list all potential renderer
  const core::objectmodel::BaseClass* bc = baseComponent->getClass();
  if (bc->hasParent("ImageFilter"))
    setImageFilter(dynamic_cast<ImageFilter*>(baseComponent));
  else
    msg_error("ImageFilterModel")
        << "Component Type " << baseComponent->getClassName() << " unknown";
}

ImageFilter* ImageFilterModel::imageFilter() const
{
  if (!m_sofaComponent)
  {
    m_imageFilter = 0;
  }
  else
  {
    const core::objectmodel::Base* baseComponent = m_sofaComponent->base();
    if (!baseComponent) m_imageFilter = 0;
  }

  return m_imageFilter;
}

void ImageFilterModel::setImageFilter(ImageFilter* imageFilter)
{
  m_imageFilter = imageFilter;
}

void ImageFilterModel::display()
{
  if (this->imageFilter()) this->imageFilter()->drawFullFrame();
}

////////

ImageFilterModelList::ImageFilterModelList(QObject* parent)
    : QObject(parent), m_sofaComponentList(nullptr)
{
  connect(this, &ImageFilterModelList::sofaComponentListChanged, this,
          &ImageFilterModelList::handleSofaDataChange);
}

sofa::qtquick::SofaComponentList ImageFilterModelList::sofaComponentList() const
{
  return m_sofaComponentList;
}

void ImageFilterModelList::setSofaComponentList(
    sofa::qtquick::SofaComponentList sofaComponentList)
{
  m_sofaComponentList = sofaComponentList;

  sofaComponentListChanged();
}

void ImageFilterModelList::handleSofaDataChange()
{
  if (m_sofaComponentList.size() < 1) return;

  for (sofacv::gui::ImageFilterModel* imageFilterModel : m_imageFilterModelList)
    delete imageFilterModel;

  m_imageFilterModelList.clear();

  for (int i = 0; i < m_sofaComponentList.size(); i++)
  // for (qtquick::SofaComponent* sofaComponent : m_sofaComponentList)
  {
    // core::objectmodel::Base* baseComponent = sofaComponent->base();
    core::objectmodel::Base* baseComponent = m_sofaComponentList.at(i)->base();
    if (!baseComponent)
    {
      // Here, list all potential renderer
      const core::objectmodel::BaseClass* bc = baseComponent->getClass();
      if (bc->hasParent("ImageFilter"))
      {
        sofacv::gui::ImageFilterModel* imageFilterModel =
            new sofacv::gui::ImageFilterModel();
        m_imageFilterModelList.push_back(imageFilterModel);
        imageFilterModel->setImageFilter(
            dynamic_cast<ImageFilter*>(baseComponent));
      }
      else
        msg_error("ImageFilterModelList") << "Type unknown";
    }
  }
}

void ImageFilterModelList::display(int index)
{
  if (m_sofaComponentList.size() <= index) return;

  ImageFilterModel* imageFilterModel =
      m_imageFilterModelList[index];

  if (imageFilterModel) imageFilterModel->display();
}

}  // namespace gui
}  // namespace sofacv
