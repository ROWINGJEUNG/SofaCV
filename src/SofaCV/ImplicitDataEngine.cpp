#include "ImplicitDataEngine.h"

namespace sofacv
{
bool ImplicitDataEngine::_bindData(sofa::core::objectmodel::BaseData* data,
                                   const std::string& alias)
{
  const std::multimap<std::string, sofa::core::objectmodel::BaseData*>&
      dataMap = this->getDataAliases();

  for (auto& d : dataMap)
    if (d.first == alias)
    {
      data->setParent(d.second, "@" + this->getPathName() + "." + alias);
      return true;
    }
  ImplicitDataEngine* engine = getPreviousEngineInGraph();
  if (engine)
    return engine->_bindData(data, alias);
  else
    return false;
}

ImplicitDataEngine* ImplicitDataEngine::getPreviousEngineInGraph()
{
  std::vector<ImplicitDataEngine*> engines;
  getContext()->get<ImplicitDataEngine>(&engines);
  for (size_t i = 0; i < engines.size(); ++i)
    if (engines[i]->getName() == getName())
      return (i) ? (engines[i - 1]) : (nullptr);
  return nullptr;
}

ImplicitDataEngine::ImplicitDataEngine()
    : d_autolink(initData(&d_autolink, false, "autolink",
                          "if set to true, allows implicit link setting "
                          "between components' data. This makes scene "
                          "writing less cumbersome but can potentially lead "
                          "to undefined / unexpected "
                          "behaviors. To use sparsely & wisely!"))
{
  f_listening.setValue(true);
}

ImplicitDataEngine::~ImplicitDataEngine() {}

void ImplicitDataEngine::addInput(sofa::core::objectmodel::BaseData* data,
                                  bool trackOnly)
{
  if (!d_autolink.getValue() || trackOnly || data->isSet())
  {
    sofa::core::DataEngine::addInput(data);
    return;
  }
  ImplicitDataEngine* engine = getPreviousEngineInGraph();
  if (engine)
  {
    bool isBinded = false;
    isBinded = engine->_bindData(data, data->getName() + "_out");

    if (!isBinded)
    {
      msg_warning(getName() + "::bindInputData()")
          << "couldn't bind input data " << data->getName()
          << " implicitly. Link is broken";
    }
    else
    {
      sofa::core::DataEngine::addInput(data);
      msg_advice(getName() + "::" + data->getName())
          << "linked to " << data->getLinkPath()
          << " implicitly. Please ensure that this makes sense. Otherwise, set "
             "autolink to false";
    }
  }
  else
    msg_warning(getName() + "::bindInputData()")
        << "couldn't bind input data " << data->getName()
        << " implicitly. Link is broken";
}

}  // namespace sofacv
