#ifndef SOFACV_IMPLICITDATAENGINE_H
#define SOFACV_IMPLICITDATAENGINE_H

#include <SofaCV/SofaCVPlugin.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/IdleEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/PropagateEventVisitor.h>

namespace sofacv
{
/**
 *  \brief from a set of Data inputs computes a set of Data outputs while hiding
 * the data tracking mechanisms
 *
 * Implementation good rules:
 *
 * void init()
 * {
 *    bindInputData(&d_data); // indicate all inputs you want to bind
 *
 *    // add this line before bindInputData() if you want to
 *    // force this data to be explicitely linked in the xml scene
 *    if (d_data.isSet())
 *      bindInputData(&d_optionalData);
 *
 *    // use this call to use a callback function
 *    // called when the input is modified, just before update()
 *    bindInputData(&d_data,
 * (ImplicitDataEngine::DataCallback)&DerivedClass::func);
 * }
 *
 * // optional (called each time a data is modified in the gui)
 * // it is not always desired. You should use data callbacks instead
 * void reinit()
 * {
 *   // perform some action
 *   ImplicitDataEngine::reinit();
 * }
 *
 * void update()
 * {
 *  // perform your general computations
 * }
 *
 */

class SOFA_SOFACV_API ImplicitDataEngine : public sofa::core::DataEngine
{
 public:
  SOFA_CLASS(ImplicitDataEngine, sofa::core::DataEngine);

  ImplicitDataEngine();
  virtual ~ImplicitDataEngine() override;

 protected:
  /// Adds a new input to this engine, binds it to its parent if not set
  /// through XML, and sets a callback if provided
  /// trackOnly allows you to have an engine with autolink, but with some
  /// variables (flagged trackOnly) that would not be looked for using the
  /// autolink (optional)
  void addInput(sofa::core::objectmodel::BaseData* data,
                bool trackOnly = false);

  /// Sets a callback method for data
  void trackData(sofa::core::objectmodel::BaseData* data);
  /// Removes the callback method for a specific data
  void untrackData(sofa::core::objectmodel::BaseData* data);

 public:
  sofa::Data<bool> d_autolink;  ///< If false, engine won't try to implicitely
                                /// bind the inputs to previously declared

 private:
  bool _bindData(sofa::core::objectmodel::BaseData* data,
                 const std::string& alias);
  ImplicitDataEngine* getPreviousEngineInGraph();
};

}  // namespace sofacv

#endif  // SOFACV_IMPLICITDATAENGINE_H
