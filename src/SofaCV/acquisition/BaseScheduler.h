#ifndef SOFACV_ACQUISITION_BASESCHEDULER_H
#define SOFACV_ACQUISITION_BASESCHEDULER_H

#include <SofaCV/SofaCVPlugin.h>
#include <SofaCV/acquisition/BaseGrabber.h>
using sofacv::acquisition::BaseGrabber;

#include <sofa/core/objectmodel/BaseObject.h>
using sofa::core::objectmodel::BaseObject;
using sofa::Data;

namespace sofacv
{
namespace acquisition
{

/*
 * \brief Schedulers are responsible for dispatching inputs while synchronizing them
 * with the simulation.
 *
 * This BaseScheduler Interface provides a method (checkElapsedTime) which is responsible
 * for conditionally calling the Input grabber's grab() method, which in turn will dispatch
 * a new frame, or new signal to the simulation
 */
class SOFA_SOFACV_API BaseScheduler : public BaseObject
{

    typedef sofa::core::objectmodel::SingleLink<
        BaseScheduler, BaseGrabber,
        sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
        Grabber;
public:
    SOFA_ABSTRACT_CLASS(BaseScheduler, BaseObject);
public:
    BaseScheduler();

    void init() override;
    void handleEvent(sofa::core::objectmodel::Event* e) override;

    virtual void grabSignal() = 0;  /// Tells the grabber to release a new frame, based on the output of shoudlGrabSignal()

    Grabber l_grabber;  /// The input grabber. Will release a new frame when notified
};

}  // acquisition
}  // sofacv

#endif // SOFACV_ACQUISITION_BASESCHEDULER_H
