#ifndef SOFACV_ACQUISITION_BASEGRABBER_H
#define SOFACV_ACQUISITION_BASEGRABBER_H

#include "SofaCV/ImplicitDataEngine.h"
#include "SofaCV/SofaCVPlugin.h"

namespace sofacv
{
namespace acquisition
{
/**
 * @brief The BaseGrabber interface
 *
 * All data grabbers should inherit this interface, as it provides a variation
 * to the ImplicitDataEngine's handleEvent function for data grabbers.
 */
class SOFA_SOFACV_API BaseGrabber : virtual public ImplicitDataEngine
{
 public:
    SOFA_ABSTRACT_CLASS(BaseGrabber, ImplicitDataEngine);

    BaseGrabber();
    virtual ~BaseGrabber() override;

    virtual void grab(double elapsed);
    sofa::Data<bool> d_interpolate { false }; /// should this grabber interpolate (if an implementation is provided) ?
    void useScheduler(bool);
protected:
    virtual void handleEvent(sofa::core::objectmodel::Event* e) override;

    bool m_hasScheduler { false }; /// If this grabber is controlled by a scheduler
};

}  // namespace acquisition
}  // namespace sofacv
#endif  // SOFACV_ACQUISITION_BASEGRABBER_H
