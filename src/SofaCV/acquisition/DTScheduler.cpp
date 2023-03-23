#include "DTScheduler.h"
#include <cmath>
#include <numeric>
#include <chrono>

namespace sofacv
{
namespace acquisition
{

SOFA_DECL_CLASS(DTScheduler)

static int DTSchedulerClass =
    sofa::core::RegisterObject(
        "DT-based Scheduler to synchronize the input feed with the simulation")
        .add<DTScheduler>();

DTScheduler::DTScheduler()
    : d_dt(initData(&d_dt, "dt", "The frequency at which we want our input to"
                                 " be retrieved")),
      m_timer(.0)
{

}

void DTScheduler::init()
{
    BaseScheduler::init();
    if (!d_dt.isSet())
    {
        d_dt.setValue(this->getContext()->getDt());
    }
    m_timer = this->getContext()->getTime();
}

void DTScheduler::grabSignal()
{
    // get the total time elapsed since last frame was retrieved
    double elapsedTime = this->getContext()->getTime() - m_timer;
    if (elapsedTime >= d_dt.getValue())
    {
        m_timer = this->getContext()->getTime();
        // Call BaseGrabber::grab() with the delta between the current step and the next one to grab
        l_grabber->grab(double(std::nearbyint(elapsedTime / d_dt.getValue())));
    }
}

}  // acquisition
}  // sofacv
