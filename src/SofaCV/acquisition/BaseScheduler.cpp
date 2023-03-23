#include "BaseScheduler.h"

namespace sofacv
{
namespace acquisition
{

BaseScheduler::BaseScheduler()
    : l_grabber(initLink("grabber", "link to the grabber component containing the input data stream"))
{
    f_listening = true;
}

void BaseScheduler::init()
{
    if (!l_grabber.get())
      msg_error(getName() + "::init()") << "Error: No grabber link set."
                                           "A scheduler needs a grabber source to schedule!";
    else
    {
        l_grabber->useScheduler(true);
    }
}

void BaseScheduler::handleEvent(sofa::core::objectmodel::Event* e)
{
    if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
    {
        grabSignal();
    }
}

} // namespace acquisition
}  // sofacv
