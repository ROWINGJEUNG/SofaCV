#include "BaseGrabber.h"

sofacv::acquisition::BaseGrabber::BaseGrabber()
    : d_interpolate(initData(&d_interpolate, false, "interpolate", "Should this "
			     "grabber interpolate? (if an implementation is provided)")),
      m_hasScheduler(false)
{
}

sofacv::acquisition::BaseGrabber::~BaseGrabber() {}

void sofacv::acquisition::BaseGrabber::handleEvent(sofa::core::objectmodel::Event *e)
{
    if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
    {
        if (!m_hasScheduler)
            update();
        return;
    }
    ImplicitDataEngine::handleEvent(e);
}

void sofacv::acquisition::BaseGrabber::grab(double elapsed)
{
    int time = int(elapsed);
    // "dumb" implementation fo the grabber just grabs until we're caught up:
    while (time)
        update();
}

void sofacv::acquisition::BaseGrabber::useScheduler(bool hasScheduler)
{
    m_hasScheduler = hasScheduler;
}
