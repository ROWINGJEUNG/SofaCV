#pragma once

#include "BaseScheduler.h"

namespace sofacv
{
namespace acquisition
{

class DTScheduler : public BaseScheduler
{
public:
    DTScheduler();
    virtual void init() override;
    virtual void grabSignal() override;

    sofa::Data<double> d_dt; /// The frequency at which we want our input to be retrieved
    double m_timer;
};

}  // acquisition
}  // sofacv
