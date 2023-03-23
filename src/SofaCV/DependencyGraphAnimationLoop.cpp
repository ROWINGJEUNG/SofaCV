#include "DependencyGraphAnimationLoop.h"
#include "DependencyGraphVisitor.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/DefaultAnimationLoop.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/AnimateVisitor.h>
#include <sofa/simulation/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/PropagateEventVisitor.h>
#include <sofa/simulation/UpdateBoundingBoxVisitor.h>
#include <sofa/simulation/UpdateContextVisitor.h>
#include <sofa/simulation/UpdateMappingEndEvent.h>
#include <sofa/simulation/UpdateMappingVisitor.h>

#include <sofa/helper/system/SetDirectory.h>
//#include <sofa/helper/system/PipeProcess.h>
#include <sofa/helper/AdvancedTimer.h>

#include <sofa/core/visual/VisualParams.h>

#include <math.h>
#include <stdlib.h>
#include <algorithm>

namespace sofa
{
namespace simulation
{
SOFA_DECL_CLASS(DependencyGraphAnimationLoop)

int DependencyGraphAnimationLoopClass =
    core::RegisterObject(
        "The Default animation loop, running the DependencyGraphVisitor")
        .add<DependencyGraphAnimationLoop>();

void DependencyGraphAnimationLoop::init()
{
  DefaultAnimationLoop::init();
  DependencyGraphVisitor visitor(sofa::core::ExecParams::defaultInstance());
  this->gnode->execute(visitor);
}

void DependencyGraphAnimationLoop::step(const core::ExecParams* params,
                                        SReal dt)
{
  DependencyGraphVisitor visitor(params);
  this->gnode->execute(visitor);

  DefaultAnimationLoop::step(params, dt);
}

}  // namespace simulation
}  // namespace sofa
