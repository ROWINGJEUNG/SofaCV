#ifndef SOFA_SIMULATION_DGANIMATIONLOOP_H
#define SOFA_SIMULATION_DGANIMATIONLOOP_H

#include <sofa/simulation/DefaultAnimationLoop.h>

namespace sofa
{
namespace simulation
{
class DependencyGraphAnimationLoop
    : public sofa::simulation::DefaultAnimationLoop
{
 public:
  DependencyGraphAnimationLoop(sofa::simulation::Node* gnode = NULL)
      : sofa::simulation::DefaultAnimationLoop(gnode)
  {
  }

  virtual ~DependencyGraphAnimationLoop() {}

  virtual void init() override;

  /// perform one animation step
  virtual void step(const sofa::core::ExecParams* params, SReal dt) override;
};

}  // namespace simulation

}  // namespace sofa

#endif /* SOFA_SIMULATION_DGANIMATIONLOOP_H */
