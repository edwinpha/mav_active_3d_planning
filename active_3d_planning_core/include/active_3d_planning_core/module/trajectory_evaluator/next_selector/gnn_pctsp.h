#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_H_

#include "active_3d_planning_core/module/trajectory_evaluator.h"

#include <cpprest/http_client.h>
#include <cpprest/json.h>

namespace active_3d_planning {
namespace next_selector {

// Select the child node which contains the highest value segment in its subtree
class GnnPCTSPSelector : public NextSelector {
 public:
  explicit GnnPCTSPSelector(PlannerI& planner);  // NOLINT

  // override virtual functions
  int selectNextBest(TrajectorySegment* traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<GnnPCTSPSelector> registration;
  float x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;

  // methods
  double evaluateSingle(TrajectorySegment* traj_in);
  double evaluateCost(TrajectorySegment* traj_in);
};

}  // namespace next_selector
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_H_
