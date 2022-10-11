#include "active_3d_planning_core/module/trajectory_evaluator/next_selector/gnn_pctsp.h"

#include <algorithm>
#include <vector>
//#include <Eigen/Core>

namespace active_3d_planning {
namespace next_selector {

// SubsequentBest
ModuleFactoryRegistry::Registration<GnnPCTSPSelector>
            GnnPCTSPSelector::registration("GnnPCTSPBest");

    GnnPCTSPSelector::GnnPCTSPSelector(PlannerI& planner) : NextSelector(planner) {}

void GnnPCTSPSelector::setupFromParamMap(Module::ParamMap* param_map) {
    setParam<float>(param_map, "x_min", &x_min_, 1.0);
    setParam<float>(param_map, "x_max", &x_max_, 1.0);
    setParam<float>(param_map, "y_min", &y_min_, 1.0);
    setParam<float>(param_map, "y_max", &y_max_, 1.0);
    setParam<float>(param_map, "z_min", &z_min_, 1.0);
    setParam<float>(param_map, "z_max", &z_max_, 1.0);
    }

int GnnPCTSPSelector::selectNextBest(TrajectorySegment* traj_in) {

  if (traj_in->children.size() > 1){

      using namespace utility;            // Common utilities like string conversions
      using namespace web;                // Common features like URIs.
      using namespace web::http;          // Common HTTP functionality
      using namespace web::http::client;  // HTTP client features

      // Create http_client to send the request.
      http_client client(U("http://localhost:5000"));

      // Definition of values to be inserted in the dictionary
      std::vector<float> depot, penalty, deterministic_prize, stochastic_prize, node_pos;
      std::vector<std::vector<float>> pre_loc;
      std::vector<std::vector<std::vector<float>>> loc;

      // Obtain the value from each of the children in the current segment
      depot.emplace_back((traj_in->children[0]->trajectory[0].position_W.x() - x_min_)/(x_max_ - x_min_));
      depot.emplace_back((traj_in->children[0]->trajectory[0].position_W.y() - y_min_)/(y_max_ - y_min_));
      depot.emplace_back((traj_in->children[0]->trajectory[0].position_W.z() - z_min_)/(z_max_ - z_min_));

      double max_x = 0.0, max_y = 0.0, max_z = 0.0;
      double x = 0.0, y = 0.0, z = 0.0;
      for (int i = 0; i < traj_in->children.size(); ++i) {
          double current_value = evaluateSingle(traj_in->children[i].get());
          if (current_value == 0) current_value = 0.1; //May happen depending on the gain being used.
          deterministic_prize.push_back(current_value);
          double current_cost = evaluateCost(traj_in->children[i].get());
          penalty.push_back(current_cost);
          //Position normalization
          //node_pos.emplace_back((traj_in->children[i]->trajectory.back().position_W.x() - x_min_)/(x_max_ - x_min_));
          //node_pos.emplace_back((traj_in->children[i]->trajectory.back().position_W.y() - y_min_)/(y_max_ - y_min_));
          //node_pos.emplace_back((traj_in->children[i]->trajectory.back().position_W.z() - z_min_)/(z_max_ - z_min_));

          //Displace normalization at the embedding
          x = traj_in->children[i]->trajectory.back().position_W.x();
          node_pos.emplace_back(x);
          if (max_x < x)
              max_x = traj_in->children[i]->trajectory.back().position_W.x();
          y = traj_in->children[i]->trajectory.back().position_W.y();
          node_pos.emplace_back(y);
          if (max_y < traj_in->children[i]->trajectory.back().position_W.y())
              max_y = traj_in->children[i]->trajectory.back().position_W.y();
          z = traj_in->children[i]->trajectory.back().position_W.z();
          node_pos.emplace_back(z);
          if (max_z < traj_in->children[i]->trajectory.back().position_W.z())
              max_z = traj_in->children[i]->trajectory.back().position_W.z();
          pre_loc.emplace_back(node_pos);
          //Eigen::Vector3d v2 << node_pos[0], node_pos[1], node_pos[2]);
          node_pos.clear();
      }
      //Eigen::MatrixXf test = Eigen::Map < Eigen::Matrix<float,traj_in->children.size(),1> >(pre_loc.data());

      //Value normalization
      float max = *std::max_element(deterministic_prize.begin(), deterministic_prize.end());
      //std::cout << traj_in->children.size() << " - SVAH: " << max << "\n" << std::endl;
      for(int index = 0; index < deterministic_prize.size(); index++)
          deterministic_prize[index] = deterministic_prize[index] / max;

      max = *std::max_element(penalty.begin(), penalty.end());
      for(int index = 0; index < penalty.size(); index++)
          penalty[index] = penalty[index] / max;

      stochastic_prize = deterministic_prize;

      loc.emplace_back(pre_loc);

      json::value v = json::value::object();

      v["depot"] = json::value::array();
      v["depot"][0] = json::value::array();
      for (size_t i = 0; i < depot.size(); ++i)
          v["depot"][0][i] = depot[i];

      v["penalty"] = json::value::array();
      v["penalty"][0] = json::value::array();
      for (size_t i = 0; i < penalty.size(); ++i)
          v["penalty"][0][i] = penalty[i];

      v["deterministic_prize"] = json::value::array();
      v["deterministic_prize"][0] = json::value::array();
      for (size_t i = 0; i < deterministic_prize.size(); ++i)
          v["deterministic_prize"][0][i] = deterministic_prize[i];

      v["stochastic_prize"] = json::value::array();
      v["stochastic_prize"][0] = json::value::array();
      for (size_t i = 0; i < stochastic_prize.size(); ++i)
          v["stochastic_prize"][0][i] = stochastic_prize[i];

      max_x++;
      max_y++;
      max_z++;
      v["loc"] = json::value::array();
      for (size_t b = 0; b < loc.size(); ++b) {
          auto batch = loc[b];
          v["loc"][b] = json::value::array();
          for (size_t p = 0; p < batch.size(); ++p) {
              auto points = batch[p];
              v["loc"][b][p] = json::value::array();
              for (size_t q = 0; q < points.size(); ++q) {
                  if(q == 0) v["loc"][b][p][q] = (points[q] - max_x)/(2 * max_x);
                  if(q == 1) v["loc"][b][p][q] = (points[q] - max_y)/(2 * max_y);
                  if(q == 2) v["loc"][b][p][q] = (points[q] - max_z)/(2 * max_z);
              }
          }
      }

      //std::cout << v.serialize() << std::endl;
      uri_builder builder2(U("/infer"));
      http_request req;
      req.set_body(v);
      req.set_request_uri(builder2.to_string());

      std::vector<int> pi;
      client.request(req)
              .then([&](http_response response) {
                  auto resp = response.extract_json().get();
                  auto index = resp.at(U("pi")).as_array()[0];
                  for (size_t i = 0; i < index.size(); i++) pi.emplace_back(index[i].as_integer());
              })
              .wait();
      int solution;
      if (pi.size() > 1) solution = (int) pi[0] - 1; //Index reference change
      else solution = 0;
      //std::cout << " - SVH: "<< solution << std::endl;
      return solution;

  }else{
      // If there is only one value in the tree return immediately zero.
      return 0;
  }
}

double GnnPCTSPSelector::evaluateSingle(TrajectorySegment* traj_in) {
  // Recursively find highest value
  if (traj_in->children.empty()) {
    return traj_in->value;
  }
  double highest_value = traj_in->value;
  for (int i = 0; i < traj_in->children.size(); ++i) {
    highest_value = 1 +
            std::max(highest_value, evaluateSingle(traj_in->children[i].get()));
  }
  return highest_value;
}

double GnnPCTSPSelector::evaluateCost(TrajectorySegment* traj_in) {
    // Recursively find highest cost
    if (traj_in->children.empty()) {
        return traj_in->cost;
    }
    double highest_value = traj_in->cost;
    for (int i = 0; i < traj_in->children.size(); ++i) {
        highest_value =
                std::max(highest_value, evaluateCost(traj_in->children[i].get()));
    }
    return highest_value;
}




}  // namespace next_selector
}  // namespace active_3d_planning
