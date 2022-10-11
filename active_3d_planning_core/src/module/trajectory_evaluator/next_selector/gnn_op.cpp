#include "active_3d_planning_core/module/trajectory_evaluator/next_selector/gnn_op.h"

#include <algorithm>
#include <vector>

namespace active_3d_planning {
namespace next_selector {

// SubsequentBest
ModuleFactoryRegistry::Registration<GnnOPSelector>
            GnnOPSelector::registration("GnnOPBest");

    GnnOPSelector::GnnOPSelector(PlannerI& planner) : NextSelector(planner) {}

void GnnOPSelector::setupFromParamMap(Module::ParamMap* param_map) {
    setParam<float>(param_map, "x_min", &x_min_, 1.0);
    setParam<float>(param_map, "x_max", &x_max_, 1.0);
    setParam<float>(param_map, "y_min", &y_min_, 1.0);
    setParam<float>(param_map, "y_max", &y_max_, 1.0);
    setParam<float>(param_map, "z_min", &z_min_, 1.0);
    setParam<float>(param_map, "z_max", &z_max_, 1.0);
    }

int GnnOPSelector::selectNextBest(TrajectorySegment* traj_in) {

  if (traj_in->children.size() > 1){

      using namespace utility;            // Common utilities like string conversions
      using namespace web;                // Common features like URIs.
      using namespace web::http;          // Common HTTP functionality
      using namespace web::http::client;  // HTTP client features

      // Create http_client to send the request.
      http_client client(U("http://localhost:5000"));

      // Definition of values to be inserted in the dictionary
      std::vector<float> depot, max_length, prize, node_pos;
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
          if (current_value == 0) current_value = 0.1;
          prize.push_back(current_value);
          //std::cout << " * prize: " << current_value << std::endl;
          max_length.emplace_back(2.0);
          //Position normalization
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
          node_pos.clear();
      }

      //Value normalization
      float max = *std::max_element(prize.begin(), prize.end());
      std::cout << traj_in->children.size() << " - SVAH: " << max << "\n" << std::endl;
      for(int index = 0; index < prize.size(); index++) prize[index] = prize[index] / max;

      loc.emplace_back(pre_loc);

      json::value v = json::value::object();

      v["depot"] = json::value::array();
      v["depot"][0] = json::value::array();
      for (size_t i = 0; i < depot.size(); ++i){
          if(i == 0) v["depot"][0][i] = (depot[i] - max_x)/(2 * max_x);
          if(i == 1) v["depot"][0][i] = (depot[i] - max_y)/(2 * max_y);
          if(i == 2) v["depot"][0][i] = (depot[i] - max_z)/(2 * max_z);
      }

      v["prize"] = json::value::array();
      v["prize"][0] = json::value::array();
      for (size_t i = 0; i < prize.size(); ++i)
          v["prize"][0][i] = prize[i];

      v["max_length"] = json::value::array();
      for (size_t i = 0; i < max_length.size(); ++i)
          v["max_length"][i] = max_length[i];

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

double GnnOPSelector::evaluateSingle(TrajectorySegment* traj_in) {
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




}  // namespace next_selector
}  // namespace active_3d_planning
