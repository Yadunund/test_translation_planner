#include "parse_graph.hpp"

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_traffic/geometry/Circle.hpp>

#include <iostream>

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: test_planner [NAV_GRAPH]" << std::endl;
    return 0;
  }

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(0.1),
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(0.1)
  };

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  const std::string& path = argv[1];
  std::cout << "parsing graph" << path << std::endl;
  const auto& graph = parse_graph(path, traits);
  std::cout << "Done parsing graph" << std::endl;
  const auto now = std::chrono::steady_clock::now();
  rmf_traffic::agv::Planner planner{
    rmf_traffic::agv::Planner::Configuration{
      graph, traits
    },
    rmf_traffic::agv::Planner::Options(nullptr)
  };

  const auto& start = rmf_traffic::agv::Planner::Start{now, 0, 0.0};
  const auto& goal = rmf_traffic::agv::Planner::Goal{3000};

  auto start_time = std::chrono::steady_clock::now();
  auto result = planner.plan(
    start, goal);
  auto finish_time = std::chrono::steady_clock::now();
  std::cout << "Time taken by planner: "
            << (finish_time - start_time).count() / 1e9
            << std::endl;

}
