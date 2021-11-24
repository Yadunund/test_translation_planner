#include "parse_graph.hpp"

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_traffic/geometry/Circle.hpp>

#include <iostream>

int main(int argc, char** argv)
{
  if (argc < 2)
  {
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
  auto result = planner.translation_plan(
    start, goal);
  auto finish_time = std::chrono::steady_clock::now();
  std::cout << "Time taken by translation only planner: "
            << (finish_time - start_time).count() / 1e9
            << std::endl;

  planner = rmf_traffic::agv::Planner{
    rmf_traffic::agv::Planner::Configuration{
      graph, traits
    },
    rmf_traffic::agv::Planner::Options(nullptr)
  };

  start_time = std::chrono::steady_clock::now();
  result = planner.plan(
    start, goal);
  finish_time = std::chrono::steady_clock::now();
  std::cout << "Time taken by default planner:: "
            << (finish_time - start_time).count() / 1e9
            << std::endl;
}
