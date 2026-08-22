/**
 * @file planning_simulator.cpp
 * @brief 不依賴 ROS2 的 Taiphoon Agent 離線路徑規劃引擎
 * @author Taiphoon Agent Team
 * @date 2026-08-20
 *
 * 此工具只組合 Taiphoon_Agent 既有的純 C++ EGO-Swarm 元件。它透過
 * stdin 接收簡單的 key-value 情境描述，並輸出 JSON 結果，讓上層的
 * Python 本機服務或其他工具可以在不啟動 ROS2／SITL 的情況下使用相同
 * 的 planner backend 與 safety validator。
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ego_swarm/ContinuousSafetyValidator.hpp"
#include "ego_swarm/OptimizedEgoPlanner.hpp"
#include "ego_swarm/PlannerTypes.hpp"
#include "ego_swarm/PeerTrajectoryTable.hpp"
#include "ego_swarm/TrajectoryFactory.hpp"
#include "perception/ObstacleCloudGenerator.hpp"
#include "perception/RawPointObstacleMap.hpp"

namespace
{

using taiphoon::ego_swarm::BsplineTrajectory;
using taiphoon::ego_swarm::ContinuousSafetyValidator;
using taiphoon::ego_swarm::OptimizedEgoPlanner;
using taiphoon::ego_swarm::PathConstraint;
using taiphoon::ego_swarm::PeerTrajectory;
using taiphoon::ego_swarm::PeerTrajectoryTable;
using taiphoon::ego_swarm::PeerTrajectoryTableConfig;
using taiphoon::ego_swarm::PlannerConfig;
using taiphoon::ego_swarm::PlannerResult;
using taiphoon::ego_swarm::PlanningSnapshot;
using taiphoon::ego_swarm::SafetyValidationContext;
using taiphoon::ego_swarm::SafetyValidationResult;
using taiphoon::ego_swarm::Vector3;
using taiphoon::ego_swarm::VehiclePlanningState;
using taiphoon::perception::CloudMode;
using taiphoon::perception::GenerationConfig;
using taiphoon::perception::ObstacleCloudGenerator;
using taiphoon::perception::ObstacleDefinition;
using taiphoon::perception::ObstacleType;
using taiphoon::perception::RawPointObstacleMap;

constexpr std::uint32_t FRAME_REVISION = 1U;
constexpr double SIMULATION_TIME_S = 0.0;
constexpr double MAX_TIME_SKEW_S = 1.0;
constexpr std::size_t DEFAULT_PEER_CONTROL_POINT_COUNT = 6U;
constexpr double DEFAULT_PEER_KNOT_INTERVAL_S = 1.0;
constexpr double DEFAULT_SAMPLE_SPACING_M = 0.5;

struct PeerDefinition
{
    int agent_id{0};
    bool terminal_hold{false};
    Vector3 start{0.0, 0.0, 0.0};
    Vector3 goal{0.0, 0.0, 0.0};
    std::size_t control_point_count{DEFAULT_PEER_CONTROL_POINT_COUNT};
    double knot_interval_s{DEFAULT_PEER_KNOT_INTERVAL_S};
};

struct SimulationInput
{
    VehiclePlanningState self_state{};
    Vector3 goal{10.0, 0.0, -5.0};
    PathConstraint path_constraint{PathConstraint::FREE_3D};
    PlannerConfig planner_config{};
    std::vector<ObstacleDefinition> obstacles{};
    double sample_spacing_m{DEFAULT_SAMPLE_SPACING_M};
    std::vector<PeerDefinition> peers{};
};

struct SimulationOutput
{
    PlannerResult planner_result{};
    SafetyValidationResult safety_result{};
    std::vector<PeerDefinition> peers{};
    std::string error{};
};

bool parse_double(const std::string& token, double& value)
{
    try
    {
        std::size_t consumed = 0U;
        value = std::stod(token, &consumed);
        return consumed == token.size() && std::isfinite(value);
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool parse_int(const std::string& token, int& value)
{
    try
    {
        std::size_t consumed = 0U;
        const long parsed = std::stol(token, &consumed);
        if (consumed != token.size() ||
            parsed < static_cast<long>(std::numeric_limits<int>::min()) ||
            parsed > static_cast<long>(std::numeric_limits<int>::max()))
        {
            return false;
        }
        value = static_cast<int>(parsed);
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool parse_size(const std::string& token, std::size_t& value)
{
    try
    {
        std::size_t consumed = 0U;
        const unsigned long long parsed = std::stoull(token, &consumed);
        if (consumed != token.size() ||
            parsed > static_cast<unsigned long long>(
                std::numeric_limits<std::size_t>::max()))
        {
            return false;
        }
        value = static_cast<std::size_t>(parsed);
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool read_vector3(
    std::istringstream& stream,
    Vector3& value)
{
    std::string tokens[3];
    for (std::string& token : tokens)
    {
        if (!(stream >> token))
        {
            return false;
        }
    }
    return parse_double(tokens[0], value.at(0)) &&
           parse_double(tokens[1], value.at(1)) &&
           parse_double(tokens[2], value.at(2));
}

bool read_point3f(
    std::istringstream& stream,
    std::array<float, 3>& value)
{
    Vector3 parsed{};
    if (!read_vector3(stream, parsed))
    {
        return false;
    }
    value = {
        static_cast<float>(parsed.at(0)),
        static_cast<float>(parsed.at(1)),
        static_cast<float>(parsed.at(2))};
    return true;
}

bool parse_path_constraint(
    const std::string& token,
    PathConstraint& constraint)
{
    if (token == "free_3d")
    {
        constraint = PathConstraint::FREE_3D;
        return true;
    }
    if (token == "fixed_z")
    {
        constraint = PathConstraint::FIXED_Z;
        return true;
    }
    if (token == "linear_z")
    {
        constraint = PathConstraint::LINEAR_Z;
        return true;
    }
    return false;
}

bool parse_obstacle_type(
    const std::string& token,
    ObstacleType& type)
{
    if (token == "box")
    {
        type = ObstacleType::Box;
        return true;
    }
    if (token == "cylinder")
    {
        type = ObstacleType::Cylinder;
        return true;
    }
    if (token == "wall")
    {
        type = ObstacleType::Wall;
        return true;
    }
    return false;
}

bool parse_input(
    std::istream& input,
    SimulationInput& output,
    std::string& error)
{
    std::string line;
    std::size_t line_number = 0U;
    while (std::getline(input, line))
    {
        ++line_number;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos)
        {
            line.erase(comment);
        }
        std::istringstream stream(line);
        std::string key;
        if (!(stream >> key))
        {
            continue;
        }

        if (key == "self_position")
        {
            if (!read_vector3(stream, output.self_state.position))
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid self_position";
                return false;
            }
        }
        else if (key == "self_velocity")
        {
            if (!read_vector3(stream, output.self_state.velocity))
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid self_velocity";
                return false;
            }
        }
        else if (key == "self_acceleration")
        {
            if (!read_vector3(stream, output.self_state.acceleration))
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid self_acceleration";
                return false;
            }
        }
        else if (key == "goal")
        {
            if (!read_vector3(stream, output.goal))
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid goal";
                return false;
            }
        }
        else if (key == "path_constraint")
        {
            std::string token;
            if (!(stream >> token) ||
                !parse_path_constraint(token, output.path_constraint))
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid path_constraint";
                return false;
            }
        }
        else if (key == "sample_spacing_m")
        {
            std::string token;
            if (!(stream >> token) || !parse_double(
                    token, output.sample_spacing_m))
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid sample_spacing_m";
                return false;
            }
        }
        else if (key == "planner")
        {
            std::string name;
            std::string token;
            if (!(stream >> name >> token))
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid planner parameter";
                return false;
            }
            bool valid = true;
            if (name == "control_point_count")
            {
                valid = parse_size(token,
                                   output.planner_config.control_point_count);
            }
            else if (name == "knot_interval_s")
            {
                valid = parse_double(token,
                                     output.planner_config.knot_interval_s);
            }
            else if (name == "sample_interval_s")
            {
                valid = parse_double(token,
                                     output.planner_config.sample_interval_s);
            }
            else if (name == "obstacle_clearance_m")
            {
                valid = parse_double(token,
                                     output.planner_config.obstacle_clearance_m);
            }
            else if (name == "peer_collision_min_distance_m")
            {
                valid = parse_double(
                    token,
                    output.planner_config.peer_collision_min_distance_m);
            }
            else if (name == "max_velocity_mps")
            {
                valid = parse_double(token,
                                     output.planner_config.max_velocity_mps);
                output.planner_config.max_velocity_m_s =
                    output.planner_config.max_velocity_mps;
            }
            else if (name == "max_acceleration_mps2")
            {
                valid = parse_double(
                    token,
                    output.planner_config.max_acceleration_mps2);
                output.planner_config.max_acceleration_m_s2 =
                    output.planner_config.max_acceleration_mps2;
            }
            else if (name == "max_jerk_mps3")
            {
                valid = parse_double(token,
                                     output.planner_config.max_jerk_mps3);
            }
            else if (name == "optimizer_max_iterations")
            {
                valid = parse_int(
                    token, output.planner_config.optimizer_max_iterations);
            }
            else if (name == "optimizer_time_budget_ms")
            {
                valid = parse_double(
                    token, output.planner_config.optimizer_time_budget_ms);
            }
            else if (name == "candidate_offset_m")
            {
                valid = parse_double(token,
                                     output.planner_config.candidate_offset_m);
            }
            else
            {
                error = "line " + std::to_string(line_number) +
                    ": unknown planner parameter '" + name + "'";
                return false;
            }
            if (!valid)
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid planner parameter '" + name + "'";
                return false;
            }
        }
        else if (key == "obstacle")
        {
            std::string type_token;
            if (!(stream >> type_token))
            {
                error = "line " + std::to_string(line_number) +
                    ": missing obstacle type";
                return false;
            }
            ObstacleDefinition obstacle;
            if (!parse_obstacle_type(type_token, obstacle.type) ||
                !read_point3f(stream, obstacle.center_ned))
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid obstacle";
                return false;
            }
            if (obstacle.type == ObstacleType::Cylinder)
            {
                std::string radius_token;
                std::string height_token;
                double radius = 0.0;
                double height = 0.0;
                if (!(stream >> radius_token >> height_token) ||
                    !parse_double(radius_token, radius) ||
                    !parse_double(height_token, height) ||
                    radius <= 0.0 || height <= 0.0)
                {
                    error = "line " + std::to_string(line_number) +
                        ": invalid cylinder obstacle";
                    return false;
                }
                obstacle.radius_m = static_cast<float>(radius);
                obstacle.height_m = static_cast<float>(height);
            }
            else if (!read_point3f(stream, obstacle.size_m))
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid box/wall obstacle";
                return false;
            }
            output.obstacles.push_back(obstacle);
        }
        else if (key == "peer")
        {
            PeerDefinition peer;
            std::string mode;
            if (!(stream >> peer.agent_id >> mode) || peer.agent_id <= 0 ||
                (mode != "hold" && mode != "move") ||
                !read_vector3(stream, peer.start))
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid peer header";
                return false;
            }
            peer.terminal_hold = mode == "hold";
            if (peer.terminal_hold)
            {
                peer.goal = peer.start;
            }
            else if (!read_vector3(stream, peer.goal))
            {
                error = "line " + std::to_string(line_number) +
                    ": invalid moving peer goal";
                return false;
            }
            output.peers.push_back(peer);
        }
        else if (key != "end")
        {
            error = "line " + std::to_string(line_number) +
                ": unknown key '" + key + "'";
            return false;
        }
    }

    output.self_state.timestamp_s = SIMULATION_TIME_S;
    return true;
}

BsplineTrajectory make_peer_trajectory(
    const PeerDefinition& peer)
{
    const std::size_t control_point_count = std::max<std::size_t>(
        peer.control_point_count, taiphoon::ego_swarm::BSPLINE_DEGREE + 1U);
    std::vector<Vector3> control_points;
    control_points.reserve(control_point_count);
    for (std::size_t index = 0U; index < control_point_count; ++index)
    {
        const double ratio = control_point_count <= 1U
            ? 0.0
            : static_cast<double>(index) /
                static_cast<double>(control_point_count - 1U);
        control_points.push_back({
            peer.start.at(0) + (peer.goal.at(0) - peer.start.at(0)) * ratio,
            peer.start.at(1) + (peer.goal.at(1) - peer.start.at(1)) * ratio,
            peer.start.at(2) + (peer.goal.at(2) - peer.start.at(2)) * ratio});
    }
    return BsplineTrajectory(control_points, peer.knot_interval_s);
}

bool build_peer_records(
    const SimulationInput& input,
    std::vector<PeerTrajectory>& records,
    std::string& error)
{
    records.clear();
    for (const PeerDefinition& definition : input.peers)
    {
        PeerTrajectory record;
        record.agent_id = definition.agent_id;
        record.trajectory_id = 1U;
        record.frame_revision = FRAME_REVISION;
        record.message_time_s = SIMULATION_TIME_S;
        record.start_time_s = SIMULATION_TIME_S;
        record.receive_time_s = SIMULATION_TIME_S;
        record.terminal_hold = definition.terminal_hold;
        record.trajectory = make_peer_trajectory(definition);
        record.end_time_s = record.trajectory.duration_s();
        if (!record.trajectory.is_valid() || record.end_time_s <= 0.0)
        {
            error = "invalid peer trajectory for agent " +
                std::to_string(definition.agent_id);
            return false;
        }
        records.push_back(record);
    }
    return true;
}

bool build_obstacle_map(
    const SimulationInput& input,
    RawPointObstacleMap& map,
    std::string& error)
{
    if (!map.set_freshness_timeout(1.0))
    {
        error = "cannot configure obstacle map freshness";
        return false;
    }
    if (input.obstacles.empty())
    {
        return map.set_explicit_empty(SIMULATION_TIME_S);
    }

    GenerationConfig config;
    config.mode = CloudMode::GroundTruth;
    config.sample_spacing_m = static_cast<float>(input.sample_spacing_m);
    try
    {
        const auto generated = ObstacleCloudGenerator::generate(
            input.obstacles, config);
        std::vector<Vector3> points;
        points.reserve(generated.first.points.size());
        for (const auto& point : generated.first.points)
        {
            points.push_back({
                static_cast<double>(point.at(0)),
                static_cast<double>(point.at(1)),
                static_cast<double>(point.at(2))});
        }
        if (!map.update(points, SIMULATION_TIME_S))
        {
            error = "cannot update obstacle map";
            return false;
        }
        return true;
    }
    catch (const std::exception& exception)
    {
        error = std::string("obstacle generation failed: ") +
            exception.what();
        return false;
    }
}

void write_vector_json(
    std::ostream& output,
    const Vector3& value)
{
    output << '[' << value.at(0) << ',' << value.at(1) << ',' <<
        value.at(2) << ']';
}

void write_trajectory_samples_json(
    std::ostream& output,
    const BsplineTrajectory& trajectory,
    const double sample_interval_s)
{
    output << '[';
    if (!trajectory.is_valid())
    {
        output << ']';
        return;
    }
    const double duration = trajectory.duration_s();
    const double interval = std::max(sample_interval_s, 0.01);
    const std::size_t count = std::min<std::size_t>(
        1000U,
        static_cast<std::size_t>(std::ceil(duration / interval)) + 1U);
    for (std::size_t index = 0U; index < count; ++index)
    {
        const double time_s = std::min(
            duration, static_cast<double>(index) * interval);
        const auto sample = trajectory.evaluate_state(time_s);
        if (index > 0U)
        {
            output << ',';
        }
        output << "{\"t\":" << time_s << ",\"position\":";
        write_vector_json(output, sample.position);
        output << ",\"velocity\":";
        write_vector_json(output, sample.velocity);
        output << '}';
    }
    output << ']';
}

const char* path_constraint_name(const PathConstraint constraint)
{
    switch (constraint)
    {
        case PathConstraint::FREE_3D:
            return "free_3d";
        case PathConstraint::FIXED_Z:
            return "fixed_z";
        case PathConstraint::LINEAR_Z:
            return "linear_z";
    }
    return "unknown";
}

std::string termination_name(
    const taiphoon::ego_swarm::PlannerTerminationReason reason)
{
    return taiphoon::ego_swarm::planner_termination_reason_string(reason);
}

void write_output_json(
    const SimulationInput& input,
    const SimulationOutput& output)
{
    const PlannerResult& plan = output.planner_result;
    const SafetyValidationResult& safety = output.safety_result;
    std::cout << std::setprecision(10);
    std::cout << "{\"ok\":" << (output.error.empty() ? "true" : "false");
    if (!output.error.empty())
    {
        std::cout << ",\"error\":\"";
        for (const char character : output.error)
        {
            if (character == '\\' || character == '"')
            {
                std::cout << '\\';
            }
            std::cout << character;
        }
        std::cout << "\"}" << std::endl;
        return;
    }

    std::cout << ",\"input\":{\"goal\":";
    write_vector_json(std::cout, input.goal);
    std::cout << ",\"path_constraint\":\"" <<
        path_constraint_name(input.path_constraint) << "\"}"
              << ",\"planner\":{\"is_valid\":" <<
        (plan.is_valid ? "true" : "false") << ",\"is_hold\":" <<
        (plan.is_hold ? "true" : "false") << ",\"termination\":\"" <<
        termination_name(plan.termination) << "\",\"planning_time_ms\":" <<
        plan.planning_time_ms << ",\"iteration_count\":" <<
        plan.iteration_count << ",\"duration_s\":" <<
        plan.trajectory.duration_s() << ",\"cost\":" << plan.cost <<
        ",\"goal_cost\":" << plan.goal_cost <<
        ",\"smoothness_cost\":" << plan.smoothness_cost <<
        ",\"dynamic_cost\":" << plan.dynamic_cost <<
        ",\"obstacle_cost\":" << plan.obstacle_cost <<
        ",\"peer_cost\":" << plan.peer_cost << "}"
              << ",\"safety\":{\"is_safe\":" <<
        (safety.is_safe ? "true" : "false") << ",\"reason\":\"" <<
        ContinuousSafetyValidator::reason_string(safety.reason) <<
        "\",\"minimum_obstacle_distance_m\":" <<
        safety.minimum_obstacle_distance_m <<
        ",\"minimum_peer_distance_m\":" <<
        safety.minimum_peer_distance_m <<
        ",\"maximum_velocity_mps\":" << safety.maximum_velocity_mps <<
        ",\"maximum_acceleration_mps2\":" <<
        safety.maximum_acceleration_mps2 <<
        ",\"maximum_jerk_mps3\":" << safety.maximum_jerk_mps3 << '}';

    std::cout << ",\"control_points\":[";
    for (std::size_t index = 0U;
         index < plan.trajectory.control_points().size(); ++index)
    {
        if (index > 0U)
        {
            std::cout << ',';
        }
        write_vector_json(std::cout,
                          plan.trajectory.control_points().at(index));
    }
    std::cout << "],\"samples\":";
    write_trajectory_samples_json(
        std::cout, plan.trajectory, input.planner_config.sample_interval_s);

    std::cout << ",\"peers\":[";
    std::vector<PeerTrajectory> peer_records;
    std::string peer_error;
    (void)build_peer_records(input, peer_records, peer_error);
    for (std::size_t index = 0U; index < peer_records.size(); ++index)
    {
        if (index > 0U)
        {
            std::cout << ',';
        }
        const PeerTrajectory& peer = peer_records.at(index);
        std::cout << "{\"agent_id\":" << peer.agent_id <<
            ",\"terminal_hold\":" <<
            (peer.terminal_hold ? "true" : "false") <<
            ",\"duration_s\":" << peer.trajectory.duration_s() <<
            ",\"samples\":";
        write_trajectory_samples_json(
            std::cout, peer.trajectory, input.planner_config.sample_interval_s);
        std::cout << '}';
    }
    std::cout << "]}" << std::endl;
}

bool run_simulation(
    const SimulationInput& input,
    SimulationOutput& output)
{
    RawPointObstacleMap obstacle_map;
    std::string error;
    if (!build_obstacle_map(input, obstacle_map, error))
    {
        output.error = error;
        return false;
    }

    std::vector<PeerTrajectory> peers;
    if (!build_peer_records(input, peers, error))
    {
        output.error = error;
        return false;
    }

    PlannerConfig config = input.planner_config;
    if (config.max_velocity_m_s <= 0.0)
    {
        config.max_velocity_m_s = config.max_velocity_mps;
    }
    if (config.max_acceleration_m_s2 <= 0.0)
    {
        config.max_acceleration_m_s2 = config.max_acceleration_mps2;
    }

    PlanningSnapshot snapshot;
    snapshot.self_state = input.self_state;
    snapshot.local_goal = input.goal;
    snapshot.planning_time_s = SIMULATION_TIME_S;
    snapshot.trajectory_start_time_s = SIMULATION_TIME_S;
    snapshot.frame_revision = FRAME_REVISION;
    snapshot.self_agent_id = 1;
    snapshot.path_constraint = input.path_constraint;
    snapshot.fixed_z_m = input.goal.at(2);
    snapshot.peers = peers;
    snapshot.obstacle_map = &obstacle_map;
    snapshot.peers_are_fresh = true;

    OptimizedEgoPlanner planner(config);
    output.planner_result = planner.plan(snapshot);

    PeerTrajectoryTableConfig table_config;
    table_config.timeout_s = 10.0;
    table_config.max_peer_time_skew_s = 1.0;
    PeerTrajectoryTable peer_table(table_config);
    for (const PeerTrajectory& peer : peers)
    {
        if (!peer_table.update(peer))
        {
            output.error = "cannot update peer trajectory table";
            return false;
        }
    }

    ContinuousSafetyValidator validator(config);
    SafetyValidationContext context;
    context.self_state = input.self_state;
    context.now_s = SIMULATION_TIME_S;
    context.trajectory_start_time_s = SIMULATION_TIME_S;
    context.max_time_skew_s = MAX_TIME_SKEW_S;
    context.max_future_start_delay_s = 1.0;
    context.frame_revision = FRAME_REVISION;
    context.trajectory_frame_revision = FRAME_REVISION;
    context.self_agent_id = 1;
    context.peer_table = &peer_table;
    context.obstacle_map = &obstacle_map;
    if (output.planner_result.trajectory.is_valid())
    {
        output.safety_result = validator.validate(
            output.planner_result.trajectory, context);
    }
    else
    {
        output.safety_result.reason =
            taiphoon::ego_swarm::SafetyFailureReason::NONFINITE_TRAJECTORY;
    }
    output.peers = input.peers;
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc > 1 && std::string(argv[1]) != "--stdin")
    {
        std::cerr << "usage: planning_simulator --stdin" << std::endl;
        return 2;
    }

    SimulationInput input;
    std::string error;
    if (!parse_input(std::cin, input, error))
    {
        SimulationOutput output;
        output.error = error;
        write_output_json(input, output);
        return 1;
    }

    SimulationOutput output;
    if (!run_simulation(input, output))
    {
        write_output_json(input, output);
        return 1;
    }
    write_output_json(input, output);
    return 0;
}
