#include <string>
#include <memory>
#include <fstream>
#include <iostream>
#include <bitset>
#include <random>
#include <vector>
#include <cmath>
#include <cassert>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/sensors/rotary_encoders.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/random_source.h"
#include "drake/geometry/drake_visualizer.h"
#include "yaml-cpp/yaml.h"

#include "modelling_control_and_simulation/controllers/acrobot/collocated_energy_ctrl.hpp"

using namespace modelling_control_and_simulation::controllers::acrobot;

    
void parseControlConfig(CollocatedEnergyCtrlConfig& cfg, const std::string& path)
{
    YAML::Node config = YAML::LoadFile(path)["/collocated_energy_ctrl"]["ros__parameters"];

    cfg.dt = config[parameter::names::dt].as<double>();
    cfg.cost = config[parameter::names::cost].as<double>();
    cfg.saturation = config[parameter::names::saturation].as<double>();
    cfg.equilibriumState = config[parameter::names::equilibriumState].as<std::vector<double>>();
    cfg.equilibriumInput = config[parameter::names::equilibriumInput].as<std::vector<double>>();
    cfg.controllerGains = config[parameter::names::controllerGains].as<std::vector<double>>();
    cfg.lqrQ = config[parameter::names::lqrQ].as<std::vector<double>>();
    cfg.lqrR = config[parameter::names::lqrR].as<std::vector<double>>();
    cfg.urdf = config[parameter::names::urdf].as<std::string>();
    cfg.jointTorqueLimits =
        config[parameter::names::jointTorqueLimits].as<std::vector<double>>();
    cfg.mean = config[parameter::names::mean].as<double>();
    cfg.stddev = config[parameter::names::stddev].as<double>();
    
  
}

void parseExperimentConfig(experiment::Config& cfg, const std::string& path)
{
    YAML::Node config = YAML::LoadFile(path)["acrobot"];

    cfg.dt = config[parameter::names::dt].as<double>();
    cfg.boundaryTime = config[parameter::names::boundaryTime].as<double>();
    cfg.realtimeTargetRate = config[parameter::names::realtimeTargetRate].as<double>();
    cfg.runs = config[parameter::names::runs].as<size_t>();
    cfg.initState = config[parameter::names::initState].as<std::vector<double>>();
    cfg.urdf = config[parameter::names::urdf].as<std::string>();
    cfg.log = config[parameter::names::log].as<std::string>();
    cfg.notes = config[parameter::names::notes].as<std::string>();
    cfg.zeroOrderHoldSec = config[parameter::names::zeroOrderHoldSec].as<double>();
    cfg.zeroOrderHoldOffset = config[parameter::names::zeroOrderHoldOffset].as<double>();
    cfg.ticksPerRevolution = config[parameter::names::ticksPerRevolution].as<int>();
    cfg.eqErrorTol= config[parameter::names::eqErrorTol].as<double>();
    cfg.timeTol = config[parameter::names::timeTol].as<double>();
    cfg.nVariables = config[parameter::names::nVariables].as<int>();
    cfg.populationSize = config[parameter::names::populationSize].as<int>();
    cfg.mutationRate = config[parameter::names::mutationRate].as<double>();
    cfg.crossOverRate = config[parameter::names::crossOverRate].as<double>();
    cfg.tournamentSelectionRate = config[parameter::names::tournamentSelectionRate].as<double>();
    cfg.maxGain = config[parameter::names::maxGain].as<double>(); 
}


int main(int argc, char **argv)
{
        
    std::string ctrl_yaml = "../configs/acrobot/collocated_energy_ctrl.yaml";
    CollocatedEnergyCtrlConfig ctrl_cfg;
    parseControlConfig(ctrl_cfg, ctrl_yaml); 

    std::string exp_yaml = "../configs/acrobot/experiment.yaml";
    experiment::Config exp_cfg;
    parseExperimentConfig(exp_cfg, exp_yaml);
        
    drake::systems::DiagramBuilder<double> builder;

    auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(&builder, exp_cfg.dt);
 
    drake::multibody::Parser parser{&plant};
    parser.AddModels(exp_cfg.urdf);
    plant.Finalize();

    drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
    
    auto controller = builder.AddSystem<CollocatedEnergyCtrl>(ctrl_cfg);
    int ticks = exp_cfg.ticksPerRevolution;
    auto state_rotary_encoder = builder.AddSystem<drake::systems::sensors::RotaryEncoders<double>>(4, 
                                                                                  std::vector<int>{0,1,2,3},
                                                                std::vector<int>{ticks, ticks, ticks, ticks});

    auto state_zoh = builder.AddSystem<drake::systems::ZeroOrderHold<double>>(exp_cfg.zeroOrderHoldSec, 4,
                                                                              exp_cfg.zeroOrderHoldOffset);
    
    builder.Connect(plant.get_state_output_port(), state_zoh->get_input_port());
    builder.Connect(state_zoh->get_output_port(), state_rotary_encoder->get_input_port());
    builder.Connect(state_rotary_encoder->get_output_port(), controller->get_input_port());
    builder.Connect(controller->get_output_port(), plant.get_actuation_input_port());

    auto state_logger = drake::systems::LogVectorOutput<double>(plant.get_state_output_port(), &builder);// actual state
    auto u_logger = drake::systems::LogVectorOutput<double>(controller->get_output_port(), &builder);
 
    auto diagram = builder.Build();
    auto root_context = diagram->CreateDefaultContext();   
    auto& plant_context = plant.GetMyMutableContextFromRoot(root_context.get());
    
 
    std::vector<double> v = exp_cfg.initState;
    Eigen::Vector4d x0{v[0], v[1], v[2], v[3]};
    plant.SetPositionsAndVelocities(&plant_context, x0);
  
    drake::systems::Simulator<double> simulator{*diagram, std::move(root_context)};
     
    simulator.set_target_realtime_rate(exp_cfg.realtimeTargetRate);

    simulator.AdvanceTo(exp_cfg.boundaryTime);
    /*
    auto state_data_log = state_logger->FindLog(simulator.get_context());
    auto state_data = state_data_log,data();
    auto u_data_log = u_logger->FindLog(simulator.get_context());
    auto u_data = u_data_log.data();
    auto time = u_data_log.sample_times();    
    
    
    //save to csv file.
    std::ofstream csv_file(exp_cfg.log); 
    csv_file << exp_cfg.CfgToString() << ctrl_cfg.CfgToString();
 
          
    csv_file << "\nt q1 q2 q1d q2d u error cost\n";

    Eigen::Matrix4d S = controller->get_cost_matrix();
    Eigen::Vector4d eq = controller->get_equilibrium_state();
    Eigen::Vector4d tmp;
    Eigen::Vector4d err;
    double cost{0.0};
    
    for (size_t col = 0; col < state_data.cols(); col++) {
        csv_file << time(col) << ", ";
        err = state_data.col(col) - eq;
        err(0) = drake::math::wrap_to(err(0), -std::number::pi, std::number::pi);
        err(1) = drake::math::wrap_to(err(1), -std::number::pi, std::number::pi);
        cost = err.dot(S*err);

        for (int row = 0; row < state_data.rows(); row++) {
            csv_file << state_data(row, col) << ",";
        }
        csv_file << " " << u_data(col) << ", " << 
        csv_file << "\n";
    }
    */
    
    return 0;
}
