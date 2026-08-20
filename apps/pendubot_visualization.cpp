#include <iostream>
#include <cmath>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/analysis/simulator.h"

#include "modelling_control_and_simulation/controllers/pendubot/collocated_energy_ctrl.hpp"

using namespace modelling_control_and_simulation::controllers::pendubot;

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


int main()
{
    
    experiment::Config exp_cfg;
    parseExperimentConfig(exp_cfg, "../configs/pendubot/experiment.yaml");
    
    drake::systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] = 
        drake::multibody::AddMultibodyPlantSceneGraph<double>(&builder, 0.0);
    
    drake::multibody::Parser parser(&plant);
    parser.AddModels("../models/pendubot/pendubot_plant.urdf");
    plant.Finalize();

    drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

    CollocatedEnergyCtrlConfig cfg;
    parseControlConfig(cfg, "../configs/pendubot/collocated_energy_ctrl.yaml");
    auto controller = builder.AddSystem<CollocatedEnergyCtrl>(cfg);

    builder.Connect(plant.get_state_output_port(), controller->get_input_port());
    builder.Connect(controller->get_output_port(), plant.get_actuation_input_port());


    auto diagram = builder.Build();
    drake::systems::Simulator<double> simulator(std::move(diagram));

    auto& plant_context = plant.GetMyMutableContextFromRoot(
            &simulator.get_mutable_context());

    std::vector<double> v = exp_cfg.initState;
    Eigen::Vector4d x0{v[0], v[1] , v[2], v[3]};
    plant.SetPositionsAndVelocities(&plant_context, x0);

    simulator.set_target_realtime_rate(1.0);
    simulator.AdvanceTo(30.0);
    
        
    
    return 0;
}
