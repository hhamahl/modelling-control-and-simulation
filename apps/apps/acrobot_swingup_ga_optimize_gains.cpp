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
#include "yaml-cpp/yaml.h"

#include "modelling_control_and_simulation/controllers/acrobot/collocated_energy_ctrl.hpp"
#include "modelling_control_and_simulation/utils/genetic_algorithms.hpp"

using namespace modelling_control_and_simulation::controllers::acrobot;
using namespace genetic_algorithms; 

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

     // Run experiment
    std::vector<Individual<10>> best_from_each_run;

    //init population
    Population<10> population(exp_cfg.populationSize, exp_cfg.nVariables);
        
    // Set up environment.
    drake::systems::DiagramBuilder<double> builder;

    auto plant = builder.AddSystem<drake::multibody::MultibodyPlant<double>>(exp_cfg.dt);
    drake::multibody::Parser parser{plant};
    parser.AddModels(exp_cfg.urdf);
    plant->Finalize();
    
    auto controller = builder.AddSystem<CollocatedEnergyCtrl>(ctrl_cfg); // give a path instead
    /* 
    int ticks = exp_cfg.ticksPerRevolution;
    auto state_rotary_encoder = builder.AddSystem<drake::systems::sensors::RotaryEncoders<double>>(4, 
                                                                                  std::vector<int>{0,1,2,3},
                                                                std::vector<int>{ticks, ticks, ticks, ticks});

    auto state_zoh = builder.AddSystem<drake::systems::ZeroOrderHold<double>>(exp_cfg.zeroOrderHoldSec, 4,
                                                                              exp_cfg.zeroOrderHoldOffset);
    
    builder.Connect(plant->get_state_output_port(), state_zoh->get_input_port());
    builder.Connect(state_zoh->get_output_port(), state_rotary_encoder->get_input_port());
    builder.Connect(state_rotary_encoder->get_output_port(), controller->get_input_port());
    */
    builder.Connect(plant->get_state_output_port(),controller->get_input_port());
    builder.Connect(controller->get_output_port(), plant->get_actuation_input_port());
    
    //auto state_logger = drake::systems::LogVectorOutput<double>(plant->get_state_output_port(), &builder);// actual state
    
    auto diagram = builder.Build();
    auto root_context = diagram->CreateDefaultContext();   
    auto& plant_context = plant->GetMyMutableContextFromRoot(root_context.get());
    
    std::vector<double> v = exp_cfg.initState;
    Eigen::Vector4d x0{v[0], v[1], v[2], v[3]};
    plant->SetPositionsAndVelocities(&plant_context, x0);
  
    drake::systems::Simulator<double> simulator{*diagram, std::move(root_context)};
     
    simulator.set_target_realtime_rate(exp_cfg.realtimeTargetRate);

    double t_in_eq{0.0};
    double current_time{0.0};
    double last_time{0.0};
    
    
    simulator.set_monitor([&plant, &controller, &exp_cfg, &t_in_eq, &current_time, &last_time](const drake::systems::Context<double>& context) {
        
        auto& plant_context = plant->GetMyContextFromRoot(context);
        auto state_cpy = plant->GetPositionsAndVelocities(plant_context);
        Eigen::Vector4d state = state_cpy;
        
        state(0) = drake::math::wrap_to(state(0), -std::numbers::pi, std::numbers::pi);
        state(1) = drake::math::wrap_to(state(1), -std::numbers::pi, std::numbers::pi);
        auto eq = controller->get_equilibrium_state();
        auto S = controller->get_cost_matrix();

        Eigen::Vector4d error = state-eq;

        error(0) = drake::math::wrap_to(error(0), -std::numbers::pi, std::numbers::pi);
        error(1) = drake::math::wrap_to(error(1), -std::numbers::pi, std::numbers::pi);
        
        last_time = current_time;
        current_time = context.get_time();

        double cost{error.dot(S*error)};
        
        if (cost < exp_cfg.eqErrorTol) {

            t_in_eq += (current_time - last_time);
        } else {
            t_in_eq = 0.0;
        }
        if(t_in_eq > exp_cfg.timeTol) {
            return drake::systems::EventStatus::ReachedTermination(plant,
        "Simulation achieved the desired goal.");        }

        return drake::systems::EventStatus::DidNothing();
    });
    
    std::random_device r;
    std::default_random_engine el(r());
    std::uniform_int_distribution<int> uniform_dist(0, 1);
    std::uniform_int_distribution<int> selection_dist(0, population.n -1);
    std::uniform_int_distribution<int> cross_point(0, 10*exp_cfg.nVariables);
    std::uniform_real_distribution<double> uniform_real_dist(0, 1);
    
    std::cout << "Running GA!" << std::endl;
    
    for(size_t i = 0; i < exp_cfg.runs; i++) {
        //Evaluate individuals i.e. controllers
        best_from_each_run.push_back(population.individuals[0]);
        for(auto& individual : population.individuals) {
        
            t_in_eq = 0.0;
            current_time = 0.0;
            last_time = 0.0;
            
            auto& sim_context =  simulator.get_mutable_context();
            plant->SetPositionsAndVelocities(&plant->GetMyMutableContextFromRoot(&sim_context), x0);
            ctrl_cfg.controllerGains = individual.DecodeGrayChromosome(exp_cfg.maxGain);
            for(auto& g : ctrl_cfg.controllerGains) {
                g += exp_cfg.maxGain; //only positive gains

            }                
            controller->set_config(ctrl_cfg);

            simulator.Initialize();
            simulator.AdvanceTo(exp_cfg.boundaryTime);
            individual.fitness = exp_cfg.boundaryTime/(simulator.get_context().get_time()) - 1;
            
            if(individual.fitness > best_from_each_run[i].fitness) {
                best_from_each_run[i] = individual;
            }
        }

        // New population
        std::vector<Individual<10>> new_pop;
        int count = 0;
        while(count < population.n) {
            std::pair<int, int> pairInd = population.Tournament(exp_cfg.tournamentSelectionRate,
                                                                el,
                                                                selection_dist,
                                                                uniform_real_dist);

            std::pair<Individual<10>, Individual<10>> pair = population.CrossOver(pairInd, 
                                                                          exp_cfg.crossOverRate,
                                                                          el,
                                                                          uniform_real_dist,
                                                                          cross_point);
            pair.first.Mutate(exp_cfg.mutationRate,
                              el,
                              uniform_real_dist);
            pair.second.Mutate(exp_cfg.mutationRate,
                              el,
                              uniform_real_dist);
            new_pop.push_back(pair.first);
            new_pop.push_back(pair.second);// New population will be even. 
            count += 2;
        }
        new_pop.shrink_to_fit();
        population.individuals.clear();
        population.individuals = new_pop; 
    }
    
    // print the best found gains
    int best = 0;
    int i = 0;
    double best_fit = -1.0;
    for(auto& ind : best_from_each_run) {
        if(ind.fitness > best_fit) {
            best = i; 
            best_fit = ind.fitness;
        }
        i++;
    }

    std::vector<double> best_gains = best_from_each_run[best].DecodeGrayChromosome(exp_cfg.maxGain);
    std::cout << "Best fitness: " << best_fit << "\n" << "Best gains: ";    
    for(auto gain : best_gains) {
        std::cout << (gain+exp_cfg.maxGain) << ", ";
    }
    std::cout << std::endl;

    /*save to csv file.
    std::ofstream csv_file(exp_cfg.log); 
    csv_file << exp_cfg.CfgToString() << ctrl_cfg.CfgToString();
 
    for(size_t run = 0; run < exp_yaml.runs; run++) { 
        
        
       
       
         // first, store parameters for this exp
        csv_file << "run: " << run << "\n";
        csv_file << "paramters\n";
        csv_file << // initState, gains, noise
        csv_file << "time\n";
        for (size_t i = 0; i < time.size(); i++) {
            csv_file << "," << i;
        }
        csv_file << "\nState\n";

        for (size_t col = 0; col < state_data.cols(); col++) {
            for (int row = 0; row < state_data.rows(); row++) {
                csv_file << "," << state_data(row, col);
            }
            csv_file << "\n";
        }

    }*/

    
    
    return 0;
}
