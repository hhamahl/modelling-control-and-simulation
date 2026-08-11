#include <iostream>
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"

#include "drake_control_algorithms/controllers/acrobot/collocated_energy_ctrl.hpp"

using namespace underact_robotics::controllers::acrobot;


int main()
{
    std::string path = "/Users/hampusahlebrand/drake_ws/src/underact_robotics/parameter_files/collocated_energy_ctrl.yaml";
    YAML::Node config = YAML::LoadFile(path)["/collocated_energy_ctrl"]["ros__parameters"];
    
    CollocatedEnergyCtrlConfig cfg;
    
    cfg.dt = config[parameter::names::dt].as<double>();
    cfg.cost = config[parameter::names::cost].as<double>();
    cfg.saturation = config[parameter::names::saturation].as<double>();
    cfg.equilibriumState = config[parameter::names::equilibriumState].as<std::vector<double>>();
    cfg.equilibriumInput = config[parameter::names::equilibriumInput].as<std::vector<double>>();
    cfg.controllerGains = config[parameter::names::controllerGains].as<std::vector<double>>();
    cfg.lqrQ = config[parameter::names::lqrQ].as<std::vector<double>>();
    cfg.lqrR = config[parameter::names::lqrR].as<std::vector<double>>();
    
    std::cout << "All is well" << std::endl;
    return 0;
}
