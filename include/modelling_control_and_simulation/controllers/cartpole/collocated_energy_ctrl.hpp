#ifndef CARTPOLE_COLLOCATED_ENERGY_CTRL_H_
#define CARTPOLE_COLLOCATED_ENERGY_CTRL_H_

#include <optional>
#include <memory>
#include <iostream>
#include <numbers>
#include <algorithm>

#include "yaml-cpp/yaml.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/math/wrap_to.h"
#include "drake/common/drake_assert.h"
#include "drake/common/random.h"

#include "modelling_control_and_simulation/utils/parameters.hpp"

namespace modelling_control_and_simulation::controllers::cartpole {


struct CollocatedEnergyCtrlConfig {
     
    std::string urdf{"../models/cartpole/cartpole.urdf"};
    
    double dt{0.0};
    double cost{10.0}; //determine when to switch
    double saturation{10.0}; // staturate energy term
    double mean{-1.0}; // sensor noise
    double stddev{-1.0}; // sensor noise
   
     
    // Linearize around this point and generate an LQR controller
    std::vector<double> equilibriumState{0.0, std::numbers::pi, 0.0,0.0};
    std::vector<double> equilibriumInput{0.0};
    std::vector<double> controllerGains{10.0, 4.0, 1.0};
    std::vector<double> lqrQ{
                              1.0, 0.0, 0.0, 0.0,
                              0.0, 1.0, 0.0, 0.0,
                              0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 1.0 
                            }; //row major

    std::vector<double> lqrR{1.0}; // row major
                           
    // Optional limits
    std::optional<std::vector<double>> jointTorqueLimits =  std::nullopt;
    std::optional<std::vector<double>> jointPositionLimits =  std::nullopt;
    std::optional<std::vector<double>> jointVelocityLimits =  std::nullopt;
    std::optional<std::vector<double>> jointAccelerationLimits = std::nullopt;

    std::string CfgToString() {
        
        std::string s{"Controller cfg\n"};
        s += std::string{parameter::names::urdf} = ": " + urdf + "\n";
        s += std::string{parameter::names::dt} + ": " + std::to_string(dt) + "\n";
        s += std::string{parameter::names::cost} + ": " + std::to_string(cost) + "\n";
        s += std::string{parameter::names::mean} + ": " + std::to_string(mean) + "\n";
        s += std::string{parameter::names::stddev} + ": " + std::to_string(stddev) + "\n";
        s += std::string{parameter::names::saturation} + ": " + std::to_string(saturation) + "\n";
        
        auto vToString = [](const std::vector<double>& v) -> std::string {
            std::string s{};
            for(size_t i = 0; i < v.size(); i++) {
                s += std::to_string(v.at(i)) + ", ";
            }
            s += "\n";
            return s;
        };
        
        s += std::string{parameter::names::equilibriumState} + ": " + vToString(equilibriumState);
        s += std::string{parameter::names::equilibriumInput} + ": " + vToString(equilibriumInput);
        s += std::string{parameter::names::controllerGains}  + ": " + vToString(controllerGains);
        
        if(jointTorqueLimits) {
            s += std::string{parameter::names::jointTorqueLimits} + ": " + vToString(jointTorqueLimits.value());
        }
        if(jointPositionLimits) {
            s += std::string{parameter::names::jointPositionLimits} + ": " + vToString(jointPositionLimits.value());
        }
        if(jointVelocityLimits) {
            s += std::string{parameter::names::jointVelocityLimits} + ": " + vToString(jointVelocityLimits.value());
        }
        if(jointAccelerationLimits) {
            s += std::string{parameter::names::jointAccelerationLimits} + ": " + vToString(jointAccelerationLimits.value());
        }
               
        return s;
    }


};


class CollocatedEnergyCtrl : public drake::systems::LeafSystem<double> {
    
    public: 
        CollocatedEnergyCtrl(const CollocatedEnergyCtrlConfig& cfg);

        Eigen::Vector4d get_equilibrium_state();
        Eigen::Matrix4d get_cost_matrix();

        void set_config(const CollocatedEnergyCtrlConfig& config_);

    private:
        void SetUpPlantAndLqr();
        double ComputeForce(const Eigen::Vector4d& state) const;
        void CalcControlForce(const drake::systems::Context<double>& context,
                                drake::systems::BasicVector<double>* output) const;
        
        CollocatedEnergyCtrlConfig config_;
        
        std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
        std::unique_ptr<drake::systems::Context<double>> context_;
        
        Eigen::Matrix<double, 1, 4> K_;
        Eigen::Matrix4d S_;
        Eigen::Vector4d eq_;

        double E_eq_;
        double b1_;
        double b2_;
         
        mutable drake::RandomGenerator generator_; 
        std::unique_ptr<std::normal_distribution<double>> sensor_noise_;
};

} // modelling_control_and_simulation::controllers::cartpole

#endif
