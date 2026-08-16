#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <variant>

namespace parameter::names {
    
    inline constexpr const char* dt = "dt";
    inline constexpr const char* simTimerPeriodSeconds = "sim_timer_period_seconds";
    inline constexpr const char* simRealtimeRate = "sim_realtime_rate";
    inline constexpr const char* ctrlPeriodSeconds = "ctrl_period_seconds";
    inline constexpr const char* ctrlCheckPeriodSeconds = "crtl_check_period_seconds";
    inline constexpr const char* statePubPeriodSeconds = "state_pub_period_seconds";
    inline constexpr const char* stateSubTopic = "state_sub_topic";
    inline constexpr const char* cost = "cost";
    inline constexpr const char* saturation = "saturation";
    inline constexpr const char* initState = "init_state";
    inline constexpr const char* equilibriumState = "equilibrium_state";
    inline constexpr const char* equilibriumInput = "equilibrium_input";
    inline constexpr const char* controllerGains = "controller_gains";
    inline constexpr const char* lqrQ = "lqr_Q";
    inline constexpr const char* lqrR = "lqr_R";
    inline constexpr const char* urdf = "urdf";
    inline constexpr const char* statePubTopic = "state_pub_topic";
    inline constexpr const char* ctrlSubTopic = "ctrl_sub_topic";
    inline constexpr const char* ctrlPubTopic = "ctrl_pub_topic";
    inline constexpr const char* mean = "mean";
    inline constexpr const char* stddev = "stddev";
    inline constexpr const char* jointTorqueLimits = "joint_torque_limits";
    inline constexpr const char* jointPositionLimits = "joint_position_limits";
    inline constexpr const char* jointVelocityLimits = "joint_velocity_limits";
    inline constexpr const char* jointAccelerationLimits = "joint_acceleration_limits";
    inline constexpr const char* nodeName = "node_name";
    inline constexpr const char* rosParameters = "ros__parameters";
    inline constexpr const char* boundaryTime = "boundary_time";
    inline constexpr const char* realtimeTargetRate = "realtime_target_rate";
    inline constexpr const char* log = "log";
    inline constexpr const char* runs = "runs";
    inline constexpr const char* notes = "notes";
    inline constexpr const char* zeroOrderHoldSec = "zero_order_hold_sec";
    inline constexpr const char* zeroOrderHoldOffset = "zero_order_hold_offset";
    inline constexpr const char* ticksPerRevolution = "ticks_per_revolution";
    inline constexpr const char* timeTol = "time_tol";
    inline constexpr const char* eqErrorTol = "eq_error_tol";
    inline constexpr const char* populationSize = "population_size";
    inline constexpr const char* nVariables = "n_variables";
    inline constexpr const char* mutationRate = "mutation_rate";
    inline constexpr const char* crossOverRate = "cross_over_rate";
    inline constexpr const char* tournamentSelectionRate = "tournament_selection_rate";
    inline constexpr const char* maxGain = "max_gain"; 


    /*
    inline constexpr const char* = "";
    inline constexpr const char* = "";
    inline constexpr const char* = "";
    inline constexpr const char* = "";
    inline constexpr const char* = "";
    inline constexpr const char* = "";
    */
        
} // parameter::names

namespace experiment {
    
    struct Config {
        
        double dt{0.0};
        double boundaryTime{10.0};
        double realtimeTargetRate{0.0};
        double zeroOrderHoldSec{0.0};
        double zeroOrderHoldOffset{0.0};
        double timeTol{1.0};
        double eqErrorTol{1.0};
        double mutationRate{1.0/30.0};
        double crossOverRate{0.8};
        double tournamentSelectionRate{0.75};
        double maxGain{10.0};

        int ticksPerRevolution{1000};
        int populationSize{2};
        int nVariables{3};


        size_t runs{1};

        std::vector<double> initState{};
        
        std::string urdf{};
        std::string log{};
        std::string notes{};

        
        std::string CfgToString() {
            std::string s{"Experiment cfg\n"};
            s += std::string{parameter::names::notes} + ": " + notes + "\n";
            s += std::string{parameter::names::urdf} + ": " + urdf + "\n";
            s += std::string{parameter::names::dt} + ": " + std::to_string(dt) + "\n";
            s += std::string{parameter::names::boundaryTime} + ": " + std::to_string(boundaryTime) + "\n";
            s += std::string{parameter::names::zeroOrderHoldSec} + ": " + std::to_string(zeroOrderHoldSec) + "\n";
            s += std::string{parameter::names::zeroOrderHoldOffset} + ": " + std::to_string(zeroOrderHoldOffset) + "\n";
            s += std::string{parameter::names::ticksPerRevolution} + ": " + std::to_string(ticksPerRevolution) + "\n";
            s += std::string{parameter::names::mutationRate} + ": " + std::to_string(mutationRate) + "\n";
            s += std::string{parameter::names::crossOverRate} + ": " + std::to_string(crossOverRate) + "\n";
            s += std::string{parameter::names::tournamentSelectionRate} + ": " + std::to_string(tournamentSelectionRate) + "\n";
            s += std::string{parameter::names::populationSize} + ": " + std::to_string(populationSize) + "\n";
            s += std::string{parameter::names::nVariables} + ": " + std::to_string(nVariables) + "\n";

            s += std::string{parameter::names::initState} + ": ";  
            for(size_t i = 0; i < initState.size(); i++) {
                s += std::to_string(initState.at(i)) + ", ";
            }
            s += "\n";
            return s;
        }
    };
}

namespace parameter {
    
    struct Parameters {
                   
            
            double dt{0.0};
            double simTimerPeriodSeconds{0.0025};
            double simRealtimeRate{1.0};
            double ctrlPeriodSeconds{0.0025};
            double ctrlCheckPeriodSeconds{0.005};
            double statePubPeriodSeconds{0.0025};
            double cost{10.0}; //determine when to switch
            double saturation{10.0}; // staturate energy term
            
            // Depends on robot and controller.
            std::vector<double> initState{};
            std::vector<double> equilibriumState{};
            std::vector<double> equilibriumInput{};
            std::vector<double> controllerGains{};
            std::vector<double> lqrQ{}; //row major
            std::vector<double> lqrR{}; // row major
                                        

            std::string urdf{"/drake_ws/src/underact_robotics/urdf/acrobot_drake.urdf"};
            std::string statePubTopic{"state"};
            std::string ctrlSubTopic{"state"};
            std::string ctrlPubTopic{"cmd"};
            std::string stateSubTopic{"state"};
                                    

            //optional
            std::optional<double> mean; // sensor noise.
            std::optional<double> stddev; // sensor noise. must be larger than 0.

            std::optional<std::vector<double>> jointTorqueLimits;
            std::optional<std::vector<double>> jointPositionLimits;
            std::optional<std::vector<double>> jointVelocityLimits; 
            std::optional<std::vector<double>> jointAccelerationLimits;
           
    
    };

} // parameter


#endif // PARAMETERS_H_
