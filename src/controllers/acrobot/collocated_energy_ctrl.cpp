#include "drake_control_algorithms/controllers/acrobot/collocated_energy_ctrl.hpp"


namespace underact_robotics::controllers::acrobot {

Eigen::Vector4d CollocatedEnergyCtrl::get_equilibrium_state()
{
    return eq_;
}

Eigen::Matrix4d CollocatedEnergyCtrl::get_cost_matrix()
{
    return S_;
}

void CollocatedEnergyCtrl::set_config(const CollocatedEnergyCtrlConfig& cfg)
{
    config_ = cfg;
}

CollocatedEnergyCtrl::CollocatedEnergyCtrl(const CollocatedEnergyCtrlConfig& cfg) : config_{cfg}
{
    SetUpPlantAndLqr();
    E_eq_ =
            plant_->EvalKineticEnergy(*context_) + plant_->EvalPotentialEnergy(*context_);
    
    // kinetic energy should be zero
    //DeclareContinuousState(4);
   
    state_port_idx_ = this->DeclareVectorInputPort("state_input", 4).get_index();
    control_port_idx_ = this->DeclareVectorOutputPort(
                                                        "torque_output",
                                                        1,
                                                        &CollocatedEnergyCtrl::CalcControlTorque
                                                     ).get_index();

}


void CollocatedEnergyCtrl::SetUpPlantAndLqr() {
    // load model
    DRAKE_DEMAND(config_.dt >= 0.0);
    plant_ = std::make_unique<drake::multibody::MultibodyPlant<double>>(config_.dt);
    drake::multibody::Parser parser(plant_.get());
    try {
        
        parser.AddModels(config_.urdf);

    } catch (std::exception& e) {
        // Log and shut down.
        std::cout << "Parsing error: [" << e.what() << " ]" << std::endl;
    }
    plant_->Finalize();
    context_= plant_->CreateDefaultContext();
    
    DRAKE_DEMAND(config_.equilibriumState.size() == 4);
    for(size_t i = 0; i < 4; i++) {
        eq_(i) = config_.equilibriumState[i];
    }
    plant_->SetPositionsAndVelocities(context_.get(), eq_);
    
    DRAKE_DEMAND(config_.equilibriumInput.size() == 1);
    plant_->get_actuation_input_port().FixValue(context_.get(), config_.equilibriumInput.at(0));

    b1_ = plant_->GetJointByName("shoulder").default_damping_vector()[0];
    b2_ = plant_->GetJointByName("elbow").default_damping_vector()[0];
    DRAKE_DEMAND(b1_>=0.0 && b2_>=0.0);

    try {
        
        auto lin_sys = drake::systems::Linearize(
                                                    *plant_,
                                                    *context_,
                                                    plant_->get_actuation_input_port().get_index(),
                                                    plant_->get_state_output_port().get_index()
                                                );

        DRAKE_DEMAND(config_.lqrQ.size() == 16);
        size_t iq = 0;
        Eigen::Matrix4d Q;
        for(size_t r = 0; r < 4; r++) {
            for(size_t c = 0; c < 4; c++) {
                Q(r,c) = config_.lqrQ.at(iq);
                iq++;
            }
        }

        DRAKE_DEMAND(config_.lqrR.size() == 1);
        Eigen::Matrix<double, 1, 1> R{config_.lqrR.at(0)};

        auto [K, S] = drake::systems::controllers::LinearQuadraticRegulator(
                                                                             lin_sys->A(),
                                                                             lin_sys->B(),
                                                                             Q,
                                                                             R
                                                                           );

        K_ = K;
        S_ = S;
    
    } catch (std::exception& e) {
        std::cout << "Failed to linearize or synthesize: " << e.what() << std::endl;
        
    }

    DRAKE_DEMAND(config_.controllerGains.size() == 3);

    if(config_.jointTorqueLimits.has_value()) {
        DRAKE_DEMAND(config_.jointTorqueLimits->size() == 1);
        DRAKE_DEMAND(config_.jointTorqueLimits.value().at(0) > 0.0);
    }

    DRAKE_DEMAND(config_.saturation >= 0.0);

    //generator_.seed(12345);// add config 
    if(config_.stddev > 0.0) {
        sensor_noise_ = std::make_unique<std::normal_distribution<double>>(config_.mean, config_.stddev);
    }

    
}

double CollocatedEnergyCtrl::ComputeTorque(const Eigen::Vector4d& state) const
{
         
    Eigen::Vector4d error = state-eq_;
    
    error(0) = drake::math::wrap_to(error(0), -std::numbers::pi, std::numbers::pi);
    error(1) = drake::math::wrap_to(error(1), -std::numbers::pi, std::numbers::pi);
    
    double cost{error.dot(S_*error)};
    double u{0.0};
  
    if (cost < config_.cost) {
        
       u = (-K_*error)(0);
    
    } else {
    
        Eigen::Matrix<double, 2, 2> M;
        Eigen::Vector2d Cv;
        Eigen::Vector2d tau_g;    
       
        plant_->CalcMassMatrix(*context_, &M);
        plant_->CalcBiasTerm(*context_, &Cv);
        tau_g = plant_->CalcGravityGeneralizedForces(*context_);
        
        double E = plant_->EvalPotentialEnergy(*context_) + plant_->EvalKineticEnergy(*context_);
        
        double M21_M11_inv = M(1,0)/M(0,0);
        
        double u_pfl  = -(tau_g(1) - M21_M11_inv*tau_g(0)) +
                            -M21_M11_inv*(Cv(0)+b1_*state(2)) + Cv(1)+b2_*state(3);

        double sat_energy = std::clamp(config_.controllerGains[2]*(E - E_eq_)*state(2),
                                        -config_.saturation,
                                         config_.saturation);

                    
        u = u_pfl - config_.controllerGains[0]*(error(1)) 
                  - config_.controllerGains[1]*state(3) 
                  + sat_energy;
    }
    

    if(config_.jointTorqueLimits.has_value()) {
        u = std::clamp(u, -config_.jointTorqueLimits.value().at(0), 
                           config_.jointTorqueLimits.value().at(0));
    } 

    return u;
}

void CollocatedEnergyCtrl::CalcControlTorque(const drake::systems::Context<double>& context,
                                                   drake::systems::BasicVector<double>* output) const
{
    
   
    
    const Eigen::Vector4d state = get_input_port(0).Eval(context);
    auto tmp = state;
    
    //add noise
    if(sensor_noise_) {
        tmp(0) += (*sensor_noise_)(generator_);
        tmp(1) += (*sensor_noise_)(generator_);
        tmp(2) += (*sensor_noise_)(generator_);
        tmp(3) += (*sensor_noise_)(generator_);
    }

    tmp(0) = drake::math::wrap_to(tmp(0), -std::numbers::pi, std::numbers::pi);
    tmp(1) = drake::math::wrap_to(tmp(1), -std::numbers::pi, std::numbers::pi);

  
    plant_->SetPositionsAndVelocities(context_.get(), tmp); 
    double u =  ComputeTorque(tmp);
    output->SetAtIndex(0, u);
}


/*
CollocatedEnergyCtrl::
CollocatedEnergyCtrl::
CollocatedEnergyCtrl::
CollocatedEnergyCtrl::
CollocatedEnergyCtrl::
*/




} // underact_robotics::controllers::acrobot
