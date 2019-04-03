//
//  fuelOpt_manipulatingFunctions.cpp
//  GTLC
//
//  Created by David Yang on 6/25/18.
//  Copyright © 2018 David Yang. All rights reserved.
//

#include <stdio.h>
#include "fuelOpt_DP.hpp"

// Function that calculates the new UAV_state as a result of the current UAV_state and action choice
UAV_state transition_UAV_state(UAV_state UAV_state_current, UAV UAV_current) {
    UAV_state UAV_state_new;
    std::vector <double> state_change                   =   {0,0};
    std::vector <double> state_new;
    std::vector <double> state_current                  =   UAV_state_current.getState();
    std::vector <double> action_current                 =   UAV_state_current.getAction();
    flight_segment segment_current                      =   UAV_state_current.getSegment();
    
    double m_fixed                                      =   UAV_current.getFixedMass();
    double wingArea                                     =   UAV_current.getWingArea();
    double c_L                                          =   UAV_current.getLiftCoefficient();
    double c_D                                          =   UAV_current.getDragCoefficient();
    hybrid_engine UAV_engine                            =   UAV_current.getEngine();
    double m_batt                                       =   UAV_engine.getBattMass();
    
    m_fixed                                             +=  m_batt;
    
    double m_fuel                                       =   state_current[0];
    double E_batt                                       =   state_current[1];
    
    double time_fuel                                    =   action_current[0];
    double time_batt                                    =   action_current[1];
    double E_battCharge                                 =   action_current[2];
    
    std::string seg_type                                =   segment_current.getSegmentType();
    double g                                            =   segment_current.getGravitationalConstant();
    double density_air                                  =   segment_current.getAirDensity();
    
    double LDRatio                                      =   c_L/c_D;
    
    double eta_ICE                                      =   UAV_engine.getICEEfficiency();
    double e_fuel                                       =   UAV_engine.getFuelEnergyDensity();
    double eta_EM                                       =   UAV_engine.getEMEfficiency();
    double eta_gen                                      =   eta_EM;
    
    double change_m_fuel                                =   0;
    double change_E_batt                                =   0;
    
    if (seg_type == "cruise"){
        double RHS                                      =   (1/std::sqrt(m_fixed + m_fuel)) + (time_fuel * pow(g,1.5))/(2*LDRatio * eta_ICE * e_fuel * sqrt((density_air * wingArea * c_L)/2));
        double m_fuel_end                               =   pow(1/RHS,2) - m_fixed;
        change_m_fuel                                   =   m_fuel_end - m_fuel;
        
        double change_m_fuel_battCharge                 =   -E_battCharge/(e_fuel * eta_ICE * eta_gen);
        
        change_m_fuel                                   +=  change_m_fuel_battCharge;
        
        double m_aircraft_ECruise                       =   m_fixed + m_fuel_end;
        std::cout << m_aircraft_ECruise << "\t" << wingArea << "\t" << density_air << "\t" << c_L << "\t" << time_batt << "\t" << LDRatio << "\t" << eta_EM << std::endl;
        double num_E_batt_cruise                        =   -time_batt * pow(m_aircraft_ECruise * g,1.5) *sqrt((density_air * wingArea * c_L)/2);
        double den_E_batt_cruise                        =   LDRatio * eta_EM;
        double change_E_batt_cruise                     =   num_E_batt_cruise/den_E_batt_cruise;
        change_E_batt                                   =   change_E_batt_cruise;
        std::cout << change_E_batt << std::endl;
        
        // Throw exception if change in E_batt is greater than current battery energy
        // throw int -1 if battery is insufficient, int -2 if fuel is insufficient
        if(-change_E_batt > E_batt){
            throw   -1;
        }
        else if (-change_m_fuel > m_fuel){
            throw   -2;
        }
    }
    else if (seg_type == "climb"){
        
    }
    change_E_batt                                       +=  E_battCharge;
    state_change[0]                                     =   change_m_fuel;
    state_change[1]                                     =   change_E_batt;
    
    state_new                                           =   addVectors(state_current,state_change);
    UAV_state_new.setState(state_new);
    return UAV_state_new;
}

long matchStateValuePair(DP_state state_in, std::vector<stateValuePair> stateValuePairSet){
    std::vector<double> state_match                     =   state_in.getState();
    std::vector<double> state_current;
    DP_state DP_state_current;
    stateValuePair SVP_current;
    
    long indexBestMatch                                 =   0;
    double bestMatchCloseness                           =   1e11;
    
    for(long i = 0; i < stateValuePairSet.size(); i++){
        SVP_current                                     =   stateValuePairSet[i];
        DP_state_current                                =   SVP_current.getDPState();
        state_current                                   =   DP_state_current.getState();
        std::vector<double> dummy                       =   state_match;
        std::transform(dummy.begin(), dummy.end(), state_current.begin(),
                       dummy.begin(), std::minus<double>());
        double currentCloseness                         =   l2_norm(dummy);
        if(currentCloseness < bestMatchCloseness){
            bestMatchCloseness                          =   currentCloseness;
            indexBestMatch                              =   i;
        }
    }
    //    SVP_bestfit                                         =   stateValuePairSet[indexBestMatch];
    return indexBestMatch;
}

// Function that calculates the optimal action given a UAV_state and cost
stateValuePair calcOptimalAction(UAV_state UAV_state_current, UAV UAV_current, std::vector <stateValuePair> forwardStateValuePairs){
    // Initialization
    std::vector <double> action_opt;
    double value_best_current                           =   0;
    stateValuePair best_SVP;
    
    // Extract state vector
    std::vector <double> state_current                  =   UAV_state_current.getState();
    double m_fuel_current                               =   state_current[0];
    double E_batt_current                               =   state_current[1];
    
    
    // Set limits for possible actions
    hybrid_engine UAV_engine                            =   UAV_current.getEngine();
    flight_segment segment_current                      =   UAV_state_current.getSegment();
    double m_batt                                       =   UAV_engine.getBattMass();
    double e_batt                                       =   UAV_engine.getBattEnergyDensity();
    double E_batt_max                                   =   m_batt * e_batt;
    double duration                                     =   segment_current.getDuration();
    
    // Initialization of various loop parameters
    double dt_ratio                                     =   0.02;       // Iteration change in cruise time due to fuel
    double dE_batt_charge_rat                           =   0.02;       // Change in the above per iteration
    
    double t_cruise_fuel                                =   0;
    double t_cruise_batt                                =   0;
    double E_batt_charge                                =   0;
    
    std::vector<double> action_try                      =   {0, 0, 0};
    UAV_state_current.setAction(action_try);
    best_SVP.setUAVState(UAV_state_current);
    // Loop over possible fuel cruise time ratios
    
    for(double t_fuel_ratio = 1; t_fuel_ratio >= 0; t_fuel_ratio -= dt_ratio){
        t_cruise_fuel                                   =   duration * t_fuel_ratio;
        t_cruise_batt                                   =   duration - t_cruise_fuel;
        // Loop over possible battery charge ratios
        for(double E_charge_ratio = 0; E_charge_ratio <=1; E_charge_ratio += dE_batt_charge_rat){
            //std::cout << "Current batt charge ratio: " << E_charge_ratio << std::endl;
            //std::cout << "Current fuel ratio: " << t_fuel_ratio << std::endl;
            E_batt_charge                               =   E_charge_ratio * E_batt_max;
            // Check if battery charge pushes battery past max capacity; break loop if so
            if(E_batt_current + E_batt_charge > E_batt_max){
                break;
            }
            action_try                                  =   {t_cruise_fuel, t_cruise_batt, E_batt_charge};
            UAV_state_current.setAction(action_try);
            try{ //transition_UAV_state can throw exceptions when battery or fuel is insufficient for cruise
                UAV_state UAV_state_next                =   transition_UAV_state(UAV_state_current, UAV_current);
                long indexForwardMatch                  =   matchStateValuePair(UAV_state_next, forwardStateValuePairs);
                stateValuePair SVP_forwardMatch         =   forwardStateValuePairs[indexForwardMatch];
                double value_try                        =   SVP_forwardMatch.getValue();
                if(value_try > value_best_current){
                    value_best_current                  =   value_try;
                    action_opt                          =   action_try;
                    std::cout << "Best value: " << value_best_current << std::endl;
                    best_SVP.setUAVState(UAV_state_current);
                    
                }
            }
            catch(int x){
                if (x == -2) // check if there isn't enough fuel; if so, no point trying to charge more, move to next iteration of the outer loop as it reduces fuel usage at this step
                    break;
                else if (x == -1) // check if there isn't enough battery for current t_fuel_ratio; break out of outer loop if this is the case since further iterations just increase battery usage
                    goto endLoop;
            }
        }
    }
endLoop:
    best_SVP.setValue(value_best_current);
    return best_SVP;
}
// Function to add an SVP to a current SVP sequence; the SVP has some state and action in its data field
stateValuePairSequence appendSVPSequence_inputState(stateValuePair SVP_in, std::vector<stateValuePairSequence> SVPSequence_in, UAV UAV_current){
    UAV_state UAV_state_in                                =   SVP_in.getDPState();
    UAV_state UAV_state_next                              =   transition_UAV_state(UAV_state_in, UAV_current);
    std::vector<stateValuePair> forwardSVPs;
    for(std::vector<stateValuePairSequence>::size_type i = 0; i != SVPSequence_in.size(); i++) {
        forwardSVPs.push_back(SVPSequence_in[i].getLastStateValuePair());
    }
    long indexMatch                                       =   matchStateValuePair(UAV_state_next,forwardSVPs);
    stateValuePairSequence SVPSequence_match              =   SVPSequence_in[indexMatch];
    SVPSequence_match.addStateValuePair(SVP_in);
    return SVPSequence_match;
}

// Function to link all possible current (time t) states to a set of optimal states and actions that already exist for t+1 to t = N
std::vector<stateValuePairSequence> linkOptimalUAVStates(std::vector<stateValuePairSequence> current_SVPSequences, UAV UAV_current, flight_segment segmentType){
    std::vector<stateValuePairSequence> new_SVPSequences;
    std::vector<stateValuePair> forwardSVPs;
    hybrid_engine engine_current                            =   UAV_current.getEngine();
    double m_fuel_max                                       =   engine_current.getMaxFuelMass();
    double e_batt                                           =   engine_current.getBattEnergyDensity();
    double m_batt                                           =   engine_current.getBattMass();
    double E_batt_max                                       =   e_batt * m_batt;
    
    double dE_iter                                          =   E_batt_max/50;
    double dm_fuel_iter                                     =   m_fuel_max/50;
    
    std::vector<stateValuePairSequence>::iterator SVP_iterator;
    for(SVP_iterator = current_SVPSequences.begin(); SVP_iterator < current_SVPSequences.end(); SVP_iterator++){
        stateValuePairSequence current_SVPSequences_i       =   *SVP_iterator;
        forwardSVPs.push_back(current_SVPSequences_i.getLastStateValuePair());
    }
    
    for(double E_batt_iter = 0; E_batt_iter <= E_batt_max; E_batt_iter += dE_iter){
        for(double m_fuel_iter = 0; m_fuel_iter <= m_fuel_max; m_fuel_iter += dm_fuel_iter){
            std::cout << "Battery state investigated: " << E_batt_iter << std::endl;
            std::cout << "Fuel state investigated: " << m_fuel_iter << std::endl;
            UAV_state UAV_state_iter;
            UAV_state_iter.setState({m_fuel_iter, E_batt_iter});
            UAV_state_iter.setSegment(segmentType);
            stateValuePair SVP_iter                         =   calcOptimalAction(UAV_state_iter, UAV_current, forwardSVPs);
            stateValuePairSequence SVPSequence_iter         =   appendSVPSequence_inputState(SVP_iter, current_SVPSequences, UAV_current);
            new_SVPSequences.push_back(SVPSequence_iter);
        }
    }
    
    return new_SVPSequences;
}

// Function to initialize the vector of SVP sequences, assuming terminal value is the last fuel value
std::vector<stateValuePairSequence> initialize_UAV_SVPSequence(UAV UAV_current, flight_segment segmentType){
    std::vector<stateValuePairSequence> initialized_SVPSequence;
    hybrid_engine engine_current                            =   UAV_current.getEngine();
    double m_fuel_max                                       =   engine_current.getMaxFuelMass();
    double e_batt                                           =   engine_current.getBattEnergyDensity();
    double m_batt                                           =   engine_current.getBattMass();
    double E_batt_max                                       =   e_batt * m_batt;
    
    double dE_iter                                          =   E_batt_max/50;
    double dm_fuel_iter                                     =   m_fuel_max/50;
    
    for(double E_batt_iter = 0; E_batt_iter <= E_batt_max; E_batt_iter += dE_iter){
        for(double m_fuel_iter = 0; m_fuel_iter <= m_fuel_max; m_fuel_iter += dm_fuel_iter){
            UAV_state UAV_state_iter;
            UAV_state_iter.setState({m_fuel_iter, E_batt_iter});
            UAV_state_iter.setSegment(segmentType);
            stateValuePair SVP_iter;
            SVP_iter.setValue(m_fuel_iter);
            SVP_iter.setUAVState(UAV_state_iter);
            stateValuePairSequence SVPSequence_iter(SVP_iter);
            initialized_SVPSequence.push_back(SVPSequence_iter);
        }
    }
    
    return initialized_SVPSequence;
}
