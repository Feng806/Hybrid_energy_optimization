//
//  fuelOpt_DP.hpp
//  GTLC
//
//  Created by David Yang on 6/16/18.
//  Copyright © 2018 David Yang. All rights reserved.
//

#ifndef fuelOpt_DP_hpp
#define fuelOpt_DP_hpp

#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <functional>
#include </Users/David/Dropbox (Personal)/GTL/GTLC/GTLC/Eigen/Dense>

class flight_segment
{
public:
    std::string segment_type;
    double density_air;
    double g;
    double duration;

    flight_segment();
    flight_segment(std::string segment_in);
    void setAirDensity(double rho_in);
    void setGravitationalConstant(double g_in);
    void setSegmentType(std::string type_in);
    void setDuration(double duration_in);
    double getAirDensity();
    double getGravitationalConstant();
    std::string getSegmentType();
    double getDuration();
};
class DP_state {
public:
    std::vector <double> state;
    std::vector <double> action;
    void setState(std::vector <double> state_in);
    
    void setAction(std::vector <double> action_in);
    
    std::vector <double> getState();
    std::vector <double> getAction();
    
    DP_state();
};
class UAV_state : public DP_state {
public:
    flight_segment current_segment;
    UAV_state(std::vector <double> state_in, std::vector <double> action_in);
    UAV_state();
    
    void setSegment(flight_segment segment_in);
    flight_segment getSegment();
};
class engine {
public:
    double power;
    double mass;
    
    engine();
    
    engine(double p_in, double m_in);
    
    void setPower(double P_in);
    void setMass(double m_in);
    double getPower();
    double getMass();
};


class hybrid_engine : public engine {
public:
    double ratio_power; // Defined as P_ICE divided by the total power
    double ratio_mass;  // Defined as m_ICE divided by the total mass
    double P_EM;
    double P_ICE;
    double m_EM;
    double m_ICE;
    
    double e_fuel;
    double e_batt;
    double m_batt;
    double m_fuel_max;
    
    double eta_ICE;
    double eta_EM;
    
    std::string type;
    
    hybrid_engine(double P_in, double m_in, double rat_P_in, double rat_m_in);
    hybrid_engine();
    void setPowerRatio(double rat_P_in);
    void setMassRatio(double rat_m_in);
    void setFuelEnergyDensity(double e_fuel_in);
    void setBattEnergyDensity(double e_batt_in);
    void setICEEfficiency(double eta_ICE_in);
    void setEMEfficiency(double eta_EM_in);
    void setBattMass(double m_batt_in);
    void setMaxFuelMass(double m_fuel_in);
    
    double getPowerRatio();
    double getMassRatio();
    double getICEPower();
    double getEMPower();
    double getFuelEnergyDensity();
    double getBattEnergyDensity();
    double getICEEfficiency();
    double getEMEfficiency();
    double getBattMass();
    double getMaxFuelMass();
};

class UAV {
public:
    double mass_fixed;
    double wingArea;
    double c_L;
    double c_D;
    
    hybrid_engine UAV_engine;
    
    UAV();
    
    UAV(double m_in, double area_in, double c_L_in, double c_D_in);
    
    void setFixedMass(double m_input);
    void setWingArea(double area_in);
    void setLiftCoefficient(double c_L_in);
    void setDragCoefficient(double c_D_in);
    double getFixedMass();
    double getWingArea();
    double getLiftCoefficient();
    double getDragCoefficient();
    void setEngine(hybrid_engine engine_in);
    hybrid_engine getEngine();
};

class stateValuePair {
public:
    UAV_state state;
    double costValue;
    
    void setUAVState(UAV_state state_in);
    void setValue(double value_in);
    
    UAV_state getDPState();
    double getValue();
};
class stateValuePairSequence {
public:
    
    std::vector <stateValuePair> SVP_sequence;
    
    stateValuePairSequence(stateValuePair SVP_in);
    
    void resetSequence();
    void addStateValuePair(stateValuePair SVP_in);
    stateValuePair getLastStateValuePair();
    std::vector <stateValuePair> getSequence();
    stateValuePairSequence(std::vector <stateValuePair> sequenceStart);
};



UAV_state transition_UAV_state(UAV_state UAV_state_current, UAV current);
std::vector<double> linspace(double a, double b, std::size_t N);
double l2_norm(std::vector<double> const& u);
std::vector <double> addVectors(std::vector <double> a, std::vector <double> b);
long matchStateValuePair(DP_state state_in, std::vector<stateValuePair> stateValuePairSet);
stateValuePair calcOptimalAction(UAV_state UAV_state_current, UAV UAV_current, std::vector <stateValuePair> forwardStateValuePairs);
std::vector<stateValuePairSequence> linkOptimalUAVStates(std::vector<stateValuePairSequence> current_SVPSequences, UAV UAV_current, flight_segment segmentType);
std::vector<stateValuePairSequence> initialize_UAV_SVPSequence(UAV UAV_current, flight_segment segmentType);
#endif /* fuelOpt_DP_hpp */
