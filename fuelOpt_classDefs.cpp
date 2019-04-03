//
//  fuelOpt_DP.cpp
//  GTLC
//
//  Created by David Yang on 6/16/18.
//  Copyright © 2018 David Yang. All rights reserved.
//

#include "fuelOpt_DP.hpp"

// Class that represents the different segments of flight and associated parameters

flight_segment::flight_segment(){
    g                                   =   9.81;
    density_air                         =   1.225;
}
    
flight_segment::flight_segment(std::string type_in){
    segment_type                        =   type_in;
    g                                   =   9.81;
    density_air                         =   1.225;
}
void flight_segment::setAirDensity(double rho_in){
    density_air                         =   rho_in;
}
void flight_segment::setGravitationalConstant(double g_in){
    g                                   =   g_in;
}
void flight_segment::setSegmentType(std::string type_in){
    segment_type                        =   type_in;
}
void flight_segment::setDuration(double t_in){
    duration                            =   t_in;
}

double flight_segment::getAirDensity(){
    return density_air;
}
double flight_segment::getGravitationalConstant(){
    return g;
}
std::string flight_segment::getSegmentType(){
    return segment_type;
}
double flight_segment::getDuration(){
    return duration;
}


// Class that represents a dynamic programming state and action pair

    
void DP_state::setState(std::vector <double> state_in){
    state               =   state_in;
}

void DP_state::setAction(std::vector <double> action_in){
    action               =   action_in;
}

std::vector <double> DP_state::getState(){
    return state;
}
std::vector <double> DP_state::getAction(){
    return action;
}

DP_state::DP_state(){
}


// DP_state subclass that is specific to UAV fuel optimization

UAV_state::UAV_state(std::vector <double> state_in, std::vector <double> action_in){
    state           = state_in;
    action          = action_in;
}
UAV_state::UAV_state(){}

void UAV_state::setSegment(flight_segment segment_in){
    current_segment            =   segment_in;
}
flight_segment UAV_state::getSegment(){
    return current_segment;
}


// Object that represents an engine with some mass and power output

engine::engine(){}
    
engine::engine(double p_in, double m_in){
    power                   =   p_in;
    mass                    =   m_in;
}

void engine::setPower(double P_in){
    power                   =   P_in;
}
void engine::setMass(double m_in){
    mass                    =   m_in;
}
double engine::getPower(){
    return power;
}
double engine::getMass(){
    return mass;
}


// engine subclass that represents a hybrid engine with an ICE and EM
hybrid_engine::hybrid_engine(double P_in, double m_in, double rat_P_in, double rat_m_in){
        power                       =   P_in;
        mass                        =   m_in;
        ratio_power                 =   rat_P_in;
        ratio_mass                  =   rat_m_in;
        
        P_ICE                       =   ratio_power * power;
        P_EM                        =   power - P_ICE;
        
        m_ICE                       =   ratio_mass * mass;
        m_EM                        =   mass - m_ICE;
    }
hybrid_engine::hybrid_engine(){}
void hybrid_engine::setPowerRatio(double rat_P_in){
    ratio_power                 =   rat_P_in;
}
void hybrid_engine::setMassRatio(double rat_m_in){
    ratio_mass                  =   rat_m_in;
}
void hybrid_engine::setFuelEnergyDensity(double e_fuel_in){
    e_fuel                      =   e_fuel_in;
}
void hybrid_engine::setBattEnergyDensity(double e_batt_in){
    e_batt                      =   e_batt_in;
}
void hybrid_engine::setICEEfficiency(double eta_ICE_in){
    eta_ICE                     =   eta_ICE_in;
}
void hybrid_engine::setEMEfficiency(double eta_EM_in){
    eta_EM                      =   eta_EM_in;
}
void hybrid_engine::setBattMass(double m_batt_in){
    m_batt                      =   m_batt_in;
}
void hybrid_engine::setMaxFuelMass(double m_fuel_in){
    m_fuel_max                  =   m_fuel_in;
}
    
double hybrid_engine::getPowerRatio(){
    return ratio_power;
}
double hybrid_engine::getMassRatio(){
    return ratio_mass;
}
double hybrid_engine::getICEPower(){
    return P_ICE;
}
double hybrid_engine::getEMPower(){
    return P_EM;
}
double hybrid_engine::getFuelEnergyDensity(){
    return e_fuel;
}
double hybrid_engine::getBattEnergyDensity(){
    return e_batt;
}
double hybrid_engine::getICEEfficiency(){
    return eta_ICE;
}
double hybrid_engine::getEMEfficiency(){
    return eta_EM;
}
double hybrid_engine::getBattMass(){
    return m_batt;
}
double hybrid_engine::getMaxFuelMass(){
    return m_fuel_max;
}


// Class that represents a UAV and associated performance parameters and components

UAV::UAV(){}
    
    UAV::UAV(double m_in, double area_in, double c_L_in, double c_D_in){
        mass_fixed              =    m_in;
        wingArea                =   area_in;
        c_L                     =   c_L_in;
        c_D                     =   c_D_in;
    }
    
void UAV::setFixedMass(double m_input){
    mass_fixed              =   m_input;
}
void UAV::setWingArea(double area_in){
    wingArea                =   area_in;
}
void UAV::setLiftCoefficient(double c_L_in){
    c_L                     =   c_L_in;
}
void UAV::setDragCoefficient(double c_D_in){
    c_D                     =   c_D_in;
}
double UAV::getFixedMass(){
    return mass_fixed;
}
double UAV::getWingArea(){
    return wingArea;
}
double UAV::getLiftCoefficient(){
    return c_L;
}
double UAV::getDragCoefficient(){
    return c_D;
}
void UAV::setEngine(hybrid_engine engine_in){
    UAV_engine               =  engine_in;
}
hybrid_engine UAV::getEngine(){
    return UAV_engine;
}


void stateValuePair::setUAVState(UAV_state state_in){
    state                                           =   state_in;
}
void stateValuePair::setValue(double value_in){
    costValue                                       =   value_in;
}

UAV_state stateValuePair::getDPState(){
    return state;
}
double stateValuePair::getValue(){
    return costValue;
}



stateValuePairSequence::stateValuePairSequence(stateValuePair SVP_in){
    SVP_sequence.push_back(SVP_in);
}

void stateValuePairSequence::resetSequence(){
    SVP_sequence.clear();
}
void stateValuePairSequence::addStateValuePair(stateValuePair SVP_in){
    SVP_sequence.push_back(SVP_in);
}
stateValuePair stateValuePairSequence::getLastStateValuePair(){
    return SVP_sequence.back();
}
std::vector <stateValuePair> stateValuePairSequence::getSequence(){
    return SVP_sequence;
}
stateValuePairSequence::stateValuePairSequence(std::vector <stateValuePair> sequenceStart){
    SVP_sequence                    =   sequenceStart;
}



