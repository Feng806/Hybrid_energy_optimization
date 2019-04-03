//
//  main.cpp
//  GTLC
//
//  Created by David Yang on 6/4/18.
//  Copyright © 2018 David Yang. All rights reserved.
//


#include "fuelOpt_DP.hpp"

int main(int argc, const char * argv[]) {
// DP settings
    int N_iters                         =   1;
    
// create cruise flight_segment
    double air_density                  =   1;
    double dt                           =   10500;
    std::string segmentType             =   "cruise";
    flight_segment cruise_segment(segmentType);
    cruise_segment.setDuration(dt);
    cruise_segment.setAirDensity(air_density);
    
// create UAV object
    double mass_fixed                   =   400;
    double wingArea                     =   15;
    double c_L                          =   0.7;
    double c_D                          =   0.1;
    
    UAV UAV_current(mass_fixed, wingArea, c_L, c_D);
    
// create hybrid_engine object
    double e_batt                       =   15e6;
    double e_fuel                       =   42.6e6;
    double p_EM                         =   5e3;
    double p_ICE                        =   0.7e3;
    
    double eta_EM                       =   0.9;
    double eta_ICE                      =   0.3;
    
    double m_EM                         =   24;
    double m_ICE                        =   76;
    double m_total                      =   m_ICE + m_EM;
    double rat_m                        =   m_ICE/m_total;
    
    double P_EM                         =   p_EM * m_EM;
    double P_ICE                        =   p_ICE * m_ICE;
    double P_total                      =   P_EM + P_ICE;
    double rat_P                        =   P_ICE/P_total;
    
    double m_batt                       =   80;
    double m_fuel_max                   =   30;
    
    hybrid_engine UAV_engine_curr(P_total, m_total, rat_P, rat_m);
    UAV_engine_curr.setBattEnergyDensity(e_batt);
    UAV_engine_curr.setBattMass(m_batt);
    UAV_engine_curr.setFuelEnergyDensity(e_fuel);
    UAV_engine_curr.setMaxFuelMass(m_fuel_max);
    UAV_engine_curr.setICEEfficiency(eta_ICE);
    UAV_engine_curr.setEMEfficiency(eta_EM);
    
    UAV_current.setEngine(UAV_engine_curr);
    
    //=============================== Test code ============================================================
    /*std::vector<double> state_i         =   {m_fuel_max, m_batt*e_batt};
    std::vector<double> action_i        =   {dt, dt, 0};
    
    UAV_state UAV_state_i;
    UAV_state_i.setSegment(cruise_segment);
    UAV_state_i.setAction(action_i);
    UAV_state_i.setState(state_i);
    
    UAV_state UAV_state_f               =   transition_UAV_state(UAV_state_i, UAV_current);
    std::vector<double> state_f         =   UAV_state_f.getState();
    
    std::vector<double>::iterator it_test_i;
    
    for(it_test_i = state_i.begin(); it_test_i < state_i.end(); it_test_i++){
        std::cout << *it_test_i << "\t";
    }
    std::cout << std::endl << std::endl;
    
    std::vector<double>::iterator it_test;
    
    for(it_test = state_f.begin(); it_test < state_f.end(); it_test++){
        std::cout << *it_test << "\t";
    }
    std::cout << std::endl;
    */
    //=============================== Main body ============================================================
    
/*

// initialize DP calculation wtih stateValuePairSequence at t = N (terminal time-step)
    std::vector<stateValuePairSequence> SVPSequence_DP              =   initialize_UAV_SVPSequence(UAV_current, cruise_segment);
    
// Loop through intermediate DP cruise time-steps
    for(int i = 0; i <= N_iters; i++){
        std::cout << "Current iteration: " << i+1 << std::endl;
        SVPSequence_DP                                              =   linkOptimalUAVStates(SVPSequence_DP, UAV_current, cruise_segment);
    }
    
    stateValuePairSequence SVPSequence_test                                                =   SVPSequence_DP.back();
    std::vector <stateValuePair> SVPS_test                                                 =    SVPSequence_test.getSequence();
    
    std::vector<stateValuePair>::iterator SVP_iterator;
    for(SVP_iterator = SVPS_test.begin(); SVP_iterator < SVPS_test.end(); SVP_iterator++){
        stateValuePair current_SVP_i       =   *SVP_iterator;
        UAV_state UAV_state_test                                                               =    current_SVP_i.getDPState();
        double value                                                                           =    current_SVP_i.getValue();
        std::vector<double> state_vect_test                                                    =    UAV_state_test.getState();
        std::vector<double> action_vect_test                                                   =    UAV_state_test.getAction();
        
        std::vector<double>::iterator stateVectIterator;
        std::vector<double>::iterator actionVectIterator;
        
        // print out state vector
        for (stateVectIterator = state_vect_test.begin(); stateVectIterator < state_vect_test.end(); stateVectIterator++){
            std::cout << *stateVectIterator << "\t";
        }
        std::cout << "\t";
        // print out action vector
        for (actionVectIterator = action_vect_test.begin(); actionVectIterator < action_vect_test.end(); actionVectIterator++){
            std::cout << *actionVectIterator << "\t";
        }
        std::cout << "\t" << value;
        std::cout << std::endl;
    }
   */
    
    // =================================================== Test ==============================================
    double t                                             =   2625;
    std::vector <double> action_test                     =   {0,t,0};
    UAV_state UAV_state_test;
    UAV_state_test.setAction(action_test);
    
    std::vector <double> state_test                      =  {m_fuel_max, m_batt*e_batt};
    UAV_state_test.setState(state_test);
    
    flight_segment segment_test;
    segment_test.setAirDensity(air_density);
    segment_test.setDuration(t);
    segment_test.setGravitationalConstant(9.81);
    segment_test.setSegmentType("cruise");
    UAV_state_test.setSegment(segment_test);
    
    UAV_state UAV_state_transitioned                              =   transition_UAV_state(UAV_state_test, UAV_current);
    std::vector<double> state_end                                 =   UAV_state_transitioned.getState();
    
    std::vector<double>::iterator stateVectIterator;
    for (stateVectIterator = state_end.begin(); stateVectIterator < state_end.end(); stateVectIterator++){
        std::cout << *stateVectIterator << "\t";
    }
    std::cout << std::endl;
  /*
    std::vector <double> test    =   {1,3};
    std::vector <double> c       =   {2,4};
   
    Eigen::Matrix2d a;
    a << 1, 2,
         3, 4;
    Eigen::Vector2d b;
    b << 1,
         3;
   
    std::transform(test.begin(), test.end(), c.begin(),
                   test.begin(), std::minus<double>());
     Eigen::Vector2d test2(c.data());
    std::cout << test2 << std::endl;
//    std::cout << b << std::endl;
//    std::cout << std::sqrt(b.dot(b)) << std::endl;
//    std::cout << test.size() << std::endl;
   */
}
