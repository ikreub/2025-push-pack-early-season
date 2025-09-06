#ifndef MCL_HPP
#define MCL_HPP
#include "structs.hpp"
#include <vector>

std::vector<double> coord_add(std::vector<double> c1,std::vector<double> c2);
void alg1();
void alg2();
void initialize_tracking();
std::vector<std::vector<double>> gen_particals(std::vector<std::vector<double>> top_three, std::vector<coord> ot);
std::vector<std::vector<double>> move_particals();
void set_particals(std::vector<std::vector<double>> setter);
inline std::vector<std::vector<double>> off_sets;
inline std::vector<double> C_pose;
inline std::vector<double> P_pose;
inline std::vector<double> E_pose;
inline std::vector<std::vector<double>> particals;
inline std::vector<double> Current_Estimate;
inline int c;
std::vector<double> measure_offset(double m0, double m30, double m45);

#endif