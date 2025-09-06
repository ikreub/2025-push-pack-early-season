#include "mcl.hpp"
#include "EZ-Template/sdcard.hpp"
#include "pros/misc.h"
#include "structs.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/imu.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

const double rt2 = sqrt(2);
const double rt3 = sqrt(3);
const double rt2p2 = rt2 - 1;
const double rt2p3 = rt2p2 / rt3;
const double dr32 = 1 + rt2 / rt3 - rt3;
const double d32 = rt2 - rt2 * rt3;

double t1 = 0;
double t2 = 0;
double t3 = 0;
double t4 = 0;
double cr = 0;

const std::vector<double> F_off = {0,0,0};
const std::vector<double> L_off = {0,0,0};
const std::vector<double> R_off = {0,0,0};
const std::vector<double> B_off = {0,0,0};


std::vector<double> coord_add(std::vector<double> c1,std::vector<double> c2){
    return {util::clamp(c1[0] + c2[0], 0, 144),util::clamp(c1[1] + c2[1],0,144),fmod(c1[2] + c2[2] + M_PI,2 * M_PI) - M_PI};
}

std::vector<std::vector<double>> gen_particals(std::vector<std::vector<double>> top_three, std::vector<std::vector<double>> ot = {}){
    for(int j = 0;  j < top_three.size(); j++){
        for(int i = 0; i < off_sets.size(); ++i){
            ot.push_back(coord_add(top_three[j],off_sets[i]));
        }
    }
    return ot;
}

void initialize_tracking(){
    for(double r = -6; r < 6; ++r){
        for(double j = -6; j < 6 * r; ++j){
            off_sets.push_back({j, r});
        }
    }
}

//void initialize_tracking(){
//    for(int r = 0; r < 9; ++r){
//        for(int j = 0; j < 4 * r; ++j){
//            off_sets.push_back({r * cos(j / (4 * r)),r * sin(j / (4 * r)),0});
//        }
//    }
//}

std::vector<std::vector<double>> move_particals(){
    std::vector<std::vector<double>> o = {};
    for(int i = 0; i < particals.size(); ++i){
        o.push_back({E_pose[0] + particals[i][0] - P_pose[0],E_pose[1] + particals[i][1] - P_pose[1],particals[i][2]});
    }
    return o;
}

double get_distance(std::vector<double> p){
    if(p[2] >= t1 && p[2] < t4){
        return 25.4 * sqrt(pow(p[0] - 144, 2) + pow(tan(-p[2] + M_PI_2) * (144 - p[0]), 2));
    }else if(p[2] >= t2 && p[2] < t1){
        return 25.4 * sqrt(pow((144 - p[1]) / tan(-p[2] + 90),2) + pow(p[1] - 144, 2));
    }else if(p[2] >= t3 && p[2] < t2){
        return 25.4 * sqrt(pow(p[0], 2) + pow(tan(-p[2] + M_PI_2) * p[0], 2));
    }else{
        return 25.4 * sqrt(pow(p[1] / tan(-p[2] + 90),2) + pow(p[1], 2));
    }
}

std::vector<double> evaluate_partical(std::vector<double> p){
    double evaluation = 0;
    int ec = 0;
    std::vector<double> pf = coord_add(p, coord_add({F_off[1] * sin(p[2]),F_off[1] * cos(p[2]),0}, {F_off[0] * cos(p[2]),-F_off[0] * sin(p[2]),0}));
    std::vector<double> pr = coord_add(p, coord_add({R_off[1] * sin(p[2]),R_off[1] * cos(p[2]),0}, {R_off[0] * cos(p[2]),-R_off[0] * sin(p[2]),M_PI_2}));
    std::vector<double> pl = coord_add(p, coord_add({L_off[1] * sin(p[2]),F_off[1] * cos(p[2]),0}, {L_off[0] * cos(p[2]),-L_off[0] * sin(p[2]),-M_PI_2}));
    std::vector<double> pb = coord_add(p, coord_add({B_off[1] * sin(p[2]),F_off[1] * cos(p[2]),0}, {B_off[0] * cos(p[2]),-B_off[0] * sin(p[2]),M_PI}));
    t1 = -atan((144 - pf[1]) / (144 - pf[0])) + M_PI_2;
    t2 = atan((144 - pf[1]) / pf[0]) - M_PI_2;
    t3 = -atan(pf[1] / pf[0]) - M_PI_2;
    t4 = atan(pf[1] / (144 - pf[0])) + M_PI_2;
    cr = front_read();
    if(cr != 9999){
        ++ec;
        evaluation = pow(cr - get_distance(pf), 2);
    }
    cr = back_read();
    if(cr != 9999){
        ++ec;
        evaluation = evaluation + pow(cr - get_distance(pb), 2);
    }
    cr = left_read();
    if(cr != 9999){
        ++ec;
        evaluation = evaluation + pow(cr - get_distance(pl), 2);
    }
    cr = right_read();
    if(cr != 9999){
        ++ec;
        evaluation = evaluation + pow(cr - get_distance(pr), 2);
    }
    if(ec == 0){
        return {p[0], p[1], 10000};
    }else{
        return {p[0], p[1], evaluation / ec};
    }
}

std::vector<std::vector<double>> full_eval(std::vector<std::vector<double>> ps, std::vector<std::vector<double>> o = {}){
    for(int i = 0; i < ps.size(); ++i){
        o.push_back(evaluate_partical(ps[i]));
    }
    return o;
}

bool sort_particals(std::vector<double>& a, std::vector<double> &b){
    return a[2] < b[2];
}

void set_particals(std::vector<std::vector<double>> setter){
    particals.clear();
    for(int i = 0; i < setter.size(); ++i){
        particals.push_back(setter[i]);
    }
}

void alg1(){   
    master.rumble("...."); 
    C_pose = {chassis.odom_x_get(),chassis.odom_y_get(),chassis.odom_theta_get() * M_PI / 180};
    E_pose = C_pose;
    particals = {C_pose};
    Current_Estimate = C_pose;
    c = 0;
}

void alg2(){
    ++c;
    P_pose = C_pose;
    C_pose = {chassis.odom_x_get(),chassis.odom_y_get(),chassis.odom_theta_get() * M_PI / 180};
    particals = gen_particals(particals);
    particals = full_eval(move_particals());
    std::sort(particals.begin(),particals.end(), sort_particals);
    particals = {particals[0], particals[1], particals[2]};
    if(particals[0][2] <= 200){
        chassis.odom_xy_set(particals[0][0], particals[0][1]);
    }
    Current_Estimate = particals[0];
    pros::delay(ez::util::DELAY_TIME);
    E_pose = {chassis.odom_x_get(),chassis.odom_y_get(),chassis.odom_theta_get() * M_PI / 180};
}

std::vector<double> measure_offset(double m0, double m30, double m45){
    return {((2 * rt3 / 3 - 1) * m45 - rt2p2 * m30 + ((3 * rt2 - 2) / rt3 + d32) * m0) / (dr32),
            (rt2p3 * m45 - rt2p2 * m30 + (3 * rt2 - 4) * m0 / rt3) / (dr32 * rt2p2)};
}

