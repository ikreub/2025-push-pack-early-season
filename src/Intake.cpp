#include "../include/intake.hpp"
#include "pros/misc.h"
#include "subsystems.hpp"


void Intake::move(int mode, int power){
    if(mode == 1){
        intake_load.move(power);
        intake_basket.move(-power);
    }else if(mode == 2){
        intake_front.move(power);
        intake_basket.move(-power);
    }else if(mode == 3){
        intake_front.move(-power);
        intake_basket.move(2 * power / 3);
        intake_load.move(power);
        intake_outake.move(-power);
    }else if(mode == 4){
        intake_front.move(power);
        intake_basket.move(power);
        intake_load.move(power);
        intake_outake.move(power);
    }else if(mode == 5){
        intake_front.move(power);
        intake_basket.move(power);
        intake_load.move(power);
        intake_outake.move(-power);
    }else if(mode == 7){
        intake_front.move(power);
    }else if(mode == 8){
        intake_front.move(-power);
    }else{
        intake_front.move(0);
        intake_basket.move(0);
        intake_load.move(0);
        intake_outake.move(0);
    }
}

void Intake::Intake_Controll(){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
        move(MATCH_LOAD, 127);
    }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        move(INTAKE, 127);
    }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        move(LOW_SCORE, 127);
    }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        move(HIGH_SCORE, 127);
    }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        move(MID_SCORE, 127);
    }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
        move(SPECIAL, 127);
    }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        move(BSPECIAL, 127);
    }else{
        move(STOP);
    }
}