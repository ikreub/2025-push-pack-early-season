#ifndef INTAKE_HPP
#define INTAKE_HPP

namespace Intake{
    const int MATCH_LOAD = 1;
    const int INTAKE = 2;
    const int LOW_SCORE = 3;
    const int HIGH_SCORE = 4;
    const int MID_SCORE = 5;
    const int STOP = 6;
    const int SPECIAL = 7;
    const int BSPECIAL = 8;
    void move(int mode, int power = 0);
    void Intake_Controll();
}

#endif