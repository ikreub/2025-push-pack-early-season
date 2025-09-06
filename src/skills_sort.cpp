#include "skills_sort.hpp"

const int RED = 8;
const int BlUEmin = 210;
const int BlUEmax = 230;

double HUE;

void SORT::ALG(){
    if(sort_on){
        HUE = sorter.get_hue();
        if(HUE <= RED){
            
        }else if(HUE >= BlUEmin && HUE <= BlUEmax){

        }
    }
}