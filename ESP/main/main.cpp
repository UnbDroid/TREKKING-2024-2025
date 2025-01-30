#include <stdio.h>
#include<motor.h>

extern "C" void app_main(void)
{
    while(1){
        move_foward(1);
    }
    
}