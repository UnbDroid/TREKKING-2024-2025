#include <stdio.h>
#include<PinConfig.h>
#include <iostream>

extern "C" void app_main(void)
{   
    while(1){
        std::cout << "Hello World" << std::endl;
    }
    
}