
//================================================================//
//==                                                                                                                                            ==//
//==                                                      BoardSupport                                                             ==//
//==                                                                                                                                            ==//
//================================================================//

//INCLUDES:
#include "bs.h"



//DEFINES:
#define POWERON    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0)
#define POWEROFF   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1)


//PRIVATE VARIABLES:



//FUNCTIONS:

void WakeUP()
{
    POWERON;
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
}



void Halt()
{
    POWEROFF;
}



