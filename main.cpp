#include <stdlib.h>
#include "NuMicro.h"
#include "Led.h"
#include "nuTimer.h"
#include "Driver.h"

volatile int TestMode = 0;
volatile int TestModePWM = 10;
volatile int Position = 0;

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Select Timer0 clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Enable Timer0 clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Update System Core Clock */
     SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}


int main(void)
{
	SYS_Init();

	Led::Init();
	Led::Module();

	Timer::Delay_ms( 100 );

	Motor::Driver::Init();

	switch (TestMode)
		{
		case 0:
			Motor::Driver::SetPWM(TestModePWM);
			Led::Red( Led::_1Hz );
			break;
		case 1:
			Motor::Driver::SetPWM(TestModePWM);
			Timer::Delay_ms(500);
			Motor::Driver::SetPWM(0);
			Timer::Delay_ms(100);
			Motor::Driver::SetPWM(-TestModePWM);
			Timer::Delay_ms(500);
			Motor::Driver::SetPWM(0);
			Timer::Delay_ms(100);
			Led::Red( Led::_3Hz );
			break;

		case 2:
			for (int i=0; i<250; i++)
			{
				Timer::Delay_ms(10);
				Motor::Driver::SetPWM(i);
			}
			Timer::Delay_ms(1000);
			for (int i=0; i<250; i++)
			{
				Timer::Delay_ms(10);
				Motor::Driver::SetPWM(250-i);
			}
			Timer::Delay_ms(100);
			for (int i=0; i<250; i++)
			{
				Timer::Delay_ms(10);
				Motor::Driver::SetPWM(-i);
			}
			Timer::Delay_ms(1000);
			for (int i=0; i<250; i++)
			{
				Timer::Delay_ms(10);
				Motor::Driver::SetPWM(-250+i);
			}
			Timer::Delay_ms(100);
			break;
		}

}


