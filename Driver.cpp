// http://www.mikrocontroller.net/articles/STM32_BLDC_Control_with_HALL_Sensor
// http://www.mikrocontroller.net/attachment/191263/main.c
// http://www.mikrocontroller.net/attachment/191261/BLDCMotorControlTimer.c
// http://www.mikrocontroller.net/attachment/191262/HallSensorTimer.c

#include "Driver.h"
/*
#include "Config.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "Timer.h"
*/

#define FREEWHEEL 0

template <typename T> T Abs (T x)
{
	if (x < 0) return -x;
	else return x;
}

namespace Motor
{
	namespace Driver
	{
		const int PwmPeriod = 500; //500
		volatile bool Clockwise = true;
		volatile int Period6 = 65535*6;
		volatile int PeriodLast = 65535;
		volatile int Position = 0;

		volatile int TargetPwm = 1;

		int StateToIndex[8] = {0, 1,5,6,3,2,4, 7};
		enum BridgeState{EN, Z, L, R, F, H };
		BridgeState HallStateToHBridge[8][3] = {
			{Z, Z, Z},	// - Free wheel

			{	F,	L,	EN	},	//1
			{	L,	R,	EN	},	//5
			{	L,	EN,	F	},	//4
			{	R,	EN,	L	},	//6
			{	EN,	F,	L	},	//2
			{	EN,	L,	R	},	//3

			{L, L, L}};	// Break

		BridgeState Channel1State = L;
		BridgeState Channel2State = L;
		BridgeState Channel3State = L;

		void InitPins()
		{
			/*
                        static GPIO_InitTypeDef GPIO_InitStructure;
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStructure);

			GPIO_WriteBit(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, (BitAction)1);

			GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_4 | GPIO_Pin_5;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
                        */
		}
		void InitBridgeTimer()
		{

                  /*    static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
			static TIM_OCInitTypeDef  TIM_OCInitStructure;
			static TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
			static NVIC_InitTypeDef NVIC_InitStructure;

			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

			// Time Base configuration

			TIM_TimeBaseStructure.TIM_Prescaler = 1;
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
			TIM_TimeBaseStructure.TIM_Period = PwmPeriod+1; // PWM max period
			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
			TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

			// Channel 1, 2, 3 – set to PWM mode - all 6 outputs
			// per channel on output is  low side fet, the opposite is for high side fet

			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // TIM_OCMode_PWM1 TIM_OCMode_Timing
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 1; // BLDC_ccr_val

			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
			TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
			TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

			TIM_OC1Init(TIM1, &TIM_OCInitStructure);
			TIM_OC2Init(TIM1, &TIM_OCInitStructure);
			TIM_OC3Init(TIM1, &TIM_OCInitStructure);

			// activate preloading the CCR register
			TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
			TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
			TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

			// Enable interrupt
			NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

			// enable motor timer
			TIM_Cmd(TIM1, ENABLE);

			// enable motor timer main output (the bridge signals)
			TIM_CtrlPWMOutputs(TIM1, ENABLE);
                        */
		}
		void InitHallTimer(void)
		{
			/*
                        static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
			static TIM_ICInitTypeDef  TIM_ICInitStructure;
			static TIM_OCInitTypeDef  TIM_OCInitStructure;
			static NVIC_InitTypeDef NVIC_InitStructure;

			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

			// timer base configuration
			// 126 => 3,5s till overflow ; 285,714kHz TimerClock [36MHz/Prescaler]
			TIM_TimeBaseStructure.TIM_Prescaler = PwmPeriod;
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseStructure.TIM_Period = 65535;
			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
			TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

			// enable hall sensor
			// T1F_ED will be connected to  HallSensoren Imputs
			// TIM4_CH1,TIM4_CH2,TIM4_CH3
			TIM_SelectHallSensor(TIM3, ENABLE);

			//  TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
			//  uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity)

			// HallSensor event is delivered with singnal TI1F_ED
			// (this is XOR of the three hall sensor lines)
			// Signal TI1F_ED: falling and rising ddge of the inputs is used
			TIM_SelectInputTrigger(TIM3, TIM_TS_TI1F_ED);

			// On every TI1F_ED event the counter is resetted and update is tiggered
			TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);

			// Channel 1 in input capture mode
			// on every TCR edge (build from TI1F_ED which is a HallSensor edge)
			// the timervalue is copied into ccr register and a CCR1 Interrupt
			// TIM_IT_CC1 is fired

			TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			// listen to T1, the  HallSensorEvent
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;
			// Div:1, every edge
			TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
			// noise filter: 1111 => 72000kHz / factor (==1) / 32 / 8 -> 281kHz
			// input noise filter (reference manual page 322)
			TIM_ICInitStructure.TIM_ICFilter = 0x3;
			TIM_ICInit(TIM3, &TIM_ICInitStructure);

			// channel 2 can be use for commution delay between hallsensor edge
			// and switching the FET into the next step. if this delay time is
			// over the channel 2 generates the commutation signal to the motor timer
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_Pulse = 1; // 1 is no delay; 2000 = 7ms
			TIM_OC2Init(TIM3, &TIM_OCInitStructure);

			// clear interrupt flag
			TIM_ClearFlag(TIM3, TIM_FLAG_CC2);

//			TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
//			TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC1);
			// timer2 output compate signal is connected to TRIGO
			//TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC2Ref);

			// Enable channel 2 compate interrupt request
			// TIM_IT_CC1 | TIM_IT_CC2
			TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);

			// Enable output compare preload
			//TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

			// Enable ARR preload
			//TIM_ARRPreloadConfig(TIM4, ENABLE);

			// Enable update event
			//TIM_ClearFlag(TIM4, TIM_FLAG_Update);
			//TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);

			// we use preemption interrupts here,  BLDC Bridge switching and
			// Hall has highest priority
			NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			// -------------------
			// HallSensor is now configured, if BLDC Timer is also configured
			// after enabling timer 4
			// the motor will start after next overflow of the hall timer because
			// this generates the first startup motor cummutation event
			TIM_Cmd(TIM3, ENABLE);

			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 10000;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC3Init(TIM3, &TIM_OCInitStructure);
			TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
                        */



		}

		void Init()
		{
			InitPins();
			InitBridgeTimer();
			InitHallTimer();

			SetPWM(0);
		}

		void SetPWM(int pwm)
		{
			if (pwm < 0)
			{
				pwm = -pwm;
				Clockwise = true;
			}
			else
			{
				Clockwise = false;
			}

			/*if (CCW == true) Clockwise = !Clockwise;*/

			if (pwm > 950) pwm = 950;
			if (pwm < 1) pwm = 1;

			TargetPwm = pwm;
		}

		void ResetPosition()
		{
			Position = 0;
		}
		int GetPosition()
		{
			/*if (CCW == false)
				return -Position;
			else
				return Position;*/
		}

		int IncrementalEncoder(int lastState, int currentState)
		{
			int delta = currentState - lastState;

			if (delta < -3) delta = 6 + delta;
			if (delta > 3) delta =  -6 + delta;
			if (delta == 3 || delta == -3) ;
			return delta;
		}

		extern "C" void TIM1_UP_TIM16_IRQHandler(void)
		{
			/*if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
			{
				TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

				static int channel1 = 1;
				static int channel2 = 1;
				static int channel3 = 1;

				int period  = PwmPeriod * 1000;

				//incrementValue = (((period * 1 )/PeriodLast) * RampCorrection) / 50;
				const int incrementValue = (((period * 6 )/Period6) * RampCorrection) / 50;

				if( Channel1State == F || Channel1State == L ) channel1 -= incrementValue;
				if( Channel1State == R || Channel1State == EN ) channel1 += incrementValue;
				if( Channel2State == F || Channel2State == L ) channel2 -= incrementValue;
				if( Channel2State == R || Channel2State == EN ) channel2 += incrementValue;
				if( Channel3State == F || Channel3State == L ) channel3 -= incrementValue;
				if( Channel3State == R || Channel3State == EN ) channel3 += incrementValue;

				if( channel1 < 1 ) channel1 = 1;
				if( channel2 < 1 ) channel2 = 1;
				if( channel3 < 1 ) channel3 = 1;

				if( channel1 > period ) channel1 = period;
				if( channel2 > period ) channel2 = period;
				if( channel3 > period ) channel3 = period;

				TIM1->CCR1 = (channel1*TargetPwm/period)*PwmPeriod/1000;
				TIM1->CCR2 = (channel2*TargetPwm/period)*PwmPeriod/1000;
				TIM1->CCR3 = (channel3*TargetPwm/period)*PwmPeriod/1000;
			}*/
		}
		void Commutate( )
		{
			bool HallA = 0, HallB = 0, HallC = 0;
			/*HallA = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);
			HallB = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
			HallC = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);*/
			int HallState = (HallA?1:0) | (HallB?2:0) | (HallC?4:0);

			static int LastHallState = 0;
			Position -= IncrementalEncoder(StateToIndex[LastHallState], StateToIndex[HallState]);
			LastHallState = HallState;

			int shift = 2;
#ifdef SMALL_MOTOR
			if (Clockwise) shift = 4; //4
			if (!Clockwise) shift = 0; //0

			int state = StateToIndex[HallState];
			state = (state-1+shift)%6+1;

			Channel1State = HallStateToHBridge[state][2];
			Channel2State = HallStateToHBridge[state][1];
			Channel3State = HallStateToHBridge[state][0];
#else
			if (Clockwise) shift = 5; //3           //5
			if (!Clockwise) shift = 1; //1  - BM28  //1  - BM18
			int state = StateToIndex[HallState];
			state = (state-1+shift)%6+1;

			Channel1State = HallStateToHBridge[state][0];
			Channel2State = HallStateToHBridge[state][1];
			Channel3State = HallStateToHBridge[state][2];
#endif
		}
		void Encoder( bool overflow = false )
		{
			static int Periods[7] = {0};
			int temp = Periods[5];
			for (int i=5; i>0; i--)
				temp += Periods[i] = Periods[i-1];

			if( overflow )
				Periods[0] = 10000;
			else
				//Periods[0] = TIM3->CCR1;

			Period6 = temp;
			PeriodLast = Periods[0];
		}

		extern "C" void TIM3_IRQHandler(void)
		{
			/*if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
			{
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
				TIM_SetCounter(TIM3, 0);
				Encoder( true );
				Commutate( );
			}
			else
			if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
			{
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
				Encoder( false );
				Commutate( );
			}
			else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
			{
				TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
			}
*/
		}
	}
}
