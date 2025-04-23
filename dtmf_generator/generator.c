// www.cteq.eu
#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "audio.h"
#include "xscutimer.h"
#include "xscugic.h"
#include "math.h"
#include "xgpio.h"

#define UINT32_MAX_AS_FLOAT 4294967295.0f //(2^32 - 1
#define UINT_SCALED_MAX_VALUE 0x3FFFF // 2^24 =>24 bits audio codec maximum value is 0xFF FFFF

#define TIMER_DEVICE_ID		XPAR_XSCUTIMER_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define TIMER_IRPT_INTR		XPAR_SCUTIMER_INTR

#define SAMPLE_RATE 		8000 // Sampling rate of audio input, is also used for generating an interrupt at this frequency. 48kHz.
#define AVERAGING_BUFFER_SIZE 1
//dtmf frequencies
#define DTMF_X0 1209.0f
#define DTMF_X1 1336.0f
#define DTMF_X2 1477.0f
#define DTMF_Y0 697.0f
#define DTMF_Y1	770.0f
#define DTMF_Y2 852.0f
#define DTMF_Y3 941.0f

volatile unsigned long u32DataL, u32DataR;
volatile uint8_t isr_flag = 0;

typedef struct callback_ref_t
{
	XScuTimer *timer_instance_ptr;
	float32_t *frequency1;
	float32_t *frequency2;
} callback_ref_t;

// Timer_ISR for sine generation (no LUT, our processor seems to be fast enough ;-) )
static void Timer_ISR(void *callback_ref)
{
	callback_ref_t *callback = ( callback_ref_t * )callback_ref;

	XScuTimer *timer_instance_ptr = ( XScuTimer * )callback->timer_instance_ptr;
	float32_t frequency1 = *( ( float32_t * )callback->frequency1 );
	float32_t frequency2 = *( ( float32_t * )callback->frequency2 );

	XScuTimer_ClearInterruptStatus( timer_instance_ptr );
	float32_t theta_increment = 2 * PI * frequency1 / SAMPLE_RATE;
	float32_t beta_increment = 2 * PI * frequency2 / SAMPLE_RATE;
	static float32_t theta = 0.0f;
	static float32_t beta = 0.0f;
	static float32_t running_sum = 0.0f;
	static float32_t buffer[AVERAGING_BUFFER_SIZE] = { 0 };
	static uint8_t buffer_counter = 0;
	static unsigned long isr_counter = 0;


	theta += theta_increment ;
	if ( theta > 2* PI)
		theta -= 2* PI;

	beta += beta_increment ;
	if ( beta > 2* PI)
		beta -= 2* PI;

	float32_t sine_value = arm_sin_f32(theta); // CMSIS function
	sine_value += arm_sin_f32(beta);

	running_sum -= buffer[buffer_counter];
	buffer[buffer_counter] = sine_value;
	buffer_counter = ( buffer_counter + 1 ) % AVERAGING_BUFFER_SIZE;
	running_sum += sine_value;

	uint32_t scaled_value = (uint32_t)(((running_sum / AVERAGING_BUFFER_SIZE ) + 1.0f ) * UINT_SCALED_MAX_VALUE);
	Xil_Out32(I2S_DATA_TX_R_REG, scaled_value);
	++isr_counter;

	if (isr_counter == SAMPLE_RATE )
	{
		isr_flag = ( isr_flag % 12 ) + 1;
		isr_counter = 0;
	}
}

static int Timer_Intr_Setup(XScuGic * IntcInstancePtr, callback_ref_t *callback_data, u16 TimerIntrId)
{
	XScuGic_Config *IntcConfig;
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig, IntcConfig->CpuBaseAddress);
	// Step 1: Interrupt Setup
	Xil_ExceptionInit();
	// Step 2: Interrupt Setup
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, IntcInstancePtr);
	// Step 3: Interrupt Setup
	XScuGic_Connect(IntcInstancePtr, TimerIntrId, (Xil_ExceptionHandler)Timer_ISR, ( void * )callback_data );
	// Step 4: Interrupt Setup
	XScuGic_Enable(IntcInstancePtr, TimerIntrId);
	// Step 5:
	XScuTimer_EnableInterrupt(callback_data->timer_instance_ptr);
	// Step 6: Interrupt Setup
	Xil_ExceptionEnable();
	return XST_SUCCESS;
}

int main()
{
	  init_platform();
	  IicConfig(XPAR_XIICPS_0_DEVICE_ID);
	  AudioPllConfig();
	  AudioConfigureJacks();
	  LineinLineoutConfig();

	  XScuTimer scu_timer;
	  XScuTimer_Config *Scu_ConfigPtr;
	  XScuGic IntcInstance;
	  XGpio	gpio;
	  uint32_t read_value;
	  float32_t frequency_x = DTMF_X1;
	  float32_t frequency_y = DTMF_Y1;
	  uint8_t send_value_x = 0;
	  uint8_t send_value_y = 0;

	  Scu_ConfigPtr = XScuTimer_LookupConfig(XPAR_PS7_SCUTIMER_0_DEVICE_ID);
	  XScuTimer_CfgInitialize( &scu_timer, Scu_ConfigPtr, Scu_ConfigPtr->BaseAddr );

	  XGpio_Initialize( &gpio, XPAR_AXI_GPIO_1_DEVICE_ID );
	  XGpio_SetDataDirection( &gpio, 1, ~0xF );

	  callback_ref_t callback_data = {
			  .timer_instance_ptr = &scu_timer,
			  .frequency1 = &frequency_x,
			  .frequency2 = &frequency_y
	  };

	  Timer_Intr_Setup(&IntcInstance, &callback_data, XPS_SCU_TMR_INT_ID);
	  XScuTimer_LoadTimer( &scu_timer,( XPAR_PS7_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2 ) / ( SAMPLE_RATE ) );
	  XScuTimer_EnableAutoReload(&scu_timer);
	  XScuTimer_Start(&scu_timer);

	  for(;;){
		  read_value = XGpio_DiscreteRead( &gpio, 1 );
		  if ( read_value != 0 )
		  {
			  printf( "input: %lx \n", read_value );

			  switch ( read_value )
			  {
			  case 1:
				  send_value_x -= send_value_x == 0 ? 0 : 1;
				  break;
			  case 2:
				  send_value_x += send_value_x == 2 ? 0 : 1;
				  break;
			  case 4:
				  send_value_y -= send_value_y == 0 ? 0 : 1;
				  break;
			  case 8:
				  send_value_y += send_value_y == 3 ? 0 : 1;
				  break;
			  default:
				  break;
			  }

			  while ( read_value != 0 )
			  {
				  read_value = XGpio_DiscreteRead( &gpio, 1 );
			  }
		  }

		  switch ( send_value_x )
		  {
			  case 0:
				  frequency_x = DTMF_X0;
				  break;
			  case 1:
				  frequency_x = DTMF_X1;
				  break;
			  case 2:
				  frequency_x = DTMF_X2;
				  break;
			  default:
				  frequency_x = DTMF_X0;
				  break;
		  }

		  switch ( send_value_y )
		  {
			  case 0:
				  frequency_y = DTMF_Y0;
				  break;
			  case 1:
				  frequency_y = DTMF_Y1;
				  break;
			  case 2:
				  frequency_y = DTMF_Y2;
				  break;
			  case 3:
				  frequency_y = DTMF_Y3;
				  break;
			  default:
				  frequency_y = DTMF_Y0;
				  break;
		  }
	  }
    cleanup_platform();
    return 0;
}
