/******************************************************************************
* Copyright (C) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/
/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>

#include "platform.h"
#include "xil_printf.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "xscutimer.h"
#include "xscugic.h"
#include "xparameters.h"
#include "xiicps.h"
#include "sleep.h"
#include "audio.h"

#define TIMER_DEVICE_ID		XPAR_XSCUTIMER_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define TIMER_IRPT_INTR		XPAR_SCUTIMER_INTR
#define TIMER_LOAD_VALUE	0xFFFF


// DTMF Frequencies (Hz)
#define DTMF_LOW_FREQ_1 697.0f
#define DTMF_LOW_FREQ_2 770.0f
#define DTMF_LOW_FREQ_3 852.0f
#define DTMF_LOW_FREQ_4 941.0f
#define DTMF_HIGH_FREQ_1 1209.0f
#define DTMF_HIGH_FREQ_2 1336.0f
#define DTMF_HIGH_FREQ_3 1477.0f

#define SAMPLE_RATE 8000.0f
#define BUFFER_SIZE 2048
#define FFT_SIZE (BUFFER_SIZE / 2) // Choose a power of 2 for efficient FFT
#define NUM_DTMF_FREQS 7
#define MAGNITUDE_THRESHOLD 10000 // Adjust based on your signal levels
#define AVERAGE_LENGTH 8

// Mapping DTMF frequencies to characters
const char dtmf_map[4][3] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}
};

// Audio processing states
typedef enum {
	STATE_IDLE,
	STATE_BUFFERING,
	STATE_PROCESSING,
	STATE_PRINTING
} AudioState;

/* -------------------------------------------------------------------
 * External Input and Output buffer Declarations
 * ------------------------------------------------------------------- */
static q31_t audio_buffer[BUFFER_SIZE];
static q31_t processing_buffer[BUFFER_SIZE]; // Nieuwe buffer voor verwerking
static q31_t fft_output[BUFFER_SIZE*2];
static arm_rfft_instance_q31 fft_instance;
static volatile AudioState audio_state = STATE_IDLE;
static int sample_index = 0;

/* ------------------------------------------------------------------
 * Global variables
 * ------------------------------------------------------------------- */
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;


static void Timer_ISR(void * CallBackRef)
{
  	    XScuTimer *TimerInstancePtr = (XScuTimer *) CallBackRef;
  		XScuTimer_ClearInterruptStatus(TimerInstancePtr);

  		//-------------------------------------------------------//

  		switch (audio_state) {
			case STATE_BUFFERING: {
				q31_t audioIn_L = Xil_In32(I2S_DATA_RX_L_REG);
				audio_buffer[sample_index] = audioIn_L;
				if (sample_index >= BUFFER_SIZE) {
					sample_index = 0;
					audio_state = STATE_PROCESSING;
				}
				sample_index++;
				break;
			}
			default:
				// Do nothing if not in buffering state
				break;
		}
}

static int Timer_Intr_Setup(XScuGic * IntcInstancePtr, XScuTimer *TimerInstancePtr, u16 TimerIntrId)
{
	int Status;
	XScuGic_Config *IntcConfig;
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig, IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) return Status;

	// Step 1: Interrupt Setup
	Xil_ExceptionInit();

	// Step 2: Interrupt Setup
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, IntcInstancePtr);
	// Step 3: Interrupt Setup
	Status = XScuGic_Connect(IntcInstancePtr, TimerIntrId, (Xil_ExceptionHandler)Timer_ISR, (void *)TimerInstancePtr);
	if (Status != XST_SUCCESS) return Status;

	// Step 4: Interrupt Setup
	XScuGic_Enable(IntcInstancePtr, TimerIntrId);
	// Step 5: Interrupt Setup
	XScuTimer_EnableInterrupt(TimerInstancePtr);
	// Step 6: Interrupt Setup
	Xil_ExceptionEnable();
	return XST_SUCCESS;
}

void process_audio_data(XScuTimer *TimerInstancePtr) {
	switch (audio_state) {
			case STATE_PROCESSING: {
				// Disable timer interrupt to prevent race condition
				XScuTimer_Stop(TimerInstancePtr);

				// Copy the buffer to avoid overwriting during FFT
				memcpy(processing_buffer, audio_buffer, sizeof(audio_buffer));

				arm_rfft_q31(&fft_instance, processing_buffer, fft_output);
				arm_cmplx_mag_q31(processing_buffer, fft_output, FFT_SIZE);

				// Detect DTMF-tonen
				float32_t dtmf_freqs[NUM_DTMF_FREQS] = {
					DTMF_LOW_FREQ_1, DTMF_LOW_FREQ_2, DTMF_LOW_FREQ_3, DTMF_LOW_FREQ_4,
					DTMF_HIGH_FREQ_1, DTMF_HIGH_FREQ_2, DTMF_HIGH_FREQ_3
				};
				uint32_t dtmf_indices[NUM_DTMF_FREQS];
				q31_t dtmf_magnitudes[NUM_DTMF_FREQS];

				for (int i = 0; i < NUM_DTMF_FREQS; i++) {
					float32_t target_freq = dtmf_freqs[i];
					float32_t bin_width = SAMPLE_RATE / FFT_SIZE;
					dtmf_indices[i] = (uint32_t)(target_freq / bin_width + 0.5f);
					dtmf_magnitudes[i] = (dtmf_indices[i] < FFT_SIZE / 2) ? fft_output[dtmf_indices[i]] : 0;
				}

				int low_freq_index = -1;
				q31_t max_low_magnitude = 0;
				int high_freq_index = -1;
				q31_t max_high_magnitude = 0;

				for (int i = 0; i < 4; i++) {
					if (dtmf_magnitudes[i] > max_low_magnitude && dtmf_magnitudes[i] > MAGNITUDE_THRESHOLD) {
						max_low_magnitude = dtmf_magnitudes[i];
						low_freq_index = i;
					}
				}
				for (int i = 4; i < 7; i++) {
					if (dtmf_magnitudes[i] > max_high_magnitude && dtmf_magnitudes[i] > MAGNITUDE_THRESHOLD) {
						max_high_magnitude = dtmf_magnitudes[i];
						high_freq_index = i - 4;
					}
				}

				if (low_freq_index != -1 && high_freq_index != -1) {
					char dtmf = dtmf_map[low_freq_index][high_freq_index];
					printf("DTMF Tone Detected: %c \r\n", dtmf);
				}
				else {
				    printf("No DTMF Tone Detected\r\n");
				}
				audio_state = STATE_BUFFERING; // Go back to buffering

				// Enable timer interrupt again
				XScuTimer_Start(TimerInstancePtr);
				break;
			}
			default:
				// Do nothing if not in processing state
				break;
		}
}

int main()
{
	int Status;
    init_platform();
	//Configure the IIC data structure
	IicConfig(XPAR_XIICPS_0_DEVICE_ID);

	//Configure the Audio Codec's PLL
	AudioPllConfig();

	//Configure the Line in and Line out ports.
	//Call LineInLineOutConfig() for a configuration that
	//enables the HP jack too.
	AudioConfigureJacks();
	LineinLineoutConfig();

	print("DTMF Detection Demo (8kHz Sampling)\n\r");
	    print("=================================================\n\r");

	    // Initialize FFT instance
	    Status = arm_rfft_init_1024_q31(&fft_instance, ifftFlag, doBitReverse);
	    if (Status != ARM_MATH_SUCCESS) {
	        print("FFT Initialization Failed\r\n");
	        while (1);
	    }

	    XScuTimer Scu_Timer;
	    XScuTimer_Config *Scu_ConfigPtr;
	    XScuGic IntcInstance;

	    Scu_ConfigPtr = XScuTimer_LookupConfig(XPAR_PS7_SCUTIMER_0_DEVICE_ID);
	    Status = XScuTimer_CfgInitialize(&Scu_Timer, Scu_ConfigPtr, Scu_ConfigPtr->BaseAddr);
	    if (Status != XST_SUCCESS) {
	        print("Timer Initialization Failed\r\n");
	        cleanup_platform();
	        return -1;
	    }

	    Status = Timer_Intr_Setup(&IntcInstance, &Scu_Timer, XPS_SCU_TMR_INT_ID);
	    if (Status != XST_SUCCESS) {
	        print("Interrupt Setup Failed\r\n");
	        cleanup_platform();
	        return -1;
	    }

	    // Configure timer to trigger at the sampling rate (8kHz)
	    XScuTimer_LoadTimer(&Scu_Timer, (XPAR_PS7_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2) / (unsigned int)SAMPLE_RATE);
	    XScuTimer_EnableAutoReload(&Scu_Timer);
	    XScuTimer_Start(&Scu_Timer);

	    print("Listening for DTMF tones...\r\n");
	    audio_state = STATE_BUFFERING;

	    for (;;) {
	        // Main loop - interrupt driven processing
	    	process_audio_data(&Scu_Timer); // Call the function to process audio data
	    }

	    cleanup_platform();
	    return 0;
}
