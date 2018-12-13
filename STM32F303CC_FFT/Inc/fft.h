#define ARM_MATH_CM4
#define __FPU_PRESENT	1






//#define OUTPUT_Calc_frq
#define OUTPUT_MAX_frq
#define OUTPUT_FFT_Alldata

#define ENABLE_Hanning_Window




#define FFT_Float		//ENABLE float OR Q15

#define LENGTH_SAMPLES 	4096//256//1024//1024//512  /*ADC LEN*2 =Re & Im */
#define IFFTFLAG		0
#define DOBITREV		1
#define fftSize			LENGTH_SAMPLES/2

#define SMP_kHz	100//80
#define COUNT	30//50

#define PI 3.1415926535

#define MaxValOffset	2	//eliminate DC element.


#if SMP_kHz==80
	#if LENGTH_SAMPLES==512
		#define SHIFT 216//216
	#elif LENGTH_SAMPLES==1024
		#define SHIFT 447//216
	#endif
#elif SMP_kHz==100
	#if LENGTH_SAMPLES==128
		#define ShiftWidth  7
		#define Shift10k	6
		#define Shift20k	12
		#define Shift30k	19
		#define Shift40k	25
	#elif LENGTH_SAMPLES==256
		#define ShiftWidth  13
		#define Shift10k	12
		#define Shift20k	25
		#define Shift30k	38
		#define Shift40k	51
	#elif LENGTH_SAMPLES==512
		#define ShiftWidth  25
		#define Shift10k	25
		#define Shift20k	51
		#define Shift30k	76
		#define Shift40k	102
	#elif LENGTH_SAMPLES==1024
		#define ShiftWidth  51
		#define Shift10k	51
		#define Shift20k	102
		#define Shift30k	153
		#define Shift40k	204
	#elif LENGTH_SAMPLES==2048
		#define ShiftWidth  102
		#define Shift10k	102
		#define Shift20k	204
		#define Shift30k	306
		#define Shift40k	408
	#else
		#define ShiftWidth  1
		#define Shift10k	1
		#define Shift20k	1
		#define Shift30k	1
		#define Shift40k	1
	#endif
#endif


#ifdef FFT_Float
	#if LENGTH_SAMPLES==256
		#define FFTLen arm_cfft_sR_f32_len128
	#elif LENGTH_SAMPLES==512
		#define FFTLen arm_cfft_sR_f32_len256
	#elif LENGTH_SAMPLES==1024
		#define FFTLen arm_cfft_sR_f32_len512
	#elif LENGTH_SAMPLES==2048
		#define FFTLen arm_cfft_sR_f32_len1024
	#elif LENGTH_SAMPLES==4096
		#define FFTLen arm_cfft_sR_f32_len2048
	#elif LENGTH_SAMPLES==8192
		#define FFTLen arm_cfft_sR_f32_len4096
	#endif
#else
	#if LENGTH_SAMPLES==256
		#define FFTLen arm_cfft_sR_q15_len128
	#elif LENGTH_SAMPLES==512
		#define FFTLen arm_cfft_sR_q15_len256
	#elif LENGTH_SAMPLES==1024
		#define FFTLen arm_cfft_sR_q15_len512
	#elif LENGTH_SAMPLES==2048
		#define FFTLen arm_cfft_sR_q15_len1024
	#elif LENGTH_SAMPLES==4096
		#define FFTLen arm_cfft_sR_q15_len2048
	#elif LENGTH_SAMPLES==8192
		#define FFTLen arm_cfft_sR_q15_len4096
	#endif
#endif


extern char fft_status;
enum{ ADC_BUFFER_LENGTH = LENGTH_SAMPLES/2 };//=LENGTH_SAMPLES
enum{ WAIT,RUN};

extern uint16_t g_ADCBuffer[ADC_BUFFER_LENGTH];

#ifdef FFT_Float
	extern float FFT_Input[LENGTH_SAMPLES];
	extern float FFT_Output[LENGTH_SAMPLES/2];
#else
	extern q15_t FFT_Input[LENGTH_SAMPLES];
	extern q15_t FFT_Output[LENGTH_SAMPLES/2];
#endif


void Init_dsp_FFT();
void Exec_dsp_FFT();
void ReStartAdcDma();

#ifdef FFT_Float
	void Calc_frq(uint16_t num, uint16_t Fshift,  float data[], float *frq_cen,float *dfrq);
#else
	void Calc_frq(uint16_t num, uint16_t Fshift,  uint32_t data[], float *frq_cen,float *dfrq);
#endif
