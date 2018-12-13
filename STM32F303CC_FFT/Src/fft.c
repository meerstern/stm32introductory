#include "stm32f3xx_hal.h"

#include "fft.h"

#include "arm_math.h"
#include "arm_const_structs.h"


#ifdef FFT_Float
	float FFT_Input[LENGTH_SAMPLES];
	float FFT_Output[LENGTH_SAMPLES/2];
#else
	q15_t FFT_Input[LENGTH_SAMPLES];
	q15_t FFT_Output[LENGTH_SAMPLES/2];
#endif



int32_t ResFFT[LENGTH_SAMPLES/2];
uint32_t RangeData[4][ShiftWidth+1];

uint32_t g_ADCValue;
int g_MeasurementNumber;

uint16_t g_ADCBuffer[ADC_BUFFER_LENGTH];

int ADC_Count;
char fft_status;

char flag=0;

uint32_t adcValue = 0;
uint32_t volt = 0;
uint32_t refIndex = 213, testIndex = 0;

arm_status status=ARM_MATH_SUCCESS;
//status = ARM_MATH_SUCCESS;
float32_t maxValue;
unsigned int MaxValue[4]={0,0,0,0};






void Init_dsp_FFT(){

	fft_status=WAIT;
    //printf("Clock: %d \n", (SystemCoreClock / 1000000) );
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&g_ADCBuffer, ADC_BUFFER_LENGTH);
	HAL_TIM_Base_Start_IT(&htim2);
}



void Exec_dsp_FFT(){

	  for(int j=0;j<LENGTH_SAMPLES;j++){
		//printf("%d, ",g_ADCBuffer[j]);
	  }


#ifdef FFT_Float
		  arm_cfft_f32(&FFTLen, FFT_Input, IFFTFLAG, DOBITREV);
		  arm_cmplx_mag_f32(FFT_Input, FFT_Output, fftSize);
		  arm_max_f32(&FFT_Output[MaxValOffset], (fftSize/2)-MaxValOffset, &maxValue, &testIndex);
		  testIndex=testIndex+MaxValOffset;
#else
		  arm_cfft_q15(&FFTLen, FFT_Input, IFFTFLAG, DOBITREV);
		  arm_cmplx_mag_q15(FFT_Input, FFT_Output, fftSize);
		  arm_max_q15(FFT_Output, fftSize, &maxValue, &testIndex);
#endif



			  for(int i=0;i<fftSize;i+=1){
	//			  q15_t fi;
	//			  q15_t fq;
	//			  q31_t res;
	//			  fi=FFT_Input[2*i];
	//			  fq=FFT_Input[2*i+1];
	//			  res=fi*fi+fq*fq;
	//			  FFT_Output[i]=sqrtf(res);
	//			  uint32_t tmp=*((uint32_t *)FFT_Input + i); // real & imag
	//			  uint32_t magsq = tmp* tmp;
	//			  FFT_Output[i]=sqrtf(magsq);
			  }


#ifdef OUTPUT_FFT_Alldata
			for(int j=0;j<fftSize/2;j++){
	#ifdef FFT_Float
			  printf("%f, ",FFT_Output[j]);
	#else
			  printf("%d, ",FFT_Output[j]);
	#endif
			  }
			  printf(" \n");
#endif


#ifdef OUTPUT_Calc_frq

			  float frq_cen[4];
			  float dfrq[4];
			  char ChrCFrq[4][7];
			  char ChrDFrq[4][7];
			  char ChrMax[4][5];

			  for(int k = 0; k<4 ; k++){

				  int shift=0;
				  if(k==0)		shift=Shift10k;
				  else if(k==1)	shift=Shift20k;
				  else if(k==2)	shift=Shift30k;
				  else if(k==3)	shift=Shift40k;

				  for(int i = 0; i < ShiftWidth; i++) {

					  RangeData[k][i]=FFT_Output[i+shift]/1;

					  if(FFT_Output[i+shift]>MaxValue[k]){
						  MaxValue[k]=FFT_Output[i+shift];
					  }


				  }

				  Calc_frq(ShiftWidth, shift, RangeData[k], &frq_cen[k], &dfrq[k]);

				  printf(", %2.4f",frq_cen[k]);
				  printf(", %2.4f,",dfrq[k]);
				  printf(" %d, ",MaxValue[k]);

			  }


			  for(int l = 0; l<4 ; l++){
			  		frq_cen[l]=0.0;
			  		dfrq[l]=0.0;
			  		MaxValue[l]=0.0;
			  }

#endif

#ifdef OUTPUT_MAX_frq
			  float maxfrq=((float)(testIndex)*(float)SMP_kHz/(float)LENGTH_SAMPLES*2.0);
			  printf("MAX Indx:%d, frq:%.1f val:%.1f \n\r",(int16_t)testIndex,maxfrq,maxValue);
#endif

			  //printf(" \n\n");
				 // HAL_Delay(10);

				  //Re-Start DMA Transfer
				  //HAL_Delay(500);
				  fft_status=WAIT;


}

void ReStartAdcDma(){
	  HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer, ADC_BUFFER_LENGTH);
	  //GPIOA->BSRR= GPIO_PIN_4;//LED ON

	  HAL_TIM_Base_Start_IT(&htim2);
}



//Calc Frq Char
#ifdef FFT_Float
void Calc_frq(uint16_t num, uint16_t Fshift,  float data[], float *frq_cen,float *dfrq){
#else
void Calc_frq(uint16_t num, uint16_t Fshift,  uint32_t data[], float *frq_cen,float *dfrq){
#endif



	uint16_t i;
	float fn;
	float p_sum=0.0,pf_sum=0.0,pf2_sum=0.0;
	float f_cen=0.0,tmp=0.0,df=0.0;

	for( i=0; i < num; i++){
		fn=((float)(i+Fshift)*(float)SMP_kHz/(float)(fftSize));
		tmp=(float)data[i]*fn;
		pf_sum=pf_sum+tmp;
		p_sum=p_sum+data[i];

	}
	if(0.0<p_sum)f_cen=pf_sum/p_sum;
	else f_cen=0.0;
	*frq_cen=f_cen;

	for( i=0; i < num; i++){
		fn=((float)(i+Fshift)*(float)SMP_kHz/(float)(fftSize));
		tmp=(fn-f_cen)*(fn-f_cen);
		tmp=(float)data[i]*tmp;
		pf2_sum=pf2_sum+tmp;
	}
	if(0.0<p_sum)df=pf2_sum/p_sum;
	else df=0.0;
	//df=pow(df,0.5);
	//df=sqrt(df);
	arm_sqrt_f32(df,&df);
	//df=sqrtf(df);
	*dfrq=df;
}

