#include "Neopixel.h"
#include "main.h"
#include <math.h>

//matriz para almacenar datos del # led y RGB_Color
uint8_t LED_Data[numPixels][4];

//matriz si se quiere guardar el brillo de los leds
uint8_t LED_Mod[numPixels][4];

//tamaño buffer para almacenar los PWM que saldran por DMA
uint16_t pwmData[(24 * numPixels) + 50];
volatile uint8_t datasentflag = 0;

extern float brilloled;

//carga un RGB_Color deseado al led deseado
void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b){
	if (n >= numPixels) // evita escribir fuera del arreglo
	    {
	        return;
	    }

	LED_Data[n][0] = n;
	LED_Data[n][1] = g;
	LED_Data[n][2] = r;
	LED_Data[n][3] = b;
}


void setBrightness(uint8_t b){
	if(b > 100)
	   b = 100;
	brilloled = b;
	for(int i = 0; i < numPixels; i++){
		LED_Mod[i][0] = LED_Data[i][0];
		for(uint8_t j = 1; j < 4; j++){
			//aplicar correccion gamma y brillo
			uint8_t brill = Gamma_correccion(LED_Data[i][j], brilloled);
			LED_Mod[i][j] = brill;
		}
	}

}

void pixelShow(void){
	uint32_t RGB_Color = 0;
	uint32_t indx = 0;

	for(int i = 0; i < numPixels; i++){
		LED_Mod[i][0] = LED_Data[i][0];
		for(uint8_t j = 1; j < 4; j++){
			//aplicar correccion gamma y brillo
			uint8_t brill = Gamma_correccion(LED_Data[i][j], brilloled);
			LED_Mod[i][j] = brill;
		}

		RGB_Color = ((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));

		for (int i = 23; i >= 0; i--) {
		            if (RGB_Color & (1 << i)) {
		                pwmData[indx] = CCR_1;  // Poner 1
		            } else
		                pwmData[indx] = CCR_0; // Poner 0
		            indx++;
		        }

		    }
		    for (int i = 0; i < 50; i++) {
		        pwmData[indx] = 0;
		        indx++;
		    }

		    datasentflag = 0;

		    if (HAL_TIM_PWM_Start_DMA(&neoPixel_timer, neoPixel_canal, (uint32_t *)pwmData, indx) != HAL_OK)
		    {
		        Error_Handler();
		    }

		    while (!datasentflag)
		    {
		        /* Esperar a que termine el envío por DMA */
		    }

		    datasentflag = 0;
}

void pixelClear(void){
	for(uint8_t i = 0; i < numPixels; i++){
		LED_Data[i][0] = i;
		LED_Data[i][1] = 0;
		LED_Data[i][2] = 0;
		LED_Data[i][3] = 0;
	}
}

uint8_t Gamma_correccion(uint8_t c, float b){
	//limitar el valor de brillo a un rango de 0-100
	if (b > 100.0f)
		b = 100.0f;
	if (b < 0.0f)
		b = 0.0f;

	// normalizo el brillo al rango de 0.0-1.0
	float brillo_Factor = b / 100.0f;

	//aplicar el factor
	float adjustedColor = c * brillo_Factor;

	//formula de escalamiento gamma
	float gammaCorrectedColor = pow((adjustedColor / 255.0f), GAMMA) * 255.0f;

	 //analiza que este dentro los limites
	 if (gammaCorrectedColor > 255.0f)
         gammaCorrectedColor = 255.0f;
     if (gammaCorrectedColor < 0.0f)
    	 gammaCorrectedColor = 0.0f;

    // lo regresa como 8 bit escalado
     return (uint8_t) gammaCorrectedColor;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if ((htim->Instance == neoPixel_timer.Instance) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)) {
		HAL_TIM_PWM_Stop_DMA(&neoPixel_timer, neoPixel_canal);
		datasentflag = 1;
	}
}
