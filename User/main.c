#include "stm32f10x.h"
#include <stdio.h> 

volatile float current_temperature = 0.0f;
volatile float temperature_threshold = 30.0f;
const float threshold_levels[] = {30.0f, 35.0f, 25.0f};
volatile uint8_t threshold_index = 0;
volatile uint8_t adc_sample_flag = 0;
volatile uint8_t uart_send_flag = 0;
volatile uint16_t pwm_breath_counter = 0;
volatile uint8_t breath_direction = 1;
char uart_tx_buffer[50];
volatile uint8_t system_ready = 0;

volatile uint32_t sys_tick_counter = 0;
volatile uint32_t adc_sample_interval_ms = 20;
volatile uint32_t uart_send_interval_ms = 1000;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void ADC_Configuration(void);
uint16_t ADC_ReadValue(void);
float ConvertToCelsius(uint16_t adc_value);
void USART1_Configuration(void);
void USART_SendString(USART_TypeDef* USARTx, char *str);
void Send_Temperature_UART(float temp);
void PWM_TIM2_Configuration(void);
void Update_Breath_PWM(void);
void EXTI_Configuration(void);
void NVIC_Configuration(void);
void SysTick_Configuration(void);
void Green_LED_Blink(uint8_t times, uint32_t delay_ms);
void Update_LED_Status(void);

void ADC_Configuration(void) {
    ADC_InitTypeDef ADC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

uint16_t ADC_ReadValue(void) {
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return			ADC_GetConversionValue(ADC1);
}

float ConvertToCelsius(uint16_t adc_value) {
    return ((float)adc_value / 4095.0f) * 330.0f;
}

void PWM_TIM2_Configuration(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void Update_Breath_PWM(void) {
    if (pwm_breath_counter % 2 == 0) {
        uint16_t current_pulse = TIM2->CCR2;
        if (breath_direction) {
            current_pulse += 4;
            if (current_pulse >= 999) {
                current_pulse = 999;
                breath_direction = 0;
            }
        } else {
            if (current_pulse <= 4) {
                current_pulse = 0;
                breath_direction = 1;
            } else {
                current_pulse -= 4;
            }
        }
        TIM_SetCompare2(TIM2, current_pulse);
    }
    pwm_breath_counter++;
    if (pwm_breath_counter >= 500) {
        pwm_breath_counter = 0;
    }
}

void EXTI_Configuration(void) {
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);

    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

void NVIC_Configuration(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART1_Configuration(void) {
    USART_InitTypeDef USART_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void USART_SendString(USART_TypeDef* USARTx, char *str) {
    while (*str) {
        USART_SendData(USARTx, (uint16_t)(*str++));
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
    }
}

void Send_Temperature_UART(float temp) {
    sprintf(uart_tx_buffer, "Temp: %.1f C\r\n", temp);
    USART_SendString(USART1, uart_tx_buffer);
}

void Send_RawTemperature_UART(float temp) {
    sprintf(uart_tx_buffer, "Raw Temp: %.1f C\r\n", temp);
    USART_SendString(USART1, uart_tx_buffer);
}

void RCC_Configuration(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configuration(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Green_LED_Blink(uint8_t times, uint32_t delay_ms) {
    for (uint8_t i = 0; i < times; ++i) {
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
        for(volatile uint32_t d = 0; d < delay_ms * 10000; ++d);
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        for(volatile uint32_t d = 0; d < delay_ms * 10000; ++d);
    }
}

void Update_LED_Status(void) {
    if (!system_ready) return;
	
    GPIO_SetBits(GPIOA, GPIO_Pin_0);

    if (current_temperature > temperature_threshold) {
        TIM_CtrlPWMOutputs(TIM2, ENABLE); 
    } else {
        TIM_SetCompare2(TIM2, 0);
    }
}

void SysTick_Configuration(void) {
    if (SysTick_Config(SystemCoreClock / 1000)) { 
        while (1);
    }
}

int main(void) {
    SystemInit();

    RCC_Configuration();
    GPIO_Configuration();
    ADC_Configuration();
    USART1_Configuration();
    PWM_TIM2_Configuration();
    EXTI_Configuration();
    NVIC_Configuration();
    SysTick_Configuration();

    Green_LED_Blink(2, 200);
    GPIO_SetBits(GPIOA, GPIO_Pin_0);
    system_ready = 1;
    \
    temperature_threshold = threshold_levels[threshold_index];


    while (1) {
        if (adc_sample_flag) {
            adc_sample_flag = 0;
            uint16_t adc_raw = ADC_ReadValue();
            //current_temperature = ConvertToCelsius(adc_raw);
						current_temperature = 31;
        }

        Update_LED_Status();

        if (uart_send_flag) {
            uart_send_flag = 0;
            if (system_ready) {
                 {
									 Send_Temperature_UART(current_temperature);
									 Send_RawTemperature_UART(ADC_ReadValue());
								 }
            }
        }
    }
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t received_data = USART_ReceiveData(USART1);
        if (received_data == 0x21) {
            char buffer[30];
            sprintf(buffer, "Threshold: %.1f C\r\n", temperature_threshold);
            USART_SendString(USART1, buffer);
        } else {
            USART_SendString(USART1, "invalid instruction.\r\n");
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void EXTI9_5_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        for(volatile int i=0; i < 1000000; i++);

        threshold_index = (threshold_index + 1) % 3;
        temperature_threshold = threshold_levels[threshold_index];
        
        char buffer[30];
        sprintf(buffer, "New Threshold: %.1f C\r\n", temperature_threshold);
        USART_SendString(USART1, buffer);
        
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
}

void SysTick_Handler(void) {
    sys_tick_counter++;

    if (sys_tick_counter % adc_sample_interval_ms == 0) {
        adc_sample_flag = 1;
    }

    if (sys_tick_counter % uart_send_interval_ms == 0) {
        uart_send_flag = 1;
    }
    
    if (system_ready && current_temperature > temperature_threshold) {
        Update_Breath_PWM();
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
    while (1) {}
}
#endif