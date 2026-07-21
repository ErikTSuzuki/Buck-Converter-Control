/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*==============================================================================
 * CONTROLE DIGITAL DO CONVERSOR BUCK - Sinal 1 (trapezoidal), fs = 10 kHz
 *
 * Estrutura fiel ao CA10 (PID em equacao de diferencas, slide 11) e ao P-A1
 * (rotina de controle inteira na interrupcao de fim de conversao do ADC,
 * main em laco vazio). Adaptada do projeto Controle_Digital_V2 (funcionando
 * com Sinal 6 em 7,5 kHz) para o Sinal 1 do WP1/WP2:
 *   - trapezio com periodo 40 ms (f1 = 25 Hz), nivel medio 12 V
 *   - patamares 12 V / 18 V / 12 V / 6 V / 12 V, rampas de 1 ms
 *   - fs = 10000 Hz -> N = 400 amostras por ciclo (Ts = 100 us)
 *============================================================================*/

//Variaveis do slide (CA10 - slide 11)
volatile float ek = 0.0f;
volatile float ek1 = 0.0f;

volatile float uPk = 0.0f;
volatile float uIk = 0.0f;
volatile float uIk1 = 0.0f;
volatile float uDk = 0.0f;
volatile float uPIDk = 0.0f;

//Ganhos do Controle/Sistema  ->  CONFIRMAR/AJUSTAR EM BANCADA
//Ponto de partida: ganhos do projeto de referencia (Kp=0.05, Ki=0.025,
//Kd=0.08 @ 7,5 kHz, CCR 0-400, Kc~41 counts/V) reescalados para este
//hardware (10 kHz, CCR 0-3600, Kc~83 counts/V):
//  Kp' = Kp*(3600/400)*(41.3/82.7)            ~ 0.22
//  Ki' = Ki*(3600/400)*(41.3/82.7)*(7500/10k) ~ 0.08
//  Kd' = Kd*(3600/400)*(41.3/82.7)*(10k/7500) ~ 0.45
volatile float Kp = 0.27f;
volatile float Ki = 0.10f;
volatile float Kd = 0.55f;
volatile float TS = 0.0001f;             //Periodo de amostragem = 1/10000 s

volatile uint16_t pwm_ref = 0;           //CCR do canal de visualizacao (CH4/PA11)
volatile uint16_t pwm_buck = 0;          //CCR aplicado ao buck (CH1/PA8)
volatile float duty_buck = 0.0f;

volatile float ref = 12.0f;              //Referencia instantanea em volts
volatile float ref_ADC = 0.0f;           //Referencia convertida para counts do ADC
volatile uint16_t adc_value = 0;         //Amostra de VoA (12 bits) p/ Live Expressions

//Counts do ADC medidos com Vo = 6 V e Vo = 18 V  ->  CONFIRMAR EM BANCADA
//Valores teoricos p/ condicionamento do P-A1 (Vo 0-30V -> 0-2V, offset +0,5V,
//VDDA = 3,3 V):  6 V -> 0,9 V -> 1117 counts ; 18 V -> 1,7 V -> 2109 counts
#define ADC_VOUT_MIN  1295.0f
#define ADC_VOUT_MAX  2100.0f

#define REF_MIN 6.0f                     //Patamar minimo do Sinal 1
#define REF_MAX 18.0f                    //Patamar maximo do Sinal 1

//Constante de conversao volts -> counts do ADC (deriva da calibracao acima)
volatile float Kc = (ADC_VOUT_MAX - ADC_VOUT_MIN) / (REF_MAX - REF_MIN);

//PWM: TIM1 center-aligned, ARR = 3600 -> fPWM = 72MHz/(2*3600) = 10 kHz
#define PWM_PERIOD 3600.0f               //Valor do duty maximo (CCR = ARR)
#define DUTY_MIN   180.0f                //5%  - nao deixa o PWM saturar
#define DUTY_MAX   3420.0f               //95% - preserva refresh do bootstrap

//Sinal 1 (WP1 p.13 / WP2 p.9): 400 amostras por ciclo @ 10 kHz = 40 ms
//Segmentos (em amostras de 100 us):
//  k =   0..49  : 12 V constante        (0 - 5 ms)
//  k =  50..59  : rampa 12 -> 18 V      (5 - 6 ms)
//  k =  60..149 : 18 V constante        (6 - 15 ms)
//  k = 150..159 : rampa 18 -> 12 V      (15 - 16 ms)
//  k = 160..249 : 12 V constante        (16 - 25 ms)
//  k = 250..259 : rampa 12 -> 6 V       (25 - 26 ms)
//  k = 260..349 : 6 V constante         (26 - 35 ms)
//  k = 350..359 : rampa 6 -> 12 V       (35 - 36 ms)
//  k = 360..399 : 12 V constante        (36 - 40 ms)
#define N_CICLO  400                     //Amostras por periodo do sinal
#define N_RAMPA  10                      //Amostras de cada rampa (1 ms)
#define PASSO_RAMPA 0.6f                 //6 V / 10 amostras = 0,6 V por amostra

//==============================================================================
// MODO CALIBRACAO (teste de bancada)
//   1 = malha ABERTA: CCR1 vem de pwm_manual (ajuste ao vivo pelo Live
//       Expressions) e adc_value continua sendo amostrado a 10 kHz.
//       Use para levantar ADC_VOUT_MIN/ADC_VOUT_MAX e conferir o sentido
//       do driver (pwm_manual maior => Vo maior, gracas a polaridade LOW).
//   0 = malha FECHADA: controle PID normal com o Sinal 1.
//==============================================================================
#define MODO_CALIBRACAO 0

volatile uint16_t pwm_manual = 0;        //CCR1 manual: 0=chave aberta; 900=25%; 2700=75% (max 3420)

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   //PWM do buck no PA8 (parte com CCR1 = 0 -> chave aberta)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   //Canal interno: trigger do ADC no pico do contador
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   //Visualizacao da referencia no PA11

  HAL_ADC_Start_IT(&hadc1);                   //ADC disparado pelo TIM1_CC2, interrupcao de fim de conversao
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)  //Rotina de controle @ 10 kHz
{ //ADC Convertion Complete Callback

#if MODO_CALIBRACAO
    //Malha aberta: le o ADC e aplica o duty manual definido no Live Expressions.
    //Ajuste pwm_manual ate o multimetro marcar 6,00 V (anote adc_value ->
    //ADC_VOUT_MIN) e depois 18,00 V (anote adc_value -> ADC_VOUT_MAX).
    adc_value = HAL_ADC_GetValue(hadc);

    if(pwm_manual > (uint16_t)DUTY_MAX) pwm_manual = (uint16_t)DUTY_MAX;  //95%: preserva o bootstrap

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_manual);
    return;
#endif

    //Construcao do Sinal 1 (trapezoidal) conforme P-A1 Atividade 3:
    //contador de amostras k, sinal dividido em etapas (constante/rampa),
    //reiniciando o contador ao final de cada ciclo (k == N_CICLO)
    static uint16_t k = 0;

    if      (k < 50)  ref = 12.0f;                                  //patamar 12 V
    else if (k < 60)  ref = 12.0f + PASSO_RAMPA * (float)(k - 50);  //rampa 12->18 V
    else if (k < 150) ref = REF_MAX;                                //patamar 18 V
    else if (k < 160) ref = 18.0f - PASSO_RAMPA * (float)(k - 150); //rampa 18->12 V
    else if (k < 250) ref = 12.0f;                                  //patamar 12 V
    else if (k < 260) ref = 12.0f - PASSO_RAMPA * (float)(k - 250); //rampa 12->6 V
    else if (k < 350) ref = REF_MIN;                                //patamar 6 V
    else if (k < 360) ref = 6.0f + PASSO_RAMPA * (float)(k - 350);  //rampa 6->12 V
    else              ref = 12.0f;                                  //patamar 12 V

    k++;
    if (k >= N_CICLO) k = 0;

    //Amostra de VoA (12 bits, disparada pelo TIM1_CC2 no pico do contador)
    adc_value = HAL_ADC_GetValue(hadc);

    //Converte a referencia de volts para counts do ADC
    ref_ADC = ADC_VOUT_MIN + (ref - REF_MIN) * Kc;

    //Implementacao das equacoes do slide (CA10 - slide 11)
    ek = ref_ADC - (float)adc_value;      //Erro

    uPk = Kp * ek;                        //Acao proporcional. Corrige de acordo com o erro atual.
    uIk = uIk1 + Ki * ek;                 //Acao integral. Soma o erro ao longo do tempo.
    uDk = Kd * (ek - ek1);                //Acao derivativa. Olha a variacao do erro.

    //Anti-windup: o integrador nao pode acumular alem do que o atuador
    //consegue aplicar (0 a 95% do duty). Sem isso, qualquer periodo com
    //Vo abaixo da referencia (ex.: potencia desligada) enrola uIk para
    //centenas de milhares e o PWM fica travado no maximo por minutos.
    if(uIk > DUTY_MAX) uIk = DUTY_MAX;
    if(uIk < 0.0f)     uIk = 0.0f;

    uPIDk = uPk + uIk + uDk;              //Soma as tres acoes do controlador.

    uIk1 = uIk;                           //Salva os valores atuais para a proxima amostra.
    ek1 = ek;

    //Driver com logica invertida ja compensado em HARDWARE (CH1 com
    //polaridade LOW no CubeMX): d = CCR1/3600, proporcionalidade direta.
    //Nao repetir a inversao em software (armadilha da dupla inversao).
    duty_buck = uPIDk;

    if(duty_buck < DUTY_MIN) duty_buck = DUTY_MIN;    //Limita entre 5% e 95%
    if(duty_buck > DUTY_MAX) duty_buck = DUTY_MAX;

    pwm_buck = (uint16_t)duty_buck;                   //Apenas para visualizar no Live Expressions

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_buck);

    //CH4 (PA11): PWM proporcional a referencia, para visualizacao no
    //osciloscopio (apos filtro RC) - 6V=0 : 12V=0.5 : 18V=1
    float ref_norm = (ref - REF_MIN) / (REF_MAX - REF_MIN);

    if(ref_norm > 1.0f) ref_norm = 1.0f;
    if(ref_norm < 0.0f) ref_norm = 0.0f;

    pwm_ref = (uint16_t)(ref_norm * PWM_PERIOD);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_ref);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
