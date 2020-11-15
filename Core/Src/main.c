/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    // 参数1： 使用哪个usart ,参数2： 字符， 参数3：输出数量，参数4：超时等待时间
    HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1 , 0xffff);
    return ch;
}

//printf 替换成log
#define LOG_ENABLE 1
#if LOG_ENABLE
#define log(format,...) printf(format"\r\n",##__VA_ARGS__)
#else
#define log(format,...)
#endif
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
uint8_t data[20]={0};

//接收usart1串口数据  回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    //收到数据
    //发送数据到usart
    HAL_UART_Transmit(&huart1,data,1,3000);
    //开启中断
    HAL_UART_Receive_IT(&huart1,data,1);
}
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
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    /*********************** 蜂鸣器 ***********************/
//  //响起来
//  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
//  //延迟2秒钟
//  HAL_Delay(2000);
//  //声音消失
//  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);

    //读取开关状态
    /*********************** 串口发送数据 ***********************/
//    uint8_t data[] = "hello";
//    HAL_UART_Transmit(&huart1,data, sizeof(data),3000);

    /*********************** 开启中断 ***********************/
    HAL_UART_Receive_IT(&huart1,data,1);

    /*********************** 电机控制 ***********************/
//    //正反转
//    //开启定时器
//    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
//
//    //通电 可以控制正反转
//    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
//
//    //速度控制 PWM  7200 3000  3000/7200
//    __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,7200);

    /*********************** 驱动舵机 ***********************/
    //开启定时器
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    //驱动舵机 PWM
    //0度
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 500);
    //等2秒钟
    HAL_Delay(2000);
    //45度
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 1000);
    //等2秒钟
    HAL_Delay(2000);
    //90度
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 1500);
    //等2秒钟
    HAL_Delay(2000);
    //135度
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 2000);
    //等2秒钟
    HAL_Delay(2000);
    //180度
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, 2500);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1) {
//        //温度
////        printf("temputer:%d\r\n",30);
////        log("temputer:%d",30);
//        //uint_8  8bit
//        //float 4 32位
//        //串口通信  115200  /100
//        printf("termputer:%hd\r\n",(short)(36.5*100));
//        HAL_Delay(1000);
        /*********************** 接收串口数据 ***********************/
        /**
         * 参数1:串口
         * 参数2:用来接收收到的数据
         * 参数3:数据内存大小
         * 参数4:超时时间
         */
//        uint8_t data[20]={0};
//        HAL_UART_Receive(&huart1,data, sizeof(data),3000);
//        //打印收到的数据
//        HAL_UART_Transmit(&huart1,data,1,3000);
//        if(0)
        /*********************** 不断发送串口消息 ***********************/
//        uint8_t data[] = "hello";
//        HAL_UART_Transmit(&huart1,data, sizeof(data),3000);
//        HAL_Delay(1000);
        /*********************** 开关按钮 ***********************/
//        //读取开关按钮状态
//        GPIO_PinState state = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
//        if (state == GPIO_PIN_RESET) {
//            //如果低电平 亮led灯
//            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
//        } else if (state == GPIO_PIN_SET) {
//            //如果高电平  灭led灯
//            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);
//        }
        /*********************** 蜂鸣器 ***********************/
//        //响起来
//        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
//        //延迟2秒钟
//        HAL_Delay(2000);
//        //声音消失
//        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
//        HAL_Delay(2000);
        /*********************** led灯 ***********************/
//        //亮灯
//        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
//        //等待2秒钟
//        //参数:等待毫秒值
//        HAL_Delay(2000);
//        //灭灯
//        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);
//        //等待2秒钟
//        HAL_Delay(2000);
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
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
