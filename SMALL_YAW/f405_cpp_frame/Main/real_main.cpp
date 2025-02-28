#include "real_main.hpp"
#include "BMI088driver.hpp"
#include "fire_task.hpp"
#include "gimbal_task.hpp"
#include "bsp_can.hpp"
#include "dm_mit_mode.hpp"

/******************************************************外部声明************************************************/

static BSP_DWT_n::BSP_DWT_c* mydwt = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();

DM_Motor_n::DM_Mit_Mode_c *pPitch_motor = nullptr;
void pitch_rx_call_backk(BSP_CAN_Part_n::CANInstance_c *register_instance);
using namespace DM_Motor_n;

float Vel[5] = {0,0,0,0,0};

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();

  // 创建对象，然后调用成员函数
  // static BMI088Instance_c bmi088_test;
	// bmi088_test.BMI088Init(&hspi1, 1);
  // // 读取陀螺仪数据
  // const INS_t* INS = bmi088_test.Get_INS_Data_Point();

  // //陀螺仪数据出错软重启/倒着上电软重启
  // if(-INS->Roll>30||-INS->Roll<-40) //Pitch
  // {
  //     __ASM volatile ("cpsid i"); //禁用中断
  //     HAL_NVIC_SystemReset(); //软件复位
  // }

  DM_Motor_n::DM_ModePrame_s congfig_ = {
            // 速度kp-0.00372 0.01572
            .kp_max = 500,
            .kp_min = 0,
            .kd_min = 0,
            .kd_max = 5,
            .v_min = -45,
            .v_max = 45,
            .p_min = -12.5,
            .p_max = 12.5,
            .t_min = -54,
            .t_max = 54,
            // 位置速度
            .postion_bits = 16,
            .velocity_bits = 12,
            .toeque_bits = 12,
            .kp_bits = 12,
            .kd_bits = 12,
            .can_init_config = {
                .can_handle = &hcan2,
                .tx_id = 0xD4,  //Slave_ID
                .rx_id = 0xC4,  //Master_ID
                .SAND_IDE = CAN_ID_STD,
            },
            .output = {.p_des = 0, .v_des = 0, .Kp = 0, .Kd = 0, .t_des = 0},
        };
        DM_Motor_n::DM_ModePrame_s *pconfig = new DM_Motor_n::DM_ModePrame_s(congfig_);
        pPitch_motor = new DM_Motor_n::DM_Mit_Mode_c(pconfig);
        pPitch_motor->ECF_SetRxCallBack(pitch_rx_call_backk);
        //pPitch_motor->StateSet(DM_PORTECT_ZERO_POSITION);
        pPitch_motor->StateSet(DM_ENABLE);
  while(1)
  {
      pPitch_motor->Transmit(Vel[0],Vel[1],Vel[2],Vel[3],Vel[4]);
      mydwt->ECF_DWT_Delay_ms(3);
  }

  MX_FREERTOS_Init();

  osKernelStart();

  while (1)
  {

  }
  
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void pitch_rx_call_backk(BSP_CAN_Part_n::CANInstance_c *register_instance) // 够潮的dm框架
    {
        int p_int, v_int, t_int;
        uint8_t rx_buf[8] = {0};
        memcpy(rx_buf, register_instance->rx_buff, 8); // 存储数据，防止变化
        pPitch_motor->get_data_.nowState = (DM_Motor_n::DM_NowState_e)(rx_buf[0]  >> 4);
        p_int = (rx_buf[1] << 8) | rx_buf[2];
        v_int = (rx_buf[3] << 4) | (rx_buf[4] >> 4);
        t_int = ((rx_buf[4] & 0xF) << 8) | rx_buf[5];
        pPitch_motor->get_data_.mos_temperture = rx_buf[6];
        pPitch_motor->get_data_.motor_temperture = rx_buf[7];
        pPitch_motor->get_data_.postion = DM_Motor_n::uint_to_float(p_int, -2, 2, 16); // (-12.5,12.5)  //单位是转
        pPitch_motor->get_data_.velocity = DM_Motor_n::uint_to_float(v_int, -45, 45, 12);    // (-45.0,45.0)
        pPitch_motor->get_data_.toeque = DM_Motor_n::uint_to_float(t_int, -18, 18, 12);      //(-18.0,18.0)
    }
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */