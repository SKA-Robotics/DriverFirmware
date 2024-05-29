#include "system.h"
#include "output.h"

void SystemClock_Config(void) {
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {
        .ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2,
        .SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
        .AHBCLKDivider = RCC_SYSCLK_DIV1,
        .APB1CLKDivider = RCC_HCLK_DIV4,
        .APB2CLKDivider = RCC_HCLK_DIV1,
    };
    RCC_OscInitTypeDef RCC_OscInitStruct = {
        .OscillatorType = RCC_OSCILLATORTYPE_HSE,
        .HSEState = RCC_HSE_ON,
        .PLL.PLLState = RCC_PLL_ON,
        .PLL.PLLSource = RCC_PLLSOURCE_HSE,
        .PLL.PLLMUL = RCC_PLL_MUL9,
    };
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // UART
    GPIO_InitStruct.Pin = UART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH,
    HAL_GPIO_Init(UART_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(UART_PORT, &GPIO_InitStruct);

    // LEDs
    GPIO_InitStruct.Pin =
        LED_POWER_PIN | LED_CAN_PIN | LED_ENC0_PIN | LED_ENC1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW,
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // PWM
    GPIO_InitStruct.Pin = PWM0_CH1_PIN | PWM0_CH2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH,
    HAL_GPIO_Init(PWM0_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = PWM1_CH1_PIN | PWM1_CH2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH,
    HAL_GPIO_Init(PWM1_PORT, &GPIO_InitStruct);

    // ADC
    GPIO_InitStruct.Pin = THERMISTOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(THERMISTOR_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = CURRENT0_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(CURRENT0_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = CURRENT1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(CURRENT1_PORT, &GPIO_InitStruct);

    // SPI
    GPIO_InitStruct.Pin = SPI_MOSI_PIN | SPI_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI_PORT, &GPIO_InitStruct);

    // Chip select pins
    GPIO_InitStruct.Pin = CS_ENC0_PIN | CS_DRV0_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CS_ENC0DRV0_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = CS_ENC1_PIN | CS_DRV1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CS_ENC1DRV1_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(CS_ENC0DRV0_PORT, CS_ENC0_PIN | CS_DRV0_PIN,
                      GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_ENC1DRV1_PORT, CS_ENC1_PIN | CS_DRV1_PIN,
                      GPIO_PIN_SET);

    // Driver nFault pins
    GPIO_InitStruct.Pin = NFAULT0_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(NFAULT0_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = NFAULT1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(NFAULT1_PORT, &GPIO_InitStruct);

    // CAN pins
    GPIO_InitStruct.Pin = CAN_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CAN_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAN_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAN_PORT, &GPIO_InitStruct);

    // // Remap alternate functions
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_AFIO_REMAP_TIM1_PARTIAL();
    __HAL_AFIO_REMAP_TIM3_ENABLE();
    __HAL_AFIO_REMAP_USART3_DISABLE();
    __HAL_AFIO_REMAP_CAN1_1();
}

void MX_TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Period = 10 - 1;       // 100 Hz
    htim2.Init.Prescaler = 36000 - 1; // Tick every millisecond
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void MX_TIM4_Init(void) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    htim4.Instance = TIM4;
    htim4.Init.Period = 100 - 1;      // 10 Hz
    htim4.Init.Prescaler = 36000 - 1; // Tick every millisecond
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim4);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void MX_PWM_Init(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    htim1.Instance = TIM1;
    htim1.Init.Period = MAX_PWM;
    htim1.Init.Prescaler = 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_OC_InitTypeDef OC_InitStruct = {
        .OCMode = TIM_OCMODE_PWM1,
        .Pulse = 0,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
    };
    HAL_TIM_PWM_Init(&htim1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &OC_InitStruct, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &OC_InitStruct, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Period = MAX_PWM;
    htim3.Init.Prescaler = 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim3);
    HAL_TIM_PWM_ConfigChannel(&htim3, &OC_InitStruct, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3, &OC_InitStruct, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void MX_ADC_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma1.Instance = DMA1_Channel1;
    hdma1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma1.Init.MemInc = DMA_MINC_ENABLE;
    hdma1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma1.Init.Mode = DMA_CIRCULAR;
    hdma1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma1);
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma1);

    __HAL_RCC_ADC1_CLK_ENABLE();
    RCC_PeriphCLKInitTypeDef adcClk_InitStruct = {
        .PeriphClockSelection = RCC_PERIPHCLK_ADC,
        .AdcClockSelection = RCC_ADCPCLK2_DIV4,
    };
    hadc1.Instance = ADC1;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.NbrOfConversion = ADC_CHANNELS;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion = 1;
    HAL_RCCEx_PeriphCLKConfig(&adcClk_InitStruct);
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef channelInitStruct = {0};
    channelInitStruct.SamplingTime = ADC_SAMPLETIME_28CYCLES5_SMPR1ALLCHANNELS;
    channelInitStruct.Channel = ADC_CHANNEL_8;
    channelInitStruct.Rank = ADC_REGULAR_RANK_1;
    HAL_ADC_ConfigChannel(&hadc1, &channelInitStruct);
    channelInitStruct.Channel = ADC_CHANNEL_15;
    channelInitStruct.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&hadc1, &channelInitStruct);
    channelInitStruct.Channel = ADC_CHANNEL_9;
    channelInitStruct.Rank = ADC_REGULAR_RANK_3;
    HAL_ADC_ConfigChannel(&hadc1, &channelInitStruct);

    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, ADC_CHANNELS);
}

void MX_SPI_Init(void) {
    __HAL_RCC_SPI1_CLK_ENABLE();
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        ErrorHandler();
    }
    __HAL_SPI_ENABLE(&hspi1);
}

void MX_USART3_UART_Init(void) {
    __HAL_RCC_USART3_CLK_ENABLE();
    huart.Instance = USART3;
    huart.Init.BaudRate = 115200;
    huart.Init.WordLength = UART_WORDLENGTH_8B;
    huart.Init.StopBits = UART_STOPBITS_1;
    huart.Init.Parity = UART_PARITY_NONE;
    huart.Init.Mode = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart) != HAL_OK) {
        ErrorHandler();
    }
}

void MX_CAN_Init(void) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    hcan.Instance = CAN1;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    // 1 Mbit/s
    hcan.Init.Prescaler = 2;
    hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK) {
        ErrorHandler();
    }
    // TODO: Skonfiguruj porządnie filtr
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        ErrorHandler();
    }
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        ErrorHandler();
    }
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) !=
        HAL_OK) {
        ErrorHandler();
    };
}

uint16_t readAdc(uint32_t channel) { return adcBuffer[channel]; }

CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxData[8];

int transmitCanFrame(uint16_t frameId, uint64_t data, uint32_t timeout) {
    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = frameId;
    // If you just pass the uint64_t as a uint8_t* to be transmitted, you end up
    // with least significant byte first.
    TxData[0] = (data >> 56) & 0xFF;
    TxData[1] = (data >> 48) & 0xFF;
    TxData[2] = (data >> 40) & 0xFF;
    TxData[3] = (data >> 32) & 0xFF;
    TxData[4] = (data >> 24) & 0xFF;
    TxData[5] = (data >> 16) & 0xFF;
    TxData[6] = (data >> 8) & 0xFF;
    TxData[7] = data & 0xFF;
    uint32_t startTick = HAL_GetTick();
    while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) &&
           ((HAL_GetTick() - startTick) < timeout)) {
        ; // treat it as a punishment
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
        return HAL_ERROR;
    }
    return HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

void ErrorHandler(void) {
    __disable_irq();
    printf("Error\n");
    while (1) {
    };
}