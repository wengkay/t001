
#include "eth.h"
#include "string.h"

ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUF_SIZE];
uint8_t Tx_Buff[ETH_RX_DESC_CNT][ETH_TX_BUF_SIZE];
ETH_TxPacketConfig TxConfig;

/* USER CODE BEGIN 0 */

static uint8_t txBuffer[ETH_TX_BUF_SIZE];
RxBufferList rxList = {0};

int32_t ETH_PHY_INTERFACE_Init(void);
int32_t ETH_PHY_INTERFACE_DeInit(void);
int32_t ETH_PHY_INTERFACE_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_INTERFACE_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_INTERFACE_GetTick(void);

int HAL_ETH_IsLinkUp(ETH_HandleTypeDef *heth);
void DM9161_EnableInterrupt(void);
void ConfigureRxDescriptors(void);
void ConfigureTxDescriptors(void);
/* USER CODE END 0 */

ETH_HandleTypeDef heth;
														 
									 
/* ETH init function */
void MX_ETH_Init(void)
{
  /* USER CODE BEGIN ETH_Init 0 */
	
  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.MACAddr = &ETH_MAC_ADDR[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;
	
  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */
	// 配置接收描述符缓冲区
  ConfigureRxDescriptors();
	//ConfigureTxDescriptors();
  // 启用 Ethernet MAC（关键）
  if (HAL_ETH_Start_IT(&heth) != HAL_OK)
  {
    Error_Handler();
  }
		
  // 启用中断
  ETH->DMAIER |= ETH_DMAIER_RIE;
//	DM9161_EnableInterrupt();
	__HAL_ETH_DMA_ENABLE_IT(&heth, ETH_DMA_IT_R);

  /* USER CODE END ETH_Init 2 */
}
void ConfigureRxDescriptors(void) {
    for (int i = 0; i < ETH_RX_DESC_CNT; i++) {
        DMARxDscrTab[i].DESC2 = (uint32_t)&Rx_Buff[i][0];
        DMARxDscrTab[i].DESC0 = 0x80000000; // 设置 OWN 位
        DMARxDscrTab[i].DESC3 = (uint32_t)&DMARxDscrTab[(i + 1) % ETH_RX_DESC_CNT]; // 链表环形链接
    }
}

void ConfigureTxDescriptors(void)
{
    for (int i = 0; i < ETH_TX_DESC_CNT; i++) {
        DMATxDscrTab[i].DESC0 = 0;                      // CPU拥有，清空状态位
        DMATxDscrTab[i].DESC1 = 0;                      // 清空长度或状态
        DMATxDscrTab[i].DESC2 = (uint32_t)&Tx_Buff[i]; // 指向发送缓冲区
        DMATxDscrTab[i].DESC3 = (uint32_t)&DMATxDscrTab[(i + 1) % ETH_TX_DESC_CNT]; // 指向下一个描述符
    }
    heth.TxDescList.CurTxDesc = 0; // 复位索引
}




void DM9161_EnableInterrupt(void)
{
    uint32_t val = 0;

    // 读出寄存器 0x1F 当前值
    HAL_ETH_ReadPHYRegister(&heth, 0x01, 0x1F, &val);

    // 设置 bit6: Link change interrupt，bit4: Receive interrupt
    val |= (1 << 6) | (1 << 4);

    // 写回寄存器
    HAL_ETH_WritePHYRegister(&heth, 0x01, 0x1F, val);

    DEBUG_PRINT("PHY INT reg 0x1F configured: 0x%08lX\r\n", val);  // 调试
}

void HAL_ETH_MspInit(ETH_HandleTypeDef* ethHandle)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(ethHandle->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspInit 0 */

  /* USER CODE END ETH_MspInit 0 */
    /* ETH clock enable */
    __HAL_RCC_ETH_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**ETH GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PG11     ------> ETH_TX_EN
    PG13     ------> ETH_TXD0
    PG14     ------> ETH_TXD1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

		/*Configure GPIO pin : ETH_RESET_Pin */
		GPIO_InitStruct.Pin = ETH_RESET_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(ETH_RESET_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pins : ETH_INTR_Pin*/
		GPIO_InitStruct.Pin = ETH_INTR_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ETH_MspInit 1 */
		HAL_GPIO_WritePin(ETH_RESET_GPIO_Port,ETH_RESET_Pin,GPIO_PIN_RESET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(ETH_RESET_GPIO_Port,ETH_RESET_Pin,GPIO_PIN_SET);
		HAL_Delay(100);
  /* USER CODE END ETH_MspInit 1 */
  }
}


void ETH_DM9162_DeInit(void)
{
	/* USER CODE BEGIN ETH_MspDeInit 0 */

  /* USER CODE END ETH_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ETH_CLK_DISABLE();

    /**ETH GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PG11     ------> ETH_TX_EN
    PG13     ------> ETH_TXD0
    PG14     ------> ETH_TXD1
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14);

    /* ETH interrupt Deinit */
    HAL_NVIC_DisableIRQ(ETH_IRQn);
  /* USER CODE BEGIN ETH_MspDeInit 1 */

  /* USER CODE END ETH_MspDeInit 1 */
}

/* USER CODE BEGIN 1 */

int32_t ETH_PHY_INTERFACE_Init(void)
{
    /* Configure the MDIO Clock */
    HAL_ETH_SetMDIOClockRange(&heth);
    return 0;
}
int32_t ETH_PHY_INTERFACE_DeInit(void)
{
		ETH_DM9162_DeInit();
    return 0;
}
int32_t ETH_PHY_INTERFACE_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
    if (HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) != HAL_OK)
    {
        return -1;
    }
    return 0;
}
int32_t ETH_PHY_INTERFACE_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
    if (HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) != HAL_OK)
    {
        return -1;
    }
    return 0;
}
int32_t ETH_PHY_INTERFACE_GetTick(void)
{
    return HAL_GetTick();
}

void ETH_StartLink(void)
{
    ETH_MACConfigTypeDef MACConf = {0};
    int32_t PHYLinkState = 0U;
    uint32_t linkchanged = 0U, speed = 0U, duplex = 0U;
 
}
int dm9162_link_up(void) {
    uint32_t phy_status;
    
    // 读取 PHY_BSR 寄存器
    if (HAL_ETH_ReadPHYRegister(&heth,ETH_PHY_ADDR, PHY_BSR, &phy_status) == HAL_OK) {
        // 检查 LINK_STATUS 位，判断链路是否已连接
        if (phy_status & PHY_LINK_STATUS) {
            return 1;  // 链路已连接
        }
    }
    return 0;  // 链路未连接
}
void ETH_ConstructEthernetFrame(ethernet_frame_t *frame, uint8_t *dest_mac, uint8_t *src_mac, uint8_t *type,
                                uint8_t *payload, uint16_t payload_len)
{
    // Copy the destination MAC address
    memcpy(frame->dest_mac, dest_mac, 6);
    // Copy the source MAC address
    memcpy(frame->src_mac, src_mac, 6);
    // Set the Ethernet type field
    memcpy(frame->type, type, 2);
    // Copy the payload data
    memcpy(frame->payload, payload, payload_len);
}

/**
 * @brief 使用 ETH 发送一个 packet, 若当前 ETH 繁忙, 会等待直到可用 (blocking)
 * @param pBuff: 要发送的数据指针
 * @param len: 要发送的数据长度
 * @retval 0=成功, -1=失败
 */
int ethSend(void *pBuff, int len)
{
    int retval = -1;
    // osSemaphoreAcquire(ethTxCpltSemaphore, osWaitForever);
    memcpy(txBuffer, pBuff, len);
    ETH_BufferTypeDef txBufferDef = {0};
    txBufferDef.buffer = txBuffer;
    txBufferDef.len = len;
    txBufferDef.next = NULL;

//  osMutexAcquire(ethTxConfigMutex, osWaitForever);
    TxConfig.TxBuffer = &txBufferDef;
    if (HAL_ETH_Transmit_IT(&heth, &TxConfig) == HAL_OK)
    {
//        printf("Eth tx start.\r\n");
//        for (uint32_t i = 0; i < txBufferDef.len; i++) {
//            printf("%02x ", ((uint8_t*)pBuff)[i]);
//       }
//        printf("\r\n");
        retval = 0;
    }
    else
    {
        retval = -1;
    }

//    osMutexRelease(ethTxConfigMutex);
    return retval;
}

/**
 * @brief 对 ETH Send 线程发送一个 Send 请求，会将待发送的数据复制到内部缓存中
 * 
 * @param pBuff 指向待发送的数据
 * @param len 数据大小
 * @param timeout 超时时间, 0=none blocking
 * @return int 0=成功, -1=失败
 */
int ethSendRequest(void *pBuff, uint32_t len, uint32_t timeout) {
//    ETH_AppBuff* appBuff = osMemoryPoolAlloc(txBufferPool, timeout);
    // appBuff->buffer
//    if (appBuff) {
//        memcpy(appBuff->buffer, pBuff, len);
//        appBuff->AppBuff.buffer = appBuff->buffer;
//        appBuff->AppBuff.len = len;
 //       appBuff->AppBuff.next = NULL;
 //       if (osMessageQueuePut(ethTxQueue, &appBuff, 0, timeout) == osOK) {
 //           return 0;
 //       }
 //   }

    return -1;
}

/**
 * @brief 供 ethSendThread 调用的阻塞发送函数, polling 模式
 * 
 * @param appBuff 
 * @param timeout 
 */
void ethSendProcess(ETH_AppBuff* appBuff, uint32_t timeout) {
    //osStatus_t osStat;
    HAL_StatusTypeDef halStat;
    //osStat = osMutexAcquire(ethTxConfigMutex, timeout);
    assert_param(osStat == osOK);
    TxConfig.TxBuffer = &appBuff->AppBuff;
    halStat = HAL_ETH_Transmit(&heth, &TxConfig, timeout);
    assert_param(halStat == HAL_OK);
    halStat = HAL_ETH_ReleaseTxPacket(&heth);
    assert_param(halStat == HAL_OK);

#ifdef DEBUG_MESSAGE
    // uint32_t len = appBuff->AppBuff.len;
    // uint8_t* pdata = appBuff->AppBuff.buffer;
    // printf("Transmit:\n");
    // for (uint32_t i = 0; i < len; i++) {
    //     printf("%02x ", *pdata++);
    // }
    // printf("\n");
#endif // DEBUG_MESSAGE

    //osStat = osMemoryPoolFree(txBufferPool, appBuff);
    //assert_param(osStat == osOK);
    //osStat = osMutexRelease(ethTxConfigMutex);
    //assert_param(osStat == osOK);
}

/**
 * @brief 接收数据
 *
 * @param pPacket 用于接收 packet 的指针,将被设置为指向 packet
 * @return int 返回 packet 的长度, 若失败返回 -1
 */
int ethReceive(void **pPacket)
{
    ETH_BufferTypeDef *pBuff;


    *pPacket = pBuff->buffer;
    return pBuff->len;
}

/**
 * @brief 对 ETH Receive 线程发送一个 Receive 请求，若成功，将数据复制到 pBuffer 指向的空间中
 *
 * @param pBuffer 指向接收 buffer 的指针
 * @param timeout 超时时间, 0=none blocking
 * @return int 接收到的数据长度, 若超时或失败返回 -1
 */
int ethReceiveRequest(void* pBuffer, uint32_t timeout) {
    ETH_AppBuff* appBuff = NULL;
    int len = -1;
    //osMessageQueueGet(ethRxQueue, &appBuff, NULL, timeout);
    if (appBuff != NULL) {
        ETH_BufferTypeDef* pBuffDef = &appBuff->AppBuff;
        len = pBuffDef->len;
        if (len > 0) 
            memcpy(pBuffer, pBuffDef->buffer, pBuffDef->len);
        //osMemoryPoolFree(rxBufferPool, appBuff);
    }

    return len;
}

/**
  * @brief  检查PHY链接状态并打印速率
  * @retval uint8_t: 1表示链接正常，0表示链接断开
  */
uint8_t check_phy_link(void)
{
    uint32_t phy_status = 0;
    uint32_t phy_control = 0;
    uint32_t speed = 0;
    uint8_t duplex = 0;

    // 读取PHY状态寄存器
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDR, PHY_BSR, &phy_status) != HAL_OK)
    {
        DEBUG_PRINT("Failed to read PHY status register\r\n");
        return 0;  // 读取失败
    }
		printf("BCR=0x%04X\n", phy_status);
    // 检查链接状态
    if (phy_status & PHY_LINK_STATUS)
    {
        // 读取PHY控制寄存器以获取速率信息
        if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDR, PHY_BCR, &phy_control) == HAL_OK)
        {
            // 检查速率
            if (phy_control & PHY_SPEED_100M)
            {
                speed = 100;
            }
            else
            {
                speed = 10;
            }
			
            // 检查双工模式
            duplex = (phy_control & PHY_BCR_FULLDUPLEX) ? 1:0;
            // 打印速率信息
            DEBUG_PRINT("PHY Link is normal, Speed: %dMbps,%s\r\n", speed,duplex?"Full Duplex":"Half Duplex");
        }
        else
        {
            DEBUG_PRINT("Failed to read PHY control register\r\n");
        }
        return 1;  // 链接正常
    }
    else
    {
        DEBUG_PRINT("PHY link disconnected\r\n");
        return 0;  // 链接断开
    }
}
/**
 * @brief  读取PHY寄存器 (HAL v1.27适配)
 * @param  reg: 寄存器地址
 * @return 寄存器值
 */
static uint16_t read_phy_reg(uint16_t reg)
{
    uint32_t reg_val;
    HAL_StatusTypeDef status;

    /* HAL函数参数: 句柄, PHY地址, 寄存器, 输出值 */
    status = HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDR, reg, &reg_val);
    if (status != HAL_OK) {
        printf("PHY Reading failed: reg=0x%X, status=%d\n", reg, status);
        return 0xFFFF;
    }

    return (uint16_t)reg_val;
}

/**
 * @brief  写入PHY寄存器 (HAL v1.27适配)
 * @param  reg: 寄存器地址
 * @param  val: 要写入的值
 */
static void write_phy_reg(uint16_t reg, uint16_t val)
{
    HAL_StatusTypeDef status;

    /* HAL函数参数: 句柄, PHY地址, 寄存器, 要写入的值 */
    status = HAL_ETH_WritePHYRegister(&heth, ETH_PHY_ADDR, reg, val);
    if (status != HAL_OK) {
        printf("PHY Writing failed: reg=0x%X, val=0x%X, status=%d\n", reg, val, status);
    }
}
/**
 * @brief  初始化DM9161 PHY
 * @return 0: 成功, -1: 失败
 */
int dm9161_phy_init(void)
{
    uint16_t phy_id1, phy_id2;
    uint32_t timeout;

    /* 读取PHY ID (确认DM9162) */
		phy_id1 = read_phy_reg(PHY_IDR1);
		phy_id2 = read_phy_reg(PHY_IDR2);
		uint32_t combined_id = (phy_id1 << 16) | (phy_id2 & 0xFFFF);

		if (combined_id == 0x0181B8A0) {
				printf("DM9162 ID=0x%08X\n", combined_id);

		}

    /* 软件复位PHY */
    write_phy_reg(PHY_BCR, PHY_BCR_RESET);
    timeout = 0;
    while ((read_phy_reg(PHY_BCR) & PHY_BCR_RESET) && (timeout < 1000)) {
        HAL_Delay(1);
        timeout++;
    }
    if (timeout >= 1000) {
        printf("PHYReset timeout!\n");
        return -1;
    }
		write_phy_reg(PHY_BCR, PHY_BCR_FULLDUPLEX | PHY_BCR_SPEED100);
			HAL_Delay(10);
    /* 启用自动协商 */
    write_phy_reg(PHY_BCR, PHY_BCR_AUTONEG | PHY_BCR_RESTARTAN);

    /* 等待自动协商完成 */
    timeout = 0;
    while ((read_phy_reg(PHY_BSR) & PHY_BSR_AUTONEGCOMP) == 0 && (timeout < 2000)) {
        HAL_Delay(1);
        timeout++;
    }
    if (timeout >= 2000) {
        printf("Automatic negotiation timeout!\n");
    }
	
    return 0;
}


/**
 * @brief  接收完成回调函数 (HAL内部调用)
 */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
		//printf("[ETH RX] Frame received callback triggered\n");
    /* 通知SOEM有数据到达 */
    //eth_rx_ready = 1;
}
#define ETH_TX_DESC_LEN       0x1FFF 
/**
 * @brief  发送以太网帧
 * @param  buffer: 发送缓冲区
 * @param  len: 发送长度
 * @return 实际发送长度, 0: 发送失败
 */
int ecx_nic_send(void *port, uint8_t *buffer, int len)
{
    if (len <= 0 || len > ETH_TX_BUF_SIZE)
        return 0;

    static uint8_t tx_buffer[ETH_TX_BUF_SIZE] __attribute__((aligned(32)));  // DMA 对齐
    memcpy(tx_buffer, buffer, len);

    ETH_TxPacketConfig txConfig;
    memset(&txConfig, 0, sizeof(txConfig));

    txConfig.Length = len;
    txConfig.TxBuffer = tx_buffer;
    txConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM;  // 可设为 0（无特性）
    txConfig.ChecksumCtrl = ETH_CHECKSUM_DISABLE;
    txConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

    if (HAL_ETH_Transmit(&heth, &txConfig, len) == HAL_OK)
        return 1;
    else
        return 0;
}




/**
 * @brief  接收以太网帧
 * @param  buffer: 接收缓冲区
 * @param  maxlen: 最大接收长度
 * @return 实际接收长度, 0: 无数据, -1: 错误
 */
int ecx_nic_receive(void *port, void *buf, int bufsize)
{
    // 当前描述符索引
    uint32_t cur_idx = heth.RxDescList.RxDescIdx;
    ETH_DMADescTypeDef *rx_desc = &DMARxDscrTab[cur_idx];
//		if (bufsize > 0) {
//				printf("RecvPkt (%d bytes): ", bufsize);
//				for (int i = 0; i < bufsize && i < 64; i++) {
//						printf("%02X ", ((uint8_t *)buf)[i]);
//				}
//				printf("\n");
//		}
    // 检查 OWN 位，DMA 是否完成接收
    if ((rx_desc->DESC0 & (1U << 31)) == 0)  // OWN = 0, CPU可读
    {
        // 检查是否有错误（ES 位 = Error Summary）
        if ((rx_desc->DESC0 & (1 << 15)) == 0)
        {
            // 从 DESC0 的高16位中提取帧长度
            uint32_t frame_len = (rx_desc->DESC0 >> 16) & 0x3FFF;

            if (frame_len > bufsize)
                frame_len = bufsize;

            // 读取数据，DESC2 为缓冲区地址
            memcpy(buf, (void *)(rx_desc->DESC2), frame_len);

            // 重新将描述符交还给 DMA（设置 OWN 位）
            rx_desc->DESC0 |= (1U << 31);

            // 更新下一个描述符索引
            heth.RxDescList.RxDescIdx = (cur_idx + 1) % ETH_RX_DESC_CNT;

            return frame_len;
        }
        else
        {
            // 有错误，丢弃帧，仍然要交还给 DMA
            rx_desc->DESC0 |= (1U << 31);
            heth.RxDescList.RxDescIdx = (cur_idx + 1) % ETH_RX_DESC_CNT;
            return -1;
        }
    }

    // 没有新数据
    return 0;
}


/* USER CODE END 1 */
