/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    eth.h
 * @brief   This file contains all the function prototypes for
 *          the eth.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ETH_H__
#define __ETH_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#define ETH_PHY_ADDR   0x01     // ��� PHY ��ַ��ȡ����Ӳ�����ã�
#define ETH_MACFFR_PR  ((uint32_t)0x00000001U) 
#define PHY_LINK_STATUS (1 << 2) 
#define DM9162_RST  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define DM9162_SET  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)

/* BSR 状态位（实际状态判断） */
#define PHY_BSR_LINKSTATUS    (1 << 2)    // 链路状态（Bit2）
#define PHY_BSR_100MBPS       (1 << 13)   // 实际100Mbps（Bit13）
#define PHY_BSR_FULLDUPLEX    (1 << 14)   // 实际全双工（Bit14）
#define PHY_BSR_AUTONEGCOMP   (1 << 5)    // 自动协商完成（Bit5）

/* BCR 控制位（配置参数） */
#define PHY_BCR_AUTONEG       (1 << 12)   // 自动协商使能（Bit12）
#define PHY_BCR_SPEED100      (1 << 13)   // 期望100Mbps（Bit13）
#define PHY_BCR_FULLDUPLEX    (1 << 8)    // 期望全双工（Bit8）


/* 接收描述符状态寄存器中的错误标志位 */
#define ETH_DMARXDESC_OVR    ((uint32_t)0x00020000)  /* 溢出错误 */
#define ETH_DMARXDESC_FAE    ((uint32_t)0x00040000)  /* 帧对齐错误 */
/* USER CODE END Includes */

extern ETH_HandleTypeDef heth;
extern uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUF_SIZE];
/* USER CODE BEGIN Private defines */

extern ETH_TxPacketConfigTypeDef TxConfig;

typedef struct {
  ETH_BufferTypeDef AppBuff;
  uint8_t buffer[ETH_RX_BUF_SIZE] __ALIGNED(32);
} ETH_AppBuff;

typedef struct {
  ETH_BufferTypeDef* head;
  ETH_BufferTypeDef* tail;
  unsigned int len;
} RxBufferList;

typedef struct {
  uint8_t dest_mac[6];
  uint8_t src_mac[6];
  uint8_t type[2];
  uint8_t payload[100];
} ethernet_frame_t;

// PHY寄存器定义
#define PHY_SPEED_100M 0x2000  // 100Mbps速率位
#define PHY_DUPLEX_FULL 0x0100 // 全双工模式位
// PHY 寄存器 - 标识符寄存器（IDR）
#define PHY_IDR1                 0x02  // PHY 标识符寄存器1（高16位标识符）
#define PHY_IDR2                 0x03  // PHY 标识符寄存器2（低16位标识符）

// PHY 寄存器 - 基本控制寄存器（BCR）字段
#define PHY_BCR_RESET            (1 << 15)  // 复位控制位（1=复位PHY，0=正常工作）
#define PHY_BCR_AUTONEG          (1 << 12)  // 自动协商使能位（1=使能自动协商，0=禁用）
#define PHY_BCR_RESTARTAN        (1 << 9)   // 重启自动协商位（1=触发重启，自动清0）

// PHY 寄存器 - 基本状态寄存器（BSR）字段
#define PHY_BSR_AUTONEGCOMP      (1 << 5)   // 自动协商完成状态（1=完成，0=未完成）
#define PHY_BSR_LINKSTATUS       (1 << 2)   // 链路状态（1=链路_up，0=链路_down）
#define PHY_BSR_100MBPS          (1 << 1)   // 速率状态（1=100Mbps，0=10Mbps，需结合其他位）
#define PHY_BSR_FULLDUPLEX       (1 << 0)   // 双工模式状态（1=全双工，0=半双工）

// 以太网描述符（ETH Descriptor）字段
#define ETH_DESC_OWN             (1 << 31)  // 描述符所有权位（1=DMA拥有，0=CPU拥有）
#define ETH_RX_DESC_LEN          0x0000FFFF // 接收描述符长度掩码（低16位表示接收数据长度）
#define ETH_TX_DESC_LAST         (1 << 29)  // 发送描述符结束位（1=最后一个描述符）
#define ETH_TX_DESC_TERMINATE    (1 << 28)  // 发送终止位（1=异常终止发送，0=正常发送）
/* USER CODE END Private defines */

void MX_ETH_Init(void);

/* USER CODE BEGIN Prototypes */

void ETH_StartLink(void);
void ETH_ConstructEthernetFrame(ethernet_frame_t* frame,
                                uint8_t* dest_mac,
                                uint8_t* src_mac,
                                uint8_t* type,
                                uint8_t* payload,
                                uint16_t payload_len);
int ethSend(void* pBuff, int len);
int ethSendRequest(void *pBuff, uint32_t len, uint32_t timeout);
void ethSendProcess(ETH_AppBuff* appBuff, uint32_t timeout);
int ethReceive(void** pPacket);
int ethReceiveRequest(void* pBuffer, uint32_t timeout);

int dm9162_link_up(void);
uint8_t check_phy_link(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ETH_H__ */

