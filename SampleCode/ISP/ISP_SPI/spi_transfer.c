/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool SPI initialization and IRQ function
 * @version  2.0.0
 *
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define TEST_COUNT 16

uint32_t spi_rcvbuf[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

volatile uint8_t bSpiDataReady = 0;

void SPI_Init(void)
{
    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI1 as a low level active device. */
    /* Default setting: slave selection signal is low level active. */
    SPI1->SSR = SPI_SS_ACTIVE_LOW | SPI_SSR_SS_LTRIG_Msk;
    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
    SPI1->CNTRL = SPI_SLAVE | ((32 & 0x1F) << SPI_CNTRL_TX_BIT_LEN_Pos) | (SPI_MODE_0) | SPI_CNTRL_FIFO_Msk;
    /* Set DIVIDER = 0 */
    SPI1->DIVIDER = 0UL;
    /* Set TX FIFO threshold and enable FIFO mode. */
    SPI1->FIFO_CTL = (SPI1->FIFO_CTL & ~(SPI_FIFO_CTL_TX_THRESHOLD_Msk | SPI_FIFO_CTL_RX_THRESHOLD_Msk)) |
                    (4 << SPI_FIFO_CTL_TX_THRESHOLD_Pos) |
                    (4 << SPI_FIFO_CTL_RX_THRESHOLD_Pos);
    SPI_WRITE_TX0(SPI1, 0xFFFFFFFF);    /* Dummy Write to prevent TX under run */
}

void GPIO_Init(void)
{
    /* Enable P0.4 interrupt by falling edge trigger */
    P0->IMD |= (GPIO_IMD_EDGE << 4);
    P0->IEN |= (BIT4 << GPIO_IEN_IF_EN_Pos);
    NVIC_EnableIRQ(GPIO_P0P1_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  GPIOP0P1 IRQ Handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void GPIOP0P1_IRQHandler(void)
{
    uint32_t *_response_buff;
    _response_buff = (uint32_t *)response_buff; // in isp_user.c

    if(GPIO_GET_INT_FLAG(P0, BIT4))
    {
        GPIO_CLR_INT_FLAG(P0, BIT4);
        SPI1->FIFO_CTL |= (SPI_FIFO_CTL_RX_CLR_Msk | SPI_FIFO_CTL_TX_CLR_Msk);
        g_u32TxDataCount = 0;
        g_u32RxDataCount = 0;

        // Active
        while(P04 == 0)
        {
            /* Check TX FULL flag and TX data count */
            if((SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0) && (g_u32TxDataCount < TEST_COUNT))
            {
                SPI_WRITE_TX0(SPI1, _response_buff[g_u32TxDataCount]);    /* Write to TX FIFO */
                g_u32TxDataCount++;
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                SysTick->LOAD = 1000 * CyclesPerUs;
                SysTick->VAL   = (0x00);
                SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
            }

            /* Check RX EMPTY flag */
            if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0)
            {
                g_u32RxDataCount &= 0x0F;
                spi_rcvbuf[g_u32RxDataCount++] = SPI_READ_RX0(SPI1);    /* Read RX FIFO */
            }

            if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                break;
            }
        }

        if(P04 == 1)
        {
            if((g_u32RxDataCount == 16) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900))
            {
                bSpiDataReady = 1;
            }

            spi_rcvbuf[0] &= 0x000000FF;
            g_u32TxDataCount = 0;
            g_u32RxDataCount = 0;

            if(SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0)
            {
                SPI_WRITE_TX0(SPI1, 0xFFFFFFFF);    /* Write to TX FIFO */
            }
        }
    }
    else
    {
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
