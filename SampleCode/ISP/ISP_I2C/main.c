/******************************************************************************
 * @file     main.c
 * @brief    Demonstrate how to update chip flash data through I2C interface
 *           between chip I2C and ISP Tool.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip I2C and assign update file
 *           of Flash.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "targetdev.h"


#define PLLCON_SETTING  CLK_PLLCON_50MHz_HIRC
#define PLL_CLOCK       50000000

uint32_t u32Pclk0;
uint32_t u32Pclk1;

void ProcessHardFault(void) {}
void SH_Return(void) {}

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk))
        if(--u32TimeOutCnt == 0) return -1;

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(1);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk))
        if(--u32TimeOutCnt == 0) return -1;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable module clock */
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;

    /* Enable I2C1 peripheral clock */
    //CLK_EnableModuleClock(I2C1_MODULE);
    CLK->APBCLK |= CLK_APBCLK_I2C1_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART_S_Msk)) | CLK_CLKSEL1_UART_S_HIRC;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for I2C1 SDA and SCL */
    SYS->P2_MFP &= ~(SYS_MFP_P25_Msk | SYS_MFP_P24_Msk);
    SYS->P2_MFP |= (SYS_MFP_P25_SDA1 | SYS_MFP_P24_SCL1);

    /* I2C clock pin enable schmitt trigger */
    SYS->P2_MFP |= (4<<SYS_P2_MFP_P2_TYPE_Pos);

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t au8CmdBuff[16];

    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Configure WDT */
    WDT->WTCR &= ~(WDT_WTCR_WTE_Msk | WDT_WTCR_DBGACK_WDT_Msk);
    WDT->WTCR |= (WDT_TIMEOUT_2POW18 | WDT_WTCR_WTR_Msk);

    /* Init system and multi-funcition I/O */
    if( SYS_Init() < 0 ) goto _APROM;

    /* Enable FMC ISP */
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;

    /* Get APROM size, data flash size and address */
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    /* Init I2C */
    I2C_Init();

    /* Set Systick time-out as 300ms */
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Wait for CMD_CONNECT command until Systick time-out */
    while (1)
    {
        /* Wait for CMD_CONNECT command */
        if (u8I2cDataReady == 1)
        {
            goto _ISP;
        }
        /* Systick time-out, go to APROM */
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    /* Parse command from master and send response back */
    while (1)
    {
        if (u8I2cDataReady == 1)
        {
            /* Get command from I2C receive buffer */
            memcpy(au8CmdBuff, au8I2cRcvBuf, 64);
            u8I2cDataReady = 0;
            /* Parse the current command */
            ParseCmd((unsigned char *)au8CmdBuff, 64);
        }
    }

_APROM:

    /* Reset system and boot from APROM */
    SYS->RSTSRC = (SYS_RSTSRC_RSTS_POR_Msk | SYS_RSTSRC_RSTS_RESET_Msk); /* Clear reset status flag */
    FMC->ISPCON &= ~(FMC_ISPCON_BS_Msk|FMC_ISPCON_ISPEN_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
