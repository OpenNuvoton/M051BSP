/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 15/05/22 2:30p $
 * @brief    Configure EBI interface to access BS616LV4017 (SRAM) on EBI interface.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000


extern int32_t SRAM_BS616LV4017(void);


void SYS_Init(void)
{
	uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable IRC22M clock */
    CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC22M_STB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_HIRC;

    /* Set PLL to Power-down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external 12MHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = CLK_PLLCON_50MHz_HXT;

    /* Waiting for clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk))
		if(--u32TimeOutCnt == 0) break;
    u32TimeOutCnt = __HIRC;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_HCLK_DIV2 | CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable EBI clock */
    CLK->AHBCLK |= CLK_AHBCLK_EBI_EN_Msk;

    /* Enable UART clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk;

    /* UART clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    PllClock        = PLL_CLOCK;
    SystemCoreClock = PLL_CLOCK / 1;
    CyclesPerUs     = PLL_CLOCK / 1000000;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);

    /* Set P3 multi-function pins for EBI MCLK, nWR and nRD */
    SYS->P3_MFP &= ~(SYS_MFP_P33_Msk | SYS_MFP_P36_Msk | SYS_MFP_P37_Msk);
    SYS->P3_MFP |= (SYS_MFP_P33_MCLK | SYS_MFP_P36_nWR | SYS_MFP_P37_nRD);

    /* Set P0 multi-function pins for EBI AD0 ~ AD7 */
    SYS->P0_MFP &= ~(SYS_MFP_P00_Msk | SYS_MFP_P01_Msk | SYS_MFP_P02_Msk | SYS_MFP_P03_Msk |
                     SYS_MFP_P04_Msk | SYS_MFP_P05_Msk | SYS_MFP_P06_Msk | SYS_MFP_P07_Msk);
    SYS->P0_MFP |= (SYS_MFP_P00_AD0 | SYS_MFP_P01_AD1 | SYS_MFP_P02_AD2 | SYS_MFP_P03_AD3 |
                    SYS_MFP_P04_AD4 | SYS_MFP_P05_AD5 | SYS_MFP_P06_AD6 | SYS_MFP_P07_AD7);

    /* Set P2 multi-function pins for EBI AD8 ~ AD15 */
    SYS->P2_MFP &= ~(SYS_MFP_P20_Msk | SYS_MFP_P21_Msk | SYS_MFP_P22_Msk | SYS_MFP_P23_Msk |
                     SYS_MFP_P24_Msk | SYS_MFP_P25_Msk | SYS_MFP_P26_Msk | SYS_MFP_P27_Msk);
    SYS->P2_MFP |= (SYS_MFP_P20_AD8 | SYS_MFP_P21_AD9 | SYS_MFP_P22_AD10 | SYS_MFP_P23_AD11 |
                    SYS_MFP_P24_AD12 | SYS_MFP_P25_AD13 | SYS_MFP_P26_AD14 | SYS_MFP_P27_AD15);

    /* Set P1 multi-function pins for EBI nWRL and nWRH */
    SYS->P1_MFP &= ~(SYS_MFP_P10_Msk | SYS_MFP_P11_Msk);
    SYS->P1_MFP |= (SYS_MFP_P10_nWRL | SYS_MFP_P11_nWRH);

    /* Set P4 multi-function pins for EBI nCS, ALE */
    SYS->P4_MFP &= ~(SYS_MFP_P44_Msk | SYS_MFP_P45_Msk);
    SYS->P4_MFP |= (SYS_MFP_P44_nCS | SYS_MFP_P45_ALE);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PllClock, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();
#if !( __GNUC__ )
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
#endif
    printf("+----------------------------+\n");
    printf("|    EBI SRAM Sample Code    |\n");
    printf("+----------------------------+\n\n");

    printf("***************************************************************************\n");
    printf("* Please connect BS616LV4017 to M051 Series EBI bus before EBI testing !! *\n");
    printf("***************************************************************************\n\n");

    /* Enable EBI function and bus width to 16-bit, MCLK is HCLK/2 */
    EBI->EBICON = (EBI_MCLKDIV_2 << EBI_EBICON_MCLKDIV_Pos) | EBI_EBICON_ExtBW16_Msk | EBI_EBICON_ExtEN_Msk |
                  (0x3 << EBI_EBICON_ExttALE_Pos) ;
    EBI->EXTIME = 0x03003318;

    /* Start SRAM test */
    if(SRAM_BS616LV4017() == 0)
    {
        printf("*** SRAM Test OK ***\n");
    }

    /* Disable EBI function */
    EBI->EBICON &= ~EBI_EBICON_ExtEN_Msk;

    /* Disable EBI clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_EBI_EN_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
