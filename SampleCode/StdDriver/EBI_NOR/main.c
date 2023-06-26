/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/05/22 2:26p $
 * @brief    Configure EBI interface to access W39L040P (NOR Flash) on EBI interface.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"
#include "ebi_nor.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000


/*---------------------------------------------------------------------------------------------------------*/
/* Program Continue Data to NOR Flash                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t ProgramContinueDataTest(void)
{
    uint8_t u8DataIn, u8DataOut;
    uint32_t u32NORAddr;

    printf("    >> Start to program flash ... \n");

    /* Write */
    for(u32NORAddr = 0; u32NORAddr < EBI_MAX_SIZE; u32NORAddr++)
    {
        u8DataIn = (u32NORAddr % 256) + u32NORAddr;
        if(NOR_WriteData(u32NORAddr, u8DataIn) == FALSE)
        {
            printf("Program [0x%05X]:[0x%02X] FAIL !!! \n\n", u32NORAddr, u8DataIn);
            return FALSE;
        }
    }

    /* Read */
    for(u32NORAddr = 0; u32NORAddr < EBI_MAX_SIZE; u32NORAddr++)
    {
        u8DataIn = (u32NORAddr % 256) + u32NORAddr;
        u8DataOut = NOR_ReadData(u32NORAddr);
        if(u8DataOut != u8DataIn)
        {
            printf("Read [0x%05X]:[0x%02X] FAIL !!! (Got [0x%02X]) \n\n", u32NORAddr, u8DataIn, u8DataOut);
            printf("Program flash FAIL !!! \n\n");
            return FALSE;
        }
    }
    printf("    >> Continue Data Program OK !!! \n\n");

    return TRUE;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable IRC22M clock */
    CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_HIRC;

    /* Set PLL to Power-down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external 12MHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk);

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
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |=  (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);

    /* Set P3 multi-function pins for EBI MCLK, nWR and nRD */
    SYS->P3_MFP &= ~(SYS_MFP_P33_Msk | SYS_MFP_P36_Msk | SYS_MFP_P37_Msk);
    SYS->P3_MFP |=  (SYS_MFP_P33_MCLK | SYS_MFP_P36_nWR | SYS_MFP_P37_nRD);

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
    SYS->P1_MFP |=  (SYS_MFP_P10_nWRL | SYS_MFP_P11_nWRH);

    /* Set P4 multi-function pins for EBI nCS, ALE */
    SYS->P4_MFP &= ~(SYS_MFP_P44_Msk | SYS_MFP_P45_Msk);
    SYS->P4_MFP |=  (SYS_MFP_P44_nCS | SYS_MFP_P45_ALE);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32i;
    uint32_t u32NORIDInfo;
    uint8_t u8ReadOutData;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+---------------------------------+\n");
    printf("|    EBI NOR Flash Sample Code    |\n");
    printf("+---------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect W39L040P to M051 Series EBI bus before EBI testing !! *\n");
    printf("************************************************************************\n\n");

    /* Enable EBI function and bus width to 8-bit */
    EBI_Open(0, EBI_BUSWIDTH_8BIT, EBI_TIMING_NORMAL, 0, 0);

    /* Initial NOR flash and check ID */
    NOR_Init();
    u32NORIDInfo = NOR_GetID();
    if(u32NORIDInfo == 0xDAB6)
    {
        printf("NOR W39L040P initial OK ! Manufacture ID:0x%X, Device ID:0x%X.\n", (u32NORIDInfo >> 8), (u32NORIDInfo & 0xFF));
    }
    else
    {
        printf("NOR W39L040P initial fail ! (ID:0x%X)\n\n", u32NORIDInfo);
        goto lexit;
    }

    /* Erase flash */
    NOR_Erase();
    for(u32i = 0; u32i < EBI_MAX_SIZE; u32i++)
    {
        u8ReadOutData = NOR_ReadData(u32i);
        if(u8ReadOutData != 0xFF)
        {
            printf("    >> Chip Erase Fail !! Addr:0x%X, Data:0x%X.\n\n", u32i, u8ReadOutData);
            goto lexit;
        }
    }
    printf("    >> Chip Erase OK !!!\n");

    /* Start to program NOR flash test */
    if( ProgramContinueDataTest() == TRUE )
    {
        printf("*** NOR Flash Test OK ***\n");
    }

lexit:

    /* Disable EBI function */
    EBI_Close(0);

    /* Disable EBI clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_EBI_EN_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
