/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/05/22 10:24a $
 * @brief    M051 Series Flash Memory Controller Driver Sample Code
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000

#if !defined(__ICCARM__) && !defined(__GNUC__)
extern uint32_t Image$$RO$$Base;
#endif

typedef void (FUNC_PTR)(void);
int32_t g_FMC_i32ErrCode;

void SYS_Init(void)
{
    int32_t i;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    i = 22000000; // For timeout
    while(i-- > 0)
    {
        if((CLK->CLKSTATUS & (CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk)) ==
                (CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk))
            break;
    }

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD  */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PLL_CLOCK, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    volatile uint32_t u32BootAddr;
    FUNC_PTR    *ResetFunc;

    uint8_t ch;
    uint32_t u32Data;
    uint32_t u32Cfg;

    /* Unlock protected registers for ISP function */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Enable ISP function */
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;

    /*
        This sample code shows how to boot with different firmware images in APROM by software reset method.
        In the code, VECMAP and functional pointer are used to implement multi-boot function.
        Software set VECMAP to remap page of VECMAP to 0x0~0x1ff then set the main stack and jump to new boot.
        NOTE: VECMAP only valid when CBS = 00'b or 10'b.

        To use this sample code, please:
        1. Build all targets and download to device individually. The targets are:
            FMC_MultiBoot, RO=0x0
            FMC_Boot0, RO=0x2000
            FMC_Boot1, RO=0x4000
            FMC_Boot2, RO=0x6000
            FMC_Boot3, RO=0x8000
        2. Reset MCU to execute FMC_MultiBoot.

    */

    printf("\n\n");
    printf("+--------------------------------------------------------+\n");
    printf("|    M05xx Multi-Boot without Chip Reset Sample Code     |\n");
    printf("+--------------------------------------------------------+\n");

    printf("\nCPU @ %dHz\n\n", SystemCoreClock);

#if defined(__BASE__)
    printf("Boot from 0\n");
#endif
#if defined(__BOOT0__)
    printf("Boot from 0x2000\n");
#endif
#if defined(__BOOT1__)
    printf("Boot from 0x4000\n");
#endif
#if defined(__BOOT2__)
    printf("Boot from 0x6000\n");
#endif
#if defined(__BOOT3__)
    printf("Boot from 0x8000\n");
#endif

#if defined(__ICCARM__) || defined(__GNUC__)
    printf("VECMAP = 0x%x\n", FMC_GetVECMAP());
#else
    printf("Current RO Base = 0x%x, VECMAP = 0x%x\n", (uint32_t)&Image$$RO$$Base, FMC_GetVECMAP());
#endif

    /* Check IAP mode */
    u32Cfg = FMC_Read(FMC_CONFIG_BASE);
    if((u32Cfg & 0xc0) != 0x80)
    {
        printf("Do you want to set to new IAP mode (APROM boot + LDROM)?\n");
        if(getchar() == 'y')
        {
            FMC->ISPCON |= FMC_ISPCON_CFGUEN_Msk; /* Enable user configuration update */

            /* Set CBS to b'10 */
            u32Cfg &= ~0xc0ul;
            u32Cfg |= 0x80;
            u32Data = FMC_Read(FMC_CONFIG_BASE + 0x4); /* Backup the data of config1 */
            FMC_Erase(FMC_CONFIG_BASE);
            FMC_Write(FMC_CONFIG_BASE, u32Cfg);
            FMC_Write(FMC_CONFIG_BASE + 0x4, u32Data);

            printf("Press any key to reset system to enable new IAP mode ...\n");
            getchar();
            SYS->IPRSTC1 = 0x1; /* Reset MCU */
            while(1);
        }
        else
        {
            printf("VECMAP only valid in new IAP mode. CBS = 10'b or 00'b\n");
            goto lexit;
        }
    }

    printf("Select one boot image: \n");
    printf("[0] Boot 0, base = 0x2000\n");
    printf("[1] Boot 1, base = 0x4000\n");
    printf("[2] Boot 2, base = 0x6000\n");
    printf("[3] Boot 3, base = 0x8000\n");
    printf("[Others] Boot, base = 0x0\n");

    ch = getchar();
    switch(ch)
    {
        case '0':
            u32BootAddr = 0x2000;
            break;
        case '1':
            u32BootAddr = 0x4000;
            break;
        case '2':
            u32BootAddr = 0x6000;
            break;
        case '3':
            u32BootAddr = 0x8000;
            break;
        default:
            u32BootAddr = 0x0000;
            break;
    }

    /* Disable all interrupts before change VECMAP */
    NVIC->ICER[0] = 0xFFFFFFFF;

    /* Set vector table of startup AP address */
    FMC_SetVectorPageAddr(u32BootAddr);
    if(FMC_GetVECMAP() != u32BootAddr)
    {
        printf("\nERROR: VECMAP isn't supported in current chip.\n");
        goto lexit;
    }

    /* Reset All IP before boot to new AP */
    SYS->IPRSTC2 = 0xFFFFFFFF;
    SYS->IPRSTC2 = 0;

    /* Obtain Reset Handler address of new boot. */
    ResetFunc = (FUNC_PTR *)M32(4);

#if defined(__GNUC__)
    /* Set Main Stack Pointer register of new boot */
    //__set_MSP(M32(FMC_Read(u32BootAddr)));
    __set_MSP(0x20001000);
#else
    /* Set Main Stack Pointer register of new boot */
    __set_MSP(M32(0));
#endif

    /* Call reset handler of new boot */
    ResetFunc();

    while(1);

lexit:

    /* Disable ISP function */
    FMC->ISPCON &= ~FMC_ISPCON_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nDone\n");
    while(SYS->PDID) __WFI();
}
