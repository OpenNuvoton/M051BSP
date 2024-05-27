/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/05/22 2:34p $
 * @brief    Use timer-0 toggle-output interrupt event to wake-up system. 
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);
volatile uint8_t g_u8IsTMR0WakeupFlag = 0;
volatile uint32_t g_au32TMRINTCount[4] = {0};


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power-down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    printf("\nSystem enter to power-down mode ...\n");

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(IsDebugFifoEmpty() == 0)
        if( --u32TimeOutCnt == 0 ) break;

    SCB->SCR = 4;

    /* To program PWRCON register, it needs to disable register protection first. */
    CLK->PWRCON = (CLK->PWRCON & ~(CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WAIT_CPU_Msk)) |
                  CLK_PWRCON_PD_WAIT_CPU_Msk | CLK_PWRCON_PD_WU_INT_EN_Msk;
    CLK->PWRCON |= CLK_PWRCON_PWR_DOWN_EN_Msk;

    __WFI();
}

/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_M051Series.s.
 */
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        g_au32TMRINTCount[0]++;
    }

    if(TIMER_GetWakeupFlag(TIMER0) == 1)
    {
        /* Clear Timer0 wake-up flag */
        TIMER_ClearWakeupFlag(TIMER0);

        g_u8IsTMR0WakeupFlag = 1;
    }
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

    /* Enable external 12 MHz XTAL, IRC10K */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_IRC10K_STB_Msk);

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_HCLK_DIV2 | CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable peripheral clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_TMR0_EN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL | CLK_CLKSEL1_TMR0_S_LIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD, TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |=  (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);

    /* Set P3 multi-function pins for T0 */
    SYS->P3_MFP &= ~SYS_MFP_P34_Msk;
    SYS->P3_MFP |= SYS_MFP_P34_T0;
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
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t u32InitCount;
    uint32_t u32TimeOutCnt;

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
    printf("+-------------------------------------------------+\n");
    printf("|    Timer0 Power-down and Wake-up Sample Code    |\n");
    printf("+-------------------------------------------------+\n\n");

    printf("# Timer Settings:\n");
    printf("  Timer0: Clock source is LIRC(10 kHz); Toggle-output mode and frequency is 2 Hz; Enable interrupt and wake-up.\n");
    printf("# System will enter to Power-down mode while Timer0 interrupt count is reaching 3;\n");
    printf("  And be waken-up while Timer0 interrupt count is reaching 4.\n\n");

    /* To program PWRCON register, it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Open Timer0 frequency to 2 Hz in toggle-output mode */
    TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, 1);

    /* Enable Timer0 interrupt and wake-up function */
    TIMER_EnableInt(TIMER0);
    TIMER_EnableWakeup(TIMER0);

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    u32InitCount = g_u8IsTMR0WakeupFlag = g_au32TMRINTCount[0] = 0;
    while(g_au32TMRINTCount[0] < 10)
    {
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
            printf("Timer0 interrupt count - %d\n", g_au32TMRINTCount[0]);
            if(g_au32TMRINTCount[0] == 3)
            {
                PowerDownFunction();

                /* Check if Timer0 time-out interrupt and wake-up flag occurred */
                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
                while(1)
                {
                    if(((CLK->PWRCON & CLK_PWRCON_PD_WU_STS_Msk) == CLK_PWRCON_PD_WU_STS_Msk) && (g_u8IsTMR0WakeupFlag == 1))
                        break;

                    if(--u32TimeOutCnt == 0)
                    {
                        printf("Wait for System or Timer interrupt time-out!\n");
                        break;
                    }
                }
                printf("System has been waken-up done. (Timer0 interrupt count is %d)\n\n", g_au32TMRINTCount[0]);
            }
            u32InitCount = g_au32TMRINTCount[0];
        }
    }

    /* Stop Timer0 counting */
    TIMER_Close(TIMER0);

    printf("*** PASS ***\n");

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
