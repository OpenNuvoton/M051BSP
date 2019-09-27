/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to update chip flash data through RS485 interface
 *           between chip RS485 and ISP Tool.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip RS485 and assign update file
 *           of Flash.
 * 
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"
#include "ISP_USER.h"
#include "uart_transfer.h"

#define PLLCON_SETTING  CLK_PLLCON_50MHz_HIRC
#define PLL_CLOCK       50000000

#define nRTSPin                 (P01)
#define REVEIVE_MODE            (0)
#define TRANSMIT_MODE           (1)

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    
    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= (CLK_PWRCON_OSC22M_EN_Msk | CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while (!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(1);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()
    
    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART1_EN_Msk;
    
    /* Select UART module clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART_S_Msk)) | CLK_CLKSEL1_UART_S_HIRC;
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P0 pin for UART1 nRTS */   
    P0->PMD = (P0->PMD & (~GPIO_PMD_PMD1_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD1_Pos);
    nRTSPin = REVEIVE_MODE;    
    
    /* Set P1 multi-function pins for UART1 RXD and TXD */    
    SYS->P1_MFP &= ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk);
    SYS->P1_MFP |= (SYS_MFP_P12_RXD1 | SYS_MFP_P13_TXD1);
    
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
    
    /* Init UART to 115200-8n1 */    
    UART_Init();
    
    /* Enable FMC ISP */    
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;
    
    /* Get APROM size, data flash size and address */    
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    
    /* Set Systick time-out for 300ms */     
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;   /* Use CPU clock */

    /* Wait for CMD_CONNECT command until Systick time-out */
    while (1) {
        
        /* Wait for CMD_CONNECT command */         
        if ((bufhead >= 4) || (bUartDataReady == TRUE)) {
            uint32_t lcmd;
            lcmd = inpw(uart_rcvbuf);

            if (lcmd == CMD_CONNECT) {
                goto _ISP;
            } else {
                bUartDataReady = FALSE;
                bufhead = 0;
            }
        }

        /* Systick time-out, then go to APROM */        
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            goto _APROM;
        }
    }

_ISP:

    /* Prase command from master and send response back */      
    while (1) {
        if (bUartDataReady == TRUE) {
            
            WDT->WTCR &= ~(WDT_WTCR_WTE_Msk | WDT_WTCR_DBGACK_WDT_Msk);
            WDT->WTCR |= (WDT_TIMEOUT_2POW18 | WDT_WTCR_WTR_Msk);
            
            bUartDataReady = FALSE;;        /* Reset UART data ready flag */     
            ParseCmd(uart_rcvbuf, 64);      /* Parse command from master */  
            NVIC_DisableIRQ(UART_T_IRQn);   /* Disable NVIC */
            nRTSPin = TRANSMIT_MODE;        /* Control RTS in transmit mode */
            PutString();                    /* Send response to master */

            /* Wait for data transmission is finished */
            while ((UART_T->FSR & UART_FSR_TE_FLAG_Msk) == 0);  

            nRTSPin = REVEIVE_MODE;         /* Control RTS in reveive mode */
            NVIC_EnableIRQ(UART_T_IRQn);    /* Enable NVIC */            
           
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
