/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 1 $
 * $Date: 16/06/21 1:35p $
 * @brief    Measure AVDD voltage by ADC.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"

#define PLL_CLOCK       50000000

#define VBG_VOLTAGE (1250) /* 1.25V = 1250 mV (Typical band-gap voltage of M051XxDE series) */
//#define VBG_VOLTAGE (1200) /* 1.20V = 1200 mV (Typical band-gap voltage of M051XxDN series) */
#define ADC_SAMPLE_COUNT 128 /* The last line of GetAVDDCodeByADC() need revise when ADC_SAMPLE_COUNT is changed. */
/* For example, if ADC_SAMPLE_COUNT is changed to 64, then the code need revised to "return (u32Sum >> 6);" */

/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void AdcMeasureAVDD(void);
uint32_t GetAVDDCodeByADC(void);
uint32_t GetAVDDVoltage(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8ADF;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

    /* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(7));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ADC interrupt handler                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void ADC_IRQHandler(void)
{
    uint32_t u32Flag;

    /* Get ADC conversion finish interrupt flag */
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_ADF_INT);

    /* Check ADC conversion finish */
    if(u32Flag & ADC_ADF_INT)
        g_u8ADF = 1;

    /* Clear conversion finish flag */
    ADC_CLR_INT_FLAG(ADC, u32Flag);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: GetAVDDVoltage                                                                                */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   AVDD voltage(mV).                                                                                     */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Use Band-gap voltage to calculate AVDD voltage                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetAVDDVoltage(void)
{
    uint32_t  u32ConversionResult;
    uint64_t u64MvAVDD;

    /* Calculate Vref by using conversion result of VBG */
    u32ConversionResult = GetAVDDCodeByADC();

    /* u32ConversionResult = VBG * 4096 / Vref, Vref = AVDD */
    /* => AVDD = VBG * 4096 / u32ConversionResult */
    u64MvAVDD = (VBG_VOLTAGE << 12) / (uint64_t)u32ConversionResult;

    printf("Conversion result: 0x%X\n", u32ConversionResult);

    return (uint32_t)u64MvAVDD;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: GetAVDDCodeByADC                                                                              */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   ADC code of AVDD voltage.                                                                             */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Get ADC conversion result of Band-gap voltage.                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetAVDDCodeByADC(void)
{
    uint32_t u32Count, u32Sum, u32Data, u32TimeOutCnt;

    /* Power on ADC */
    ADC_POWER_ON(ADC);

    /* Configure ADC: single-end input, single scan mode, enable ADC analog circuit. */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, BIT7);
    /* Configure the analog input source of channel 7 as internal band-gap voltage */
    ADC_CONFIG_CH7(ADC, ADC_ADCHER_PRESEL_INT_BANDGAP);

    /* Clear conversion finish flag */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Enable ADC conversion finish interrupt */
    ADC_EnableInt(ADC, ADC_ADF_INT);
    NVIC_EnableIRQ(ADC_IRQn);

    g_u8ADF = 0;
    u32Sum = 0;

    /* sample times are according to ADC_SAMPLE_COUNT definition */
    for(u32Count = 0; u32Count < ADC_SAMPLE_COUNT; u32Count++)
    {
        /* Delay for band-gap voltage stability */
        CLK_SysTickDelay(100);

        /* Start A/D conversion */
        ADC_START_CONV(ADC);

        u32Data = 0;

        /* Wait conversion done */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(g_u8ADF == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for ADC conversion done time-out!\n");
                return 0;
            }
        }
        g_u8ADF = 0;

        /* Get the conversion result */
        u32Data = ADC_GET_CONVERSION_DATA(ADC, 7);
        /* Sum each conversion data */
        u32Sum += u32Data;
    }
    /* Disable ADC interrupt */
    ADC_DisableInt(ADC, ADC_ADF_INT);
    /* Disable ADC */
    ADC_POWER_DOWN(ADC);

    /* Return the average of ADC_SAMPLE_COUNT samples */
    return (u32Sum >> 7);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    uint32_t u32AVDDVoltage;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC for AVDD Measurement sample code                 |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this sample code, software will get voltage value from AVDD.\n");

    /*------------------------------------------------------------------------------------------------------------------
       The method of measured AVDD voltage is using ADC to get conversion result of band-gap voltage.

       For example, the typical value of band-gap voltage is 1.25 V, and Vref of ADC is from AVDD.
       Through getting ADC conversion result of band-gap voltage, then AVDD voltage can be calculated by below formula:

           ConversionResult = VBG * 4096 / Vref, Vref = AVDD and VBG = 1.25V
           => AVDD = 1.25V * 4096 / ConversionResult


       Note 1 : The measured AVDD has deviation that causes by the band-gap voltage has deviation in different temperature, power voltage and ADC conversion deviation.(4 LSB)
                The deviation of measured AVDD is list as follows:

                (1) M051XxDN series:
                    The Spec. of band-gap voltage in M051XxDN is as follows:
                    -----------------------------------------------------------------------------------------
                    |                  | Min.   | Typ.   | Max.   |                                         |
                    |                  |--------------------------- VDD = 2.5 V ~ 5.5 V                     |
                    | band-gap voltage | 1.14 V | 1.20 V | 1.26 V | temperature = -40 ~ 105 degrees Celsius |
                    |                  |        |        |        |                                         |
                    -----------------------------------------------------------------------------------------

                    Deviation range of measured AVDD
                    ----------------------------------------------------
                    |                | Min. Deviation | Max. Deviation |
                    |                |                |                |
                    |                | VBG = 1.14 V   | VBG = 1.26 V   |
                    |--------------------------------------------------|
                    |  AVDD = 2.5 V  |   -5.71%       |    5.80%       |
                    |--------------------------------------------------|
                    |  AVDD = 5.5 V  |   -6.56%       |    6.79%       |
                    ----------------------------------------------------

                (2) M051XxDE series:
                    The Spec. of band-gap voltage in M051XxDE is as follows:
                    -----------------------------------------------------------------------------------------
                    |                  | Min.   | Typ.   | Max.   |                                         |
                    |                  |--------------------------- VDD = 2.5 V ~ 5.5 V                     |
                    | band-gap voltage | 1.18 V | 1.25 V | 1.32 V | temperature = -40 ~ 105 degrees Celsius |
                    |                  |        |        |        |                                         |
                    -----------------------------------------------------------------------------------------

                    Deviation range of measured AVDD
                    ----------------------------------------------------
                    |                | Min. Deviation | Max. Deviation |
                    |                |                |                |
                    |                | VBG = 1.18 V   | VBG = 1.32 V   |
                    |--------------------------------------------------|
                    |  AVDD = 2.5 V  |   -6.28%       |    6.37%       |
                    |--------------------------------------------------|
                    |  AVDD = 5.5 V  |   -7.09%       |    7.32%       |
                    ----------------------------------------------------

       Note 2: The typical value of band-gap voltage in M051XxDN series is 1.20V, and it is 1.25V in M051XxDE series.
               In this sample code is using the typical value of M051XxDE series: 1.25 V, and it can be modified by VBG_VOLTAGE definition.

    ------------------------------------------------------------------------------------------------------------------*/
    /* Measure AVDD */
    u32AVDDVoltage = GetAVDDVoltage();
    printf("AVDD Voltage: %dmV\n", u32AVDDVoltage);

    /* Disable ADC module */
    ADC_Close(ADC);

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("\nExit ADC sample code\n");

    while(1);

}

