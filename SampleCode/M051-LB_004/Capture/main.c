/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2009 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include "M051Series.h"
#include "lcd_driver.h"


#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000


uint32_t g_u32Freq = 0;
uint32_t g_u32DutyCycle = 0;
uint32_t g_u32RisingCnt = 0;
uint32_t g_u32FallingCnt = 0;
char g_strBuf[32] = {0};

void PWMA_IRQHandler(void)
{
    uint32_t u32Reg;

    u32Reg = PWMA->CCR0;

    /* Clear capture interrupt flag */
    PWMA->CCR0 |= PWM_CCR0_CAPIF0_Msk;

    /* Falling edge */
    if(u32Reg & PWM_CCR0_CFLRI0_Msk)
    {
        g_u32FallingCnt = PWMA->CFLR0;
        /* Clear falling event flag */
        PWMA->CCR0 |= PWM_CCR0_CFLRI0_Msk;
    }

    /* Rising edge */
    if(u32Reg & PWM_CCR0_CRLRI0_Msk)
    {
        g_u32RisingCnt = PWMA->CRLR0;
        /* Clear rising event flag */
        PWMA->CCR0 |= PWM_CCR0_CRLRI0_Msk;
    }

    /* Clear PWM0 interrupt flag */
    PWMA->PIIR = PWM_PIIR_PWMIF0_Msk;
}


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to internal RC */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external 12MHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & (CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk)));

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_HCLK_DIV2 | CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_SPI0_EN_Msk |
                  CLK_APBCLK_PWM01_EN_Msk | CLK_APBCLK_ADC_EN_Msk;
    /* IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL | CLK_CLKSEL1_PWM01_S_HCLK;

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
    SYS->P3_MFP |= SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0;
    /* Set P1.4, P1.5, P1.6, P1.7 for SPI0 to driver LCD */
    SYS->P1_MFP &= ~(SYS_MFP_P14_Msk | SYS_MFP_P15_Msk | SYS_MFP_P16_Msk | SYS_MFP_P17_Msk);
    SYS->P1_MFP |= SYS_MFP_P14_SPISS0 | SYS_MFP_P15_MOSI_0 | SYS_MFP_P16_MISO_0 | SYS_MFP_P17_SPICLK0;
    /* Set P2.0 => PWM0, P2.1 => PWM1 */
    SYS->P2_MFP &= ~(SYS_MFP_P20_Msk | SYS_MFP_P21_Msk);
    SYS->P2_MFP |= SYS_MFP_P20_PWM0 | SYS_MFP_P21_PWM1;
}


void UART0_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

}


void PWMA_Init(void)
{
    /* PWM Timer0: clk = HCLK / 2 /  1, Freq = clk / 65536, Max cycles = 65536      */
    /* PWM Timer1: clk = HCLK / 2 / 16, Freq = clk / 1000, duty cycle = 200/1000 % */
    PWM_SET_PRESCALER(PWMA, PWM_CH1, 1);
    PWM_SET_DIVIDER(PWMA, PWM_CH0, PWM_CLK_DIV_1);
    PWM_SET_DIVIDER(PWMA, PWM_CH1, PWM_CLK_DIV_16);

    /* Enable PWM0,1 counter. We must set PWM mode before setting CNR, CMR. */
    PWMA->PCR = PWM_PCR_CH1EN_Msk | PWM_PCR_CH1MOD_Msk |
                PWM_PCR_CH0EN_Msk | PWM_PCR_CH0MOD_Msk;

    /* Enable PWM0 as input capture mode */
    PWMA->CCR0 = PWM_CCR0_CFLRI0_Msk | PWM_CCR0_CRLRI0_Msk | PWM_CCR0_CAPCH0EN_Msk |
                 PWM_CCR0_CFL_IE0_Msk | PWM_CCR0_CRL_IE0_Msk;

    PWMA->CNR0 = 0xffff;
    PWMA->CMR0 = 0;
    PWMA->CNR1 = 1000 - 1;
    PWMA->CMR1 = 200 - 1;

    /* Enable Interrupt */
    PWMA->PIER = PWM_PIER_PWMIE0_Msk;

    NVIC_EnableIRQ(PWMA_IRQn);

    /* Enable PWM1 Output */
    PWMA->POE = PWM_POE_PWM1_Msk;

    /* Need to make sure PWM timer0 ready before enable capture */
    while(PWMA->PDR0 == 0);

    /* Enable PWM0 Capture */
    PWMA->CAPENR = 1;

}

/*----------------------------------------------------------------------------
  MAIN function
  ----------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init SPI0 and LCD */
    LCD_Init();
    LCD_EnableBackLight();
    LCD_ClearScreen();

    LCD_Print(0, "IN:P2.0(400~100kHz)");
    LCD_Print(1, "OUT:P2.1(1500Hz)");

    /* Init PWMA channel 0 to be capture mode and channel 1 to be output mode */
    PWMA_Init();

    /*
        PWM timer 0 is set as input capture function.
        PWM0 will down-count from 0xFFFF. When capture rising and falling event, the count value will
        be recorded by g_u32RisingCnt, g_u32FallingCnt in ISR.
        We can use g_u32RisingCnt and g_u32FallingCnt to calculate input frequency and duty cycle.

        PWM timer 1 is set as PWM output.
    */

    while(1)
    {
        uint32_t u32Tmp1, u32Tmp2;
        char strClearFreqVal[15] = "Freq:          ";
        char strClearDutyVal[15] = "Duty:          ";
        u32Tmp1 = (0xFFFF - g_u32RisingCnt); // low interval
        u32Tmp2 = (0xFFFF - g_u32FallingCnt); // high interval
        g_u32Freq =  u32Tmp1 + u32Tmp2;
        if(g_u32Freq)
            g_u32Freq = 24000000 / g_u32Freq;

        if(u32Tmp2 + u32Tmp2)
            g_u32DutyCycle = u32Tmp2 * 200 / (u32Tmp1 + u32Tmp2) + 1 >> 1;


        sprintf(g_strBuf, "Freq:%dHz", g_u32Freq);
        LCD_Print(2, strClearFreqVal);
        LCD_Print(2, g_strBuf);
        sprintf(g_strBuf, "Duty:%d ", g_u32DutyCycle);
        LCD_Print(3, strClearDutyVal);
        g_strBuf[8] = '%';
        LCD_Print(3, g_strBuf);
    }

}


