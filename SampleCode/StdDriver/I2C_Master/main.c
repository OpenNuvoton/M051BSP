/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/05/22 2:41p $
 * @brief    M051 Series I2C Driver Sample Code (Master)
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"

#define PLL_CLOCK           50000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t g_u8DeviceAddr;
static volatile uint8_t g_au8MstTxData[3];
static volatile uint8_t g_u8MstRxData;
static volatile uint8_t g_u8MstDataLen;
static volatile uint8_t g_u8MstEndFlag = 0;
static volatile uint8_t g_u8MstTxAbortFlag = 0;
static volatile uint8_t g_u8MstRxAbortFlag = 0;
static volatile uint8_t g_u8MstReStartFlag = 0;
static volatile uint8_t g_u8TimeoutFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2C0HandlerFn = NULL;
extern char GetChar(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
        g_u8TimeoutFlag = 1;
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1));    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen != 2)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA_SI);
        }
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, ((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
    {
        g_u8MstRxData = (unsigned char) I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
        g_u8MstEndFlag = 1;
    }
    else
    {
        /* Error condition process */
        printf("[MasterRx] Status [0x%x] Unexpected abort!! Anykey to re-start\n", u32Status);
        if(u32Status == 0x38)                 /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        else if(u32Status == 0x30)            /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        else if(u32Status == 0x48)            /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        else if(u32Status == 0x00)            /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        /*Setting MasterRx abort flag for re-start mechanism*/
        g_u8MstRxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        u32TimeOutCnt = I2C_TIMEOUT;
        while(I2C0->I2CON & I2C_I2CON_SI_Msk)
            if(--u32TimeOutCnt == 0) break;
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen != 3)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
    else
    {
        /* Error condition process */
        printf("[MasterTx] Status [0x%x] Unexpected abort!! Anykey to re-start\n", u32Status);

        if(u32Status == 0x38)                   /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        else if(u32Status == 0x00)              /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        else if(u32Status == 0x30)              /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        else if(u32Status == 0x48)              /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        else if(u32Status == 0x10)              /* Master repeat start, clear SI */
        {
            I2C_SET_DATA(I2C0, (uint32_t)((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        }
        /*Setting MasterTRx abort flag for re-start mechanism*/
        g_u8MstTxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
        u32TimeOutCnt = I2C_TIMEOUT;
        while(I2C0->I2CON & I2C_I2CON_SI_Msk)
            if(--u32TimeOutCnt == 0) break;
    }
}

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

    /* Enable UART & I2C0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);

    /* Configure the SDA0 & SCL0 of I2C0 pins */
    SYS->P3_MFP &= ~(SYS_MFP_P34_Msk | SYS_MFP_P35_Msk);
    SYS->P3_MFP |= (SYS_MFP_P34_SDA0 | SYS_MFP_P35_SCL0);

    /* I2C pin enable schmitt trigger */
    SYS->P3_MFP |= ((BIT4 | BIT5) << SYS_P3_MFP_P3_TYPE_Pos);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}

int32_t I2C0_Read_Write_SLAVE(uint8_t slvaddr)
{
    uint32_t i;

    do
    {
        /* Enable I2C timeout */
        I2C_EnableTimeout(I2C0, 0);
        g_u8MstReStartFlag = 0;
        g_u8DeviceAddr = slvaddr;
        g_u8TimeoutFlag = 0;

        for(i = 0; i < 0x100; i++)
        {
            g_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
            g_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
            g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);

            g_u8MstDataLen = 0;
            g_u8MstEndFlag = 0;

            /* I2C function to write data to slave */
            s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

            /* I2C as master sends START signal */
            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA);

            /* Wait I2C Tx Finish or Unexpected Abort*/
            do
            {
                if(g_u8TimeoutFlag)
                {
                    printf(" MasterTx time out, any to reset IP\n");
                    getchar();
                    SYS->IPRSTC2 |= SYS_IPRSTC2_I2C0_RST_Msk;
                    SYS->IPRSTC2 = 0;
                    I2C0_Init();
                    /* Set MasterTx abort flag*/
                    g_u8MstTxAbortFlag = 1;
                }
            } while(g_u8MstEndFlag == 0 && g_u8MstTxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if(g_u8MstTxAbortFlag)
            {
                /* Clear MasterTx abort flag*/
                g_u8MstTxAbortFlag = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag = 1;
                break;
            }

            /* I2C function to read data from slave */
            s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;

            g_u8MstDataLen = 0;
            g_u8DeviceAddr = slvaddr;

            I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_STA);


            /* Wait I2C Rx Finish or Unexpected Abort*/
            do {
                if(g_u8TimeoutFlag)
                {
                    /* When I2C timeout, reset IP*/
                    printf(" MasterRx time out, any to reset IP\n");
                    getchar();
                    SYS->IPRSTC2 |= SYS_IPRSTC2_I2C0_RST_Msk;
                    SYS->IPRSTC2 = 0;
                    I2C0_Init();
                    /* Set MasterRx abort flag*/
                    g_u8MstRxAbortFlag = 1;
                }
            } while(g_u8MstEndFlag == 0 && g_u8MstRxAbortFlag == 0);
            g_u8MstEndFlag = 0;

            if(g_u8MstRxAbortFlag )
            {
                /* Clear MasterRx abort flag*/
                g_u8MstRxAbortFlag = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag = 1;
                break;
            }
        }
    } while(g_u8MstReStartFlag); /*If unexpected abort happens, re-start the transmition*/

    /* Compare data */
    if(g_u8MstRxData != g_au8MstTxData[2])
    {
        printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData);
        return -1;
    }

    printf("Master Access Slave (0x%X) Test OK\n", slvaddr);
    return 0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */
    printf("\n");
    printf("+-------------------------------------------------------+\n");
    printf("| M05xx I2C Driver Sample Code(Master) for access Slave |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)               |\n");
    printf("+-------------------------------------------------------+\n");

    printf("Configure I2C0 as a master.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(P3.4), I2C0_SCL(P3.5)\n");

    /* Init I2C0 */
    I2C0_Init();

    printf("\n");
    printf("Check I2C Slave(I2C0) is running first!\n");
    printf("Press any key to continue.\n");
    GetChar();

    /* Access Slave with no address */
    printf("\n");
    printf(" == No Mask Address ==\n");
    I2C0_Read_Write_SLAVE(0x15);
    I2C0_Read_Write_SLAVE(0x35);
    I2C0_Read_Write_SLAVE(0x55);
    I2C0_Read_Write_SLAVE(0x75);
    printf("SLAVE Address test OK.\n");

    /* Access Slave with address mask */
    printf("\n");
    printf(" == Mask Address ==\n");
    I2C0_Read_Write_SLAVE(0x15 & ~0x01);
    I2C0_Read_Write_SLAVE(0x35 & ~0x04);
    I2C0_Read_Write_SLAVE(0x55 & ~0x01);
    I2C0_Read_Write_SLAVE(0x75 & ~0x04);
    printf("SLAVE Address Mask test OK.\n");

    s_I2C0HandlerFn = NULL;

    /* Close I2C0 */
    I2C0_Close();

    while(1);
}



