/***************************************************************************//**
 * @file     targetdev.c
 * @brief    ISP support function source file
 * @version  0x32
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"
#include "isp_user.h"

uint32_t GetApromSize()
{
    //the smallest of APROM size is 2K
    uint32_t size = 0x800, data;
    int result;

    do
    {
        result = FMC_Read_User(size, &data);

        if(result < 0)
        {
            return size;
        }
        else
        {
            size *= 2;
        }
    }
    while(1);
}

// Data Flash size is 4K.
void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    *addr = 0x1F000;
    *size = 4096;
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
