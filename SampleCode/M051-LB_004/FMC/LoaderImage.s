;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* @copyright SPDX-License-Identifier: Apache-2.0															*/
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/


    AREA _image, DATA, READONLY

    EXPORT  g_u32LoaderImageBase
    EXPORT  g_u32LoaderImageLimit
    
    ALIGN   4
        
g_u32LoaderImageBase
    INCBIN .\FMC_LD.bin
g_u32LoaderImageLimit

    
    END