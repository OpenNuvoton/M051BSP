;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* @copyright SPDX-License-Identifier: Apache-2.0															*/
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/


    AREA _image, DATA, READONLY

    EXPORT  loaderImageBase
    EXPORT  loaderImageLimit

    ALIGN   4
        
loaderImageBase
    INCBIN .\obj\FMC_IAP_LD.bin
loaderImageLimit

    
    END