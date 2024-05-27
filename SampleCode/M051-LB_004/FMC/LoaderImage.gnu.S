;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* @copyright SPDX-License-Identifier: Apache-2.0															*/
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/


	.section .rodata
	.globl	g_u32LoaderImageBase
	.globl	g_u32LoaderImageLimit
    
	.align	4

	.text

g_u32LoaderImageBase:
    .incbin  "../../FMC_LD.bin"
g_u32LoaderImageLimit:
    .space   4
    
    .end
