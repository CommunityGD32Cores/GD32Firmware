;/*!
;    \file    startup_gd32l301.s
;    \brief   start up file
;
;    \version 2020-09-17, V1.0.0, firmware for GD32L23x
;*/
;
;/*
;    Copyright (c) 2020, GigaDevice Semiconductor Inc.
;
;    Redistribution and use in source and binary forms, with or without modification, 
;are permitted provided that the following conditions are met:
;
;    1. Redistributions of source code must retain the above copyright notice, this 
;       list of conditions and the following disclaimer.
;    2. Redistributions in binary form must reproduce the above copyright notice, 
;       this list of conditions and the following disclaimer in the documentation 
;       and/or other materials provided with the distribution.
;    3. Neither the name of the copyright holder nor the names of its contributors 
;       may be used to endorse or promote products derived from this software without 
;       specific prior written permission.
;
;    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
;AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
;WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
;IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
;INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
;NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
;PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
;WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
;ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
;OF SUCH DAMAGE.
;*/

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)                         ; top of stack
        DCD     Reset_Handler                       ; Vector Number 1,Reset Handler

        DCD     NMI_Handler                         ; Vector Number 2,NMI Handler
        DCD     HardFault_Handler                   ; Vector Number 3,Hard Fault Handler
        DCD     0                                   ; Reserved
        DCD     0                                   ; Reserved
        DCD     0                                   ; Reserved
        DCD     0                                   ; Reserved
        DCD     0                                   ; Reserved
        DCD     0                                   ; Reserved
        DCD     0                                   ; Reserved
        DCD     SVC_Handler                         ; Vector Number 11,SVCall Handler
        DCD     0                                   ; Reserved
        DCD     0                                   ; Reserved
        DCD     PendSV_Handler                      ; Vector Number 14,PendSV Handler
        DCD     SysTick_Handler                     ; Vector Number 15,SysTick Handler

;       /* external interrupts handler */
        DCD     WWDGT_IRQHandler                             ; 16:Window Watchdog Timer
        DCD     LVD_IRQHandler                               ; 17:LVD through EXTI Line detect
        DCD     TAMPER_STAMP_IRQHandler                      ; 18:RTC Tamper and TimeStamp through EXTI Line detect
        DCD     RTC_WKUP_IRQHandler                          ; 19:RTC Wakeup from EXTI interrupt
        DCD     FMC_IRQHandler                               ; 20:FMC global interrupt
        DCD     RCU_CTC_IRQHandler                           ; 21:RCU or CTC global interrupt
        DCD     EXTI0_IRQHandler                             ; 22:EXTI Line 0
        DCD     EXTI1_IRQHandler                             ; 23:EXTI Line 1
        DCD     EXTI2_IRQHandler                             ; 24:EXTI Line 2
        DCD     EXTI3_IRQHandler                             ; 25:EXTI Line 3
        DCD     EXTI4_IRQHandler                             ; 26:EXTI Line 4
        DCD     DMA_Channel0_IRQHandler                      ; 27:DMA Channel 0 
        DCD     DMA_Channel1_IRQHandler                      ; 28:DMA Channel 1 
        DCD     DMA_Channel2_IRQHandler                      ; 29:DMA Channel 2 
        DCD     DMA_Channel3_IRQHandler                      ; 30:DMA Channel 3 
        DCD     DMA_Channel4_IRQHandler                      ; 31:DMA Channel 4 
        DCD     DMA_Channel5_IRQHandler                      ; 32:DMA Channel 5 
        DCD     DMA_Channel6_IRQHandler                      ; 33:DMA Channel 6 
        DCD     ADC_IRQHandler                               ; 34:ADC interrupt 
        DCD     USBD_HP_IRQHandler                           ; 35:USBD High Priority interrupt
        DCD     USBD_LP_IRQHandler                           ; 36:USBD Low Priority interrupt
        DCD     TIMER1_IRQHandler                            ; 37:TIMER1
        DCD     TIMER2_IRQHandler                            ; 38:TIMER2
        DCD     TIMER8_IRQHandler                            ; 39:TIMER8
        DCD     TIMER11_IRQHandler                           ; 40:TIMER11
        DCD     TIMER5_IRQHandler                            ; 41:TIMER5
        DCD     TIMER6_IRQHandler                            ; 42:TIMER6
        DCD     USART0_IRQHandler                            ; 43:USART0
        DCD     USART1_IRQHandler                            ; 44:USART1
        DCD     UART3_IRQHandler                             ; 45:UART3
        DCD     UART4_IRQHandler                             ; 46:UART4
        DCD     I2C0_EV_IRQHandler                           ; 47:I2C0 Event
        DCD     I2C0_ER_IRQHandler                           ; 48:I2C0 Error
        DCD     I2C1_EV_IRQHandler                           ; 49:I2C1 Event
        DCD     I2C1_ER_IRQHandler                           ; 50:I2C1 Error
        DCD     SPI0_IRQHandler                              ; 51:SPI0
        DCD     SPI1_IRQHandler                              ; 52:SPI1
        DCD     DAC_IRQHandler                               ; 53:DAC
        DCD     0                                            ; 54:Reserved
        DCD     I2C2_EV_IRQHandler                           ; 55:I2C2 Event
        DCD     I2C2_ER_IRQHandler                           ; 56:I2C2 Error
        DCD     RTC_Alarm_IRQHandler                         ; 57:RTC Alarm through EXTI Line detect
        DCD     USBD_WKUP_IRQHandler                         ; 58:USBD wakeup through EXTI Line detect
        DCD     EXTI5_9_IRQHandler                           ; 59:EXTI5 to EXTI9
        DCD     0                                            ; 60:Reserved
        DCD     0                                            ; 61:Reserved
        DCD     0                                            ; 62:Reserved
        DCD     EXTI10_15_IRQHandler                         ; 63:EXTI10 to EXT15
        DCD     0                                            ; 64:Reserved
        DCD     0                                            ; 65:Reserved
        DCD     0                                            ; 66:Reserved
        DCD     0                                            ; 67:Reserved
        DCD     0                                            ; 68:Reserved
        DCD     0                                            ; 69:Reserved
        DCD     0                                            ; 70:Reserved
        DCD     DMAMUX_IRQHandler                            ; 71:Reserved
        DCD     CMP0_IRQHandler                              ; 72:Comparator 0 interrupt through EXTI Line detect
        DCD     CMP1_IRQHandler                              ; 73:Comparator 1 interrupt through EXTI Line detect
        DCD     I2C0_WKUP_IRQHandler                         ; 74:I2C0 Wakeup interrupt through EXTI Line detect
        DCD     I2C2_WKUP_IRQHandler                         ; 75:I2C2 Wakeup interrupt through EXTI Line detect
        DCD     USART0_WKUP_IRQHandler                       ; 76:USART0 Wakeup interrupt through EXTI Line detect
        DCD     LPUART_IRQHandler                            ; 77:LPUART global interrupt
        DCD     CAU_IRQHandler                               ; 78:CAU
        DCD     TRNG_IRQHandler                              ; 79:TRNG
        DCD     SLCD_IRQHandler                              ; 80:SLCD
        DCD     USART1_WKUP_IRQHandler                       ; 81:USART1 Wakeup interrupt through EXTI Line detect
        DCD     I2C1_WKUP_IRQHandler                         ; 82:I2C1 Wakeup interrupt through EXTI Line detect
        DCD     LPUART_WKUP_IRQHandler                       ; 83:LPUART Wakeup interrupt through EXTI Line detect
        DCD     LPTIMER_IRQHandler                           ; 84:LPTIMER interrupt 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:NOROOT:REORDER(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
HardFault_Handler
        B HardFault_Handler      

        PUBWEAK SVC_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:NOROOT:REORDER(1)
SysTick_Handler
        B SysTick_Handler

        PUBWEAK WWDGT_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
WWDGT_IRQHandler
        B WWDGT_IRQHandler

        PUBWEAK LVD_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
LVD_IRQHandler
        B LVD_IRQHandler

        PUBWEAK TAMPER_STAMP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TAMPER_STAMP_IRQHandler
        B TAMPER_STAMP_IRQHandler

        PUBWEAK RTC_WKUP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
RTC_WKUP_IRQHandler
        B RTC_WKUP_IRQHandler

        PUBWEAK FMC_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
FMC_IRQHandler
        B FMC_IRQHandler

        PUBWEAK RCU_CTC_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
RCU_CTC_IRQHandler
        B RCU_CTC_IRQHandler

        PUBWEAK EXTI0_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
EXTI0_IRQHandler
        B EXTI0_IRQHandler

        PUBWEAK EXTI1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
EXTI1_IRQHandler
        B EXTI1_IRQHandler

        PUBWEAK EXTI2_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
EXTI2_IRQHandler
        B EXTI2_IRQHandler

        PUBWEAK EXTI3_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
EXTI3_IRQHandler
        B EXTI3_IRQHandler 
 
        PUBWEAK EXTI4_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
EXTI4_IRQHandler 
        B EXTI4_IRQHandler 

        PUBWEAK DMA_Channel0_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
DMA_Channel0_IRQHandler
        B DMA_Channel0_IRQHandler

        PUBWEAK DMA_Channel1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
DMA_Channel1_IRQHandler
        B DMA_Channel1_IRQHandler

        PUBWEAK DMA_Channel2_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
DMA_Channel2_IRQHandler
        B DMA_Channel2_IRQHandler

        PUBWEAK DMA_Channel3_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
DMA_Channel3_IRQHandler
        B DMA_Channel3_IRQHandler

        PUBWEAK DMA_Channel4_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
DMA_Channel4_IRQHandler
        B DMA_Channel4_IRQHandler

        PUBWEAK DMA_Channel5_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
DMA_Channel5_IRQHandler
        B DMA_Channel5_IRQHandler

        PUBWEAK DMA_Channel6_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
DMA_Channel6_IRQHandler
        B DMA_Channel6_IRQHandler

        PUBWEAK ADC_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
ADC_IRQHandler
        B ADC_IRQHandler

        PUBWEAK USBD_HP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
USBD_HP_IRQHandler
        B USBD_HP_IRQHandler

        PUBWEAK USBD_LP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
USBD_LP_IRQHandler
        B USBD_LP_IRQHandler

        PUBWEAK TIMER1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TIMER1_IRQHandler
        B TIMER1_IRQHandler

        PUBWEAK TIMER2_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TIMER2_IRQHandler
        B TIMER2_IRQHandler

        PUBWEAK TIMER8_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TIMER8_IRQHandler
        B TIMER8_IRQHandler

        PUBWEAK TIMER11_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TIMER11_IRQHandler
        B TIMER11_IRQHandler

        PUBWEAK TIMER5_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TIMER5_IRQHandler
        B TIMER5_IRQHandler

        PUBWEAK TIMER6_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TIMER6_IRQHandler
        B TIMER6_IRQHandler

        PUBWEAK USART0_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
USART0_IRQHandler
        B USART0_IRQHandler

        PUBWEAK USART1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
USART1_IRQHandler
        B USART1_IRQHandler

        PUBWEAK UART3_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART3_IRQHandler
        B UART3_IRQHandler

        PUBWEAK UART4_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
UART4_IRQHandler
        B UART4_IRQHandler

        PUBWEAK I2C0_EV_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C0_EV_IRQHandler
        B I2C0_EV_IRQHandler

        PUBWEAK I2C0_ER_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C0_ER_IRQHandler
        B I2C0_ER_IRQHandler

        PUBWEAK I2C1_EV_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C1_EV_IRQHandler
        B I2C1_EV_IRQHandler

        PUBWEAK I2C1_ER_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C1_ER_IRQHandler
        B I2C1_ER_IRQHandler

        PUBWEAK SPI0_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SPI0_IRQHandler
        B SPI0_IRQHandler

        PUBWEAK SPI1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SPI1_IRQHandler
        B SPI1_IRQHandler

        PUBWEAK DAC_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
DAC_IRQHandler
        B DAC_IRQHandler

        PUBWEAK I2C2_EV_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C2_EV_IRQHandler
        B I2C2_EV_IRQHandler

        PUBWEAK I2C2_ER_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C2_ER_IRQHandler
        B I2C2_ER_IRQHandler

        PUBWEAK RTC_Alarm_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
RTC_Alarm_IRQHandler
        B RTC_Alarm_IRQHandler

        PUBWEAK USBD_WKUP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
USBD_WKUP_IRQHandler
        B USBD_WKUP_IRQHandler

        PUBWEAK EXTI5_9_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
EXTI5_9_IRQHandler
        B EXTI5_9_IRQHandler

        PUBWEAK EXTI10_15_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
EXTI10_15_IRQHandler
        B EXTI10_15_IRQHandler

        PUBWEAK DMAMUX_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
DMAMUX_IRQHandler
        B DMAMUX_IRQHandler

        PUBWEAK CMP0_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
CMP0_IRQHandler
        B CMP0_IRQHandler

        PUBWEAK CMP1_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
CMP1_IRQHandler
        B CMP1_IRQHandler

        PUBWEAK I2C0_WKUP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C0_WKUP_IRQHandler
        B I2C0_WKUP_IRQHandler

        PUBWEAK I2C2_WKUP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C2_WKUP_IRQHandler
        B I2C2_WKUP_IRQHandler

        PUBWEAK USART0_WKUP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
USART0_WKUP_IRQHandler
        B USART0_WKUP_IRQHandler

        PUBWEAK LPUART_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
LPUART_IRQHandler
        B LPUART_IRQHandler

        PUBWEAK CAU_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
CAU_IRQHandler
        B CAU_IRQHandler

        PUBWEAK TRNG_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
TRNG_IRQHandler
        B TRNG_IRQHandler

        PUBWEAK SLCD_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
SLCD_IRQHandler
        B SLCD_IRQHandler

        PUBWEAK USART1_WKUP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
USART1_WKUP_IRQHandler
        B USART1_WKUP_IRQHandler

        PUBWEAK I2C1_WKUP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
I2C1_WKUP_IRQHandler
        B I2C1_WKUP_IRQHandler

        PUBWEAK LPUART_WKUP_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
LPUART_WKUP_IRQHandler
        B LPUART_WKUP_IRQHandler

        PUBWEAK LPTIMER_IRQHandler
        SECTION .text:CODE:NOROOT:REORDER(1)
LPTIMER_IRQHandler
        B LPTIMER_IRQHandler

        END