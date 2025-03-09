/*
 * @desc: assembly firmware for stm32f4 based hardware audio codec
 * @author: Aditya Chaudhary
 * @email: ac3101282@gmail.com
 * @date-initial: 17 Feb 2025
 * @date-release: -
*/


.section .vectors, "a"
.word _estack         @ Initial stack pointer (defined in linker script)
.word _reset + 1      @ Reset handler

.thumb
.syntax unified

@constants
.equ RCC_BASE, 0x40023800
.equ RCC_PLLI2SCFGR, 0x40023884
.equ BIT_26, 0x04000000
.equ BIT_27, 0x08000000
.equ BIT_23, 0x00800000

.equ RCC_AHB1ENR, (RCC_BASE + 0x30)
.equ RCC_APB1ENR, (RCC_BASE + 0x40)
.equ RCC_APB2ENR, (RCC_BASE + 0x44)

.equ SPI3_BASE, 0x40003C00
.equ SPI2_BASE, 0x40003800

.equ SPI_I2SCFGR, 0x1C
.equ SPI_I2SPR, 0x20
.equ SPI_DR, 0x0C
.equ SPI_SR, 0x08

.equ SAMPLE_RATE, 44100       // Example value, replace with actual
.equ TONE_FREQUENCY, 5000     // Example value, replace with actual
.equ DURATION, 10              // Example value, replace with actual

.equ HIGH_VALUE, 0x7FFF
.equ LOW_VALUE, 0x8000

.equ GPIOC_BASE, 0x40020800
.equ GPIOB_BASE, 0x40020400
.equ GPIOA_BASE, 0x40020000

.equ GPIOC_MODER, (GPIOC_BASE + 0x00)
.equ GPIOC_ODR, (GPIOC_BASE + 0x14)
.equ GPIOB_MODER, (GPIOB_BASE + 0x00)
.equ GPIOB_ODR, (GPIOB_BASE + 0x14)
.equ GPIOB_IDR, (GPIOB_BASE + 0x10)
.equ GPIOA_MODER, (GPIOA_BASE + 0x00)
.equ GPIOA_ODR, (GPIOA_BASE + 0x14)
.equ GPIOB_AF, (GPIOB_BASE + 0x20)
.equ GPIOB_AFH, (GPIOB_BASE + 0x24)
.equ GPIOA_AF, (GPIOA_BASE + 0x20)


.equ UART1_BASE, 0x40011000
.equ UART2_BASE, 0x40004400

.equ UART1_BRR, (UART1_BASE + 0x08)
.equ UART1_CR1, (UART1_BASE + 0x0C)
.equ UART1_DR, (UART1_BASE + 0x04)
.equ UART1_SR, (UART1_BASE + 0x00)

.equ UART2_BRR, (UART2_BASE + 0x08)
.equ UART2_CR1, (UART2_BASE + 0x0C)
.equ UART2_DR, (UART2_BASE + 0x04)
.equ UART2_SR, (UART2_BASE + 0x00)

@ Define register addresses
.equ FLASH_ACR,   0x40023C00  @ Flash Access Control Register
.equ RCC_CR,      0x40023800  @ RCC Clock Control Register
.equ RCC_PLLCFGR, 0x40023804  @ RCC PLL Configuration Register
.equ RCC_CFGR,    0x40023808  @ RCC Clock Configuration Register

@ Constants
.equ BIT0,      0x01        @ Bit 0
.equ BIT1,      0x02        @ Bit 1
.equ BIT8,      0x100       @ Bit 8
.equ BIT24,     0x01000000  @ Bit 24
.equ BIT25,     0x02000000  @ Bit 25
.equ HSI_EN,    BIT0        @ HSI enable bit
.equ HSI_RDY,   BIT1        @ HSI ready flag
.equ PLL_EN,    BIT24       @ PLL enable bit
.equ PLL_RDY,   BIT25       @ PLL ready flag
.equ PLL_SRC_HSI, 0x00000000  @ HSI selected as PLL source
.equ PLL_M_16,    0x00000010  @ M = 16
.equ PLL_N_336,   0x0000A800  @ N = 336 (shifted by 6)
.equ PLL_P_4,     0x00010000  @ P = 4 (PLLP = 2)
.equ SYS_CLK_PLL, 0x00000002  @ PLL selected as system clock
.equ FLASH_LATENCY, 0x00000002  @ 2 wait states
.equ PREFETCH_EN,    BIT8        @ Prefetch enable bit

.equ I2S_DR_, (SPI3_BASE + 0x0C)
.equ I2S_SR_, (SPI3_BASE + 0x08)

.equ I2S_DR_SP, (SPI2_BASE + 0x0C)
.equ I2S_SR_SP, (SPI2_BASE + 0x08)

.equ DELAY_SIZE, 256  @ Adjust this for deeper reverb (higher values = deeper reverb)

.section .bss
delay_buffer:
   .space DELAY_SIZE * 4  @ 256 samples, each 4 bytes (32-bit)



.section .text
.global _reset
.global main
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     .global clear_bss
.global copy_data

clear_bss:
    ldr r0, =_sbss
    ldr r1, =_ebss

copy_data:
    ldr r0, =_sdata      @ Load start of .data section (destination) into r0
    ldr r1, =_edata      @ Load end of .data section (boundary) into r1
    ldr r2, =_sidata     @ Load start of .data in Flash (source) into r2

copy_data_loop:
    cmp r0, r1           @ Check if we reached the end
    bcs copy_done        @ If r0 >= r1, stop

    ldr r3, [r2]         @ Load word from Flash (ROM) into r3
    str r3, [r0]         @ Store word into RAM (copy)

    adds r0, r0, #4      @ Move to next word in RAM
    adds r2, r2, #4      @ Move to next word in Flash


    B copy_data_loop     @ Repeat

copy_done:
    bx lr                @ Return

clear_bss_loop:
    cmp r0, r1
    bcs done

    movs r2, #0
    str r2, [r0]
    adds r0, r0, #4

    B clear_bss_loop

done:
    bx lr

@startup code
_reset:
    ldr r0, =_estack      @ Load stack pointer address
    mov sp, r0            @ Set stack pointer

    bl clear_bss
    bl copy_data

    bl main               @ Call main() function
    b .                   @ Infinite loop if main() returns

.global send_square_wave
send_square_wave:
    // Calculate samples_per_period and half_period_samples
    LDR R0, =SAMPLE_RATE
    LDR R1, =TONE_FREQUENCY
    UDIV R2, R0, R1           // R2 = samples_per_period
    LSR R3, R2, #1            // R3 = half_period_samples

    // Calculate total_samples = SAMPLE_RATE * DURATION
    LDR R4, =DURATION
    MUL R4, R0, R4            // R4 = total_samples

    // Load SPI3 base address into R5
    LDR R5, =SPI2_BASE

    // Initialize loop counter (i = 0)
    MOV R6, #0

send_loop:
    // Check if (i % samples_per_period) < half_period_samples
    UDIV R7, R6, R2           // R7 = i / samples_per_period
    MLS R7, R2, R7, R6        // R7 = i % samples_per_period
    CMP R7, R3
    BGE send_low_value

    // Send high value (0x7FFF)
    LDR R8, =HIGH_VALUE
    STR R8, [R5, #SPI_DR]
    B wait_for_ready

send_low_value:
    // Send low value (0x8000)
    LDR R8, =LOW_VALUE
    STR R8, [R5, #SPI_DR]

wait_for_ready:
    // Wait until SPI is ready to send the next sample (SPI_SR bit 1)
wait_spi_ready:
    LDR R8, [R5, #SPI_SR]
    LDR R9, =(1 << 1)
    TST R8, R9
    BEQ wait_spi_ready

    // Increment loop counter (i++)
    ADD R6, R6, #1

    // Check if i < total_samples
    CMP R6, R4
    BLT send_loop

    BX LR                       // Return from function

@ Enable clock for GPIOC
enable_test_led_clock:
    ldr r0, =RCC_AHB1ENR
    ldr r1, [r0]
    orr r1, r1, #(1 << 2)  @ Enable GPIOC clock (bit 2)
    str r1, [r0]
    bx lr

@ Set PC13 as Output
set_test_led_mode:
    ldr r0, =GPIOC_MODER
    ldr r1, [r0]
    bic r1, r1, #(3 << 26)  @ Clear bits 26:27 (PC13 mode)
    orr r1, r1, #(1 << 26)  @ Set bit 26 (PC13 as output)
    str r1, [r0]
    bx lr


change_system_clock:
    @flash related bit setting, change wait states
    ldr r0, =FLASH_ACR         @ Load FLASH_ACR address
    ldr r1, [r0]               @ Read current value
    bic r1, r1, #0x07          @ Clear latency bits
    orr r1, r1, #FLASH_LATENCY @ Set 2 wait states
    orr r1, r1, #PREFETCH_EN   @ Enable prefetch
    str r1, [r0]               @ Write back to FLASH_ACR

    @enable HSI(high speed internal oscillator)
    ldr r0, =RCC_CR            @ Load RCC_CR address
    ldr r1, [r0]               @ Read current value
    ldr r2, =(1 << 0)
    orr r1, r1, r2        @ Enable HSI
    str r1, [r0]               @ Write back to RCC_CR

    @wait for internal high speed clock ready flag
wait_hsi_ready:
    ldr r1, [r0]               @ Read RCC_CR
    ldr r2, =(1 << 1)
    tst r1, r2           @ Check HSI ready flag
    beq wait_hsi_ready         @ Wait until HSI is ready


    @configure PLL
    @1. PLL src
    ldr r0, =RCC_PLLCFGR       @ Load RCC_PLLCFGR address
    ldr r1, [r0]               @ Read current value
    ldr r2, =(1 << 22)
    bic r1, r1, r2    @ Clear PLL source bit (HSI selected)
    str r1, [r0]               @ Write back to RCC_PLLCFGR

    @2. M = 16
    ldr r1, [r0]               @ Read current value
    bic r1, r1, #0x0000003F    @ Clear M bits
    orr r1, r1, #PLL_M_16      @ Set M = 16
    str r1, [r0]               @ Write back to RCC_PLLCFGR

    @3. N = 336
    ldr r1, [r0]               @ Read current value
    ldr r2, =0x00007FC0
    bic r1, r1, r2    @ Clear N bits
    orr r1, r1, #PLL_N_336     @ Set N = 336
    str r1, [r0]               @ Write back to RCC_PLLCFGR

    @4. P = 2
    ldr r1, [r0]               @ Read current value
    ldr r2, =0x00030000
    bic r1, r1, r2    @ Clear P bits

    ldr r2, =(0 << 16)
    orr r1, r1, r2       @ Set P = 4 (PLLP = 2)
    str r1, [r0]               @ Write back to RCC_PLLCFGR

    @5. enable main PLL
    ldr r0, =RCC_CR            @ Load RCC_CR address
    ldr r1, [r0]               @ Read current value
    ldr r2, =(1 << 24)
    orr r1, r1, r2        @ Enable PLL
    str r1, [r0]               @ Write back to RCC_CR

    @6. wait for main pll clock ready flag
Wait_PLL_Ready:
    ldr r1, [r0]               @ Read RCC_CR
    ldr r2, =(1 << 25)
    tst r1, r2           @ Check PLL ready flag
    beq Wait_PLL_Ready         @ Wait until PLL is ready

    @7. system clock switch
    ldr r0, =RCC_CFGR          @ Load RCC_CFGR address
    ldr r1, [r0]               @ Read current value
    ldr r2, =0x00000003
    bic r1, r1, r2    @ Clear clock source bits

    ldr r2, =((1 << 1) | (0 << 0))
    orr r1, r1, r2   @ Select PLL as system clock
    str r1, [r0]               @ Write back to RCC_CFGR

    @8. wait for system clock flag
Wait_PLL_Switch:
    ldr r1, [r0]               @ Read RCC_CFGR
    ldr r2, =0x0000000C
    and r1, r1, r2    @ Mask clock source status bits
    ldr r2, =0x00000008
    cmp r1, r2        @ Check if PLL is selected
    bne Wait_PLL_Switch        @ Wait until PLL is selected

    bx lr


init_i2s_pll:
    // Configure PLLI2SCFGR: Set N = 302
    ldr r0, =RCC_PLLI2SCFGR
    ldr r1, [r0]
    ldr r2, =0x7FC0
    bic r1, r1, r2         // Clear bits 6-14

    ldr r2, =(302 << 6)

    orr r1, r1, r2     // Set N = 302
    str r1, [r0]

    // Configure PLLI2SCFGR: Set R = 2
    ldr r0, =RCC_PLLI2SCFGR
    ldr r1, [r0]
    ldr r2, =0x70000000
    bic r1, r1, r2     // Clear bits 28-30

    ldr r2, =(2 << 28)
    orr r1, r1, r2      // Set R = 2
    str r1, [r0]

    // Configure RCC_CFGR: Select PLLI2S as I2S clock source
    ldr r0, =RCC_CFGR
    ldr r1, [r0]
    ldr r2, =BIT_23
    bic r1, r1, r2        // Clear bit 23
    str r1, [r0]

    // Enable PLLI2S
    ldr r0, =RCC_CR
    ldr r1, [r0]
    ldr r2 , =BIT_26
    orr r1, r1, r2         // Set bit 26 to enable PLLI2S
    str r1, [r0]

    wait_plli2s_ready:
    ldr r0, =RCC_CR
    ldr r1, [r0]
    ldr r2, =BIT_27
    tst r1, r2             // Test bit 27
    BEQ wait_plli2s_ready       // Loop until bit 27 is set


    bx lr

init_i2s_mic:
    // Enable SPI3 clock (RCC_APB1ENR bit 15)
    ldr r0, =RCC_APB1ENR
    ldr r1, [r0]
    ldr r2, =(1 << 15)
    orr r1, r1, r2
    str r1, [r0]

    // Configure PB5 (SD) as alternate function
    //enable clock for GPIOB
    ldr r0, =RCC_AHB1ENR
    ldr r1, [r0]
    ldr r2, =(1 << 1)
    orr r1, r1, r2
    str r1, [r0]

    // set mode for PB5(SD) as alternate function
    ldr r0, =GPIOB_MODER
    ldr r1, [r0]
    ldr r2, =(3 << 10)
    bic r1, r1, r2

    ldr r2, =(2 << 10)
    orr r1, r1, r2
    str r1, [r0]

    //set alternate function
    ldr r0, =GPIOB_AF
    ldr r1, [r0]
    ldr r2 , =(0x00F00000)
    bic r1, r1, r2

    ldr r2,  =(0x00600000)
    orr r1, r1, r2
    str r1, [r0]

    // Configure PB3 (CK) as alternate function
    ldr r0, =GPIOB_MODER
    ldr r1, [r0]
    ldr r2, =(3 << 6)
    bic r1, r1, r2

    ldr r2, =(2 << 6)
    orr r1, r1, r2
    str r1, [r0]


    //set af
    ldr r0, =GPIOB_AF
    ldr r1, [r0]
    ldr r2, =(0x0000F000)
    bic r1, r1, r2

    ldr r2, =(0x00006000)
    orr r1, r1, r2
    str r1, [r0]

    // Configure PA4 (WS) as alternate function
    ldr r0, =GPIOA_MODER
    ldr r1, [r0]
    ldr r2, =(3 << 8)
    bic r1, r1, r2
    ldr r2, =(2 << 8)
    orr r1, r1, r2
    str r1, [r0]


    //set af
    ldr r0, =GPIOA_AF
    ldr r1, [r0]
    ldr r2, =(0x000F0000)
    bic r1, r1, r2

    ldr r2, =(0x00060000)
    orr r1, r1, r2
    str r1, [r0]


    // Configure I2S
    ldr r0, =SPI3_BASE

    // Clear SPI_I2SCFGR and SPI_I2SPR
    mov r1, #0
    str r1, [r0, #SPI_I2SCFGR]
    str r1, [r0, #SPI_I2SPR]

    // Set I2S mode (SPI_I2SCFGR bit 11)
    ldr r1, [r0, #SPI_I2SCFGR]
    ldr r2, =(1 << 11)
    orr r1, r1, r2
    str r1, [r0, #SPI_I2SCFGR]


    // Set Slave receive mode (SPI_I2SCFGR bits 8-9)
    ldr r1, [r0, #SPI_I2SCFGR]
    ldr r2, =(3 << 8)
    orr r1, r1, r2
    str r1, [r0, #SPI_I2SCFGR]

    // reSet I2S Philips Standard (SPI_I2SCFGR bits 4-5)
    ldr r1, [r0, #SPI_I2SCFGR]
    ldr r2, =((1 << 5) | (1 << 4))
    bic r1, r1, r2
    str r1, [r0, #SPI_I2SCFGR]

    // reSet PCM Frame Synchronization (SPI_I2SCFGR bit 7)
    ldr r1, [r0, #SPI_I2SCFGR]
    ldr r2, =(1 << 7)
    bic r1, r1, r2
    str r1, [r0, #SPI_I2SCFGR]

    // reSet Steady state clock polarity (SPI_I2SCFGR bit 3)
    ldr r1, [r0, #SPI_I2SCFGR]
    ldr r2, =(1 << 3)
    bic r1, r1, r2
    str r1, [r0, #SPI_I2SCFGR]

    // reSet Data length to 16 bits (SPI_I2SCFGR bits 1-2)
    ldr r1, [r0, #SPI_I2SCFGR]
    ldr r2, =((1 << 2) | (1 << 1))
    bic r1, r1, r2
    str r1, [r0, #SPI_I2SCFGR]

    // Set Channel length to 16 bits (SPI_I2SCFGR bit 0)
    ldr r1, [r0, #SPI_I2SCFGR]
    ldr r2, =(1 << 0)
    bic r1, r1, r2
    str r1, [r0, #SPI_I2SCFGR]

    // Set Master clock output disable (SPI_I2SPR bit 9)
    ldr r1, [r0, #SPI_I2SPR]
    ldr r2, =(1 << 9)
    bic r1, r1, r2
    str r1, [r0, #SPI_I2SPR]


    // Set ODD factor for prescaler (SPI_I2SPR bit 8)
    ldr r1, [r0, #SPI_I2SPR]
    ldr r2, =(1 << 8)
    orr r1, r1, r2
    str R1, [r0, #SPI_I2SPR]

    // Set I2S Linear prescaler (SPI_I2SPR bits 0-7)
    ldr r1, [r0, #SPI_I2SPR]
    mov r2, #53
    orr r1, r1, r2
    str r1, [r0, #SPI_I2SPR]

    // Enable I2S (SPI_I2SCFGR bit 10)
    ldr r1, [r0, #SPI_I2SCFGR]
    ldr r2, =(1 << 10)
    orr r1, r1, r2
    str r1, [r0, #SPI_I2SCFGR]
    bx lr


init_i2s_speaker:
        // Enable SPI2 clock (RCC_APB1ENR bit 14)
        ldr r0, =RCC_APB1ENR
        ldr r1, [r0]
        ldr r2, =(1 << 14)
        orr r1, r1, r2
        str r1, [r0]

        // Configure PB15 (SD) as alternate function
        //enable clock for GPIOB
        ldr r0, =RCC_AHB1ENR
        ldr r1, [r0]
        ldr r2, =(1 << 1)
        orr r1, r1, r2
        str r1, [r0]

        // set mode for PB5(SD) as alternate function
        ldr r0, =GPIOB_MODER
        ldr r1, [r0]
        ldr r2, =(3 << 30)
        bic r1, r1, r2

        ldr r2, =(2 << 30)
        orr r1, r1, r2
        str r1, [r0]

        //set alternate function
        ldr r0, =GPIOB_AFH
        ldr r1, [r0]
        ldr r2 , =(0xF0000000)
        bic r1, r1, r2

        ldr r2,  =(0x50000000)
        orr r1, r1, r2
        str r1, [r0]


        // Configure I2S
        ldr r0, =SPI2_BASE

        // Clear SPI_I2SCFGR and SPI_I2SPR
        mov r1, #0
        str r1, [r0, #SPI_I2SCFGR]
        str r1, [r0, #SPI_I2SPR]

        // Set I2S mode (SPI_I2SCFGR bit 11)
        ldr r1, [r0, #SPI_I2SCFGR]
        ldr r2, =(1 << 11)
        orr r1, r1, r2
        str r1, [r0, #SPI_I2SCFGR]


        // Set Master transmit mode (SPI_I2SCFGR bits 8-9)
        ldr r1, [r0, #SPI_I2SCFGR]
        ldr r2, =(2 << 8)
        orr r1, r1, r2
        str r1, [r0, #SPI_I2SCFGR]

        // reSet I2S Philips Standard (SPI_I2SCFGR bits 4-5)
        ldr r1, [r0, #SPI_I2SCFGR]
        ldr r2, =((1 << 5) | (1 << 4))
        bic r1, r1, r2
        str r1, [r0, #SPI_I2SCFGR]

        // reSet PCM Frame Synchronization (SPI_I2SCFGR bit 7)
        ldr r1, [r0, #SPI_I2SCFGR]
        ldr r2, =(1 << 7)
        bic r1, r1, r2
        str r1, [r0, #SPI_I2SCFGR]

        // reSet Steady state clock polarity (SPI_I2SCFGR bit 3)
        ldr r1, [r0, #SPI_I2SCFGR]
        ldr r2, =(1 << 3)
        bic r1, r1, r2
        str r1, [r0, #SPI_I2SCFGR]

        // reSet Data length to 16 bits (SPI_I2SCFGR bits 1-2)
        ldr r1, [r0, #SPI_I2SCFGR]
        ldr r2, =((1 << 2) | (1 << 1))
        bic r1, r1, r2
        str r1, [r0, #SPI_I2SCFGR]

        // Set Channel length to 16 bits (SPI_I2SCFGR bit 0)
        ldr r1, [r0, #SPI_I2SCFGR]
        ldr r2, =(1 << 0)
        bic r1, r1, r2
        str r1, [r0, #SPI_I2SCFGR]

        // Set Master clock output disable (SPI_I2SPR bit 9)
        ldr r1, [r0, #SPI_I2SPR]
        ldr r2, =(1 << 9)
        bic r1, r1, r2
        str r1, [r0, #SPI_I2SPR]


        // Set ODD factor for prescaler (SPI_I2SPR bit 8)
        ldr r1, [r0, #SPI_I2SPR]
        ldr r2, =(1 << 8)
        orr r1, r1, r2
        str R1, [r0, #SPI_I2SPR]

        // Set I2S Linear prescaler (SPI_I2SPR bits 0-7)
        ldr r1, [r0, #SPI_I2SPR]
        mov r2, #53
        orr r1, r1, r2
        str r1, [r0, #SPI_I2SPR]

        // Enable I2S (SPI_I2SCFGR bit 10)
        ldr r1, [r0, #SPI_I2SCFGR]
        ldr r2, =(1 << 10)
        orr r1, r1, r2
        str r1, [r0, #SPI_I2SCFGR]
        bx lr


set_button_mode:
    ldr r0, =GPIOB_MODER
    ldr r1, [r0]
    ldr r2, =(3 << 14)
    bic r1, r1, r2

    ldr r2, =(0 << 14)
    orr r1, r1, r2
    str r1, [r0]
    bx lr


set_buzzer_mode:
    ldr r0, =GPIOC_MODER
    ldr r1, [r0]
    ldr r2, =(3 << 26)
    bic r1, r1, r2

    ldr r2, =(1 << 26) @set as output
    orr r1, r1, r2
    str r1, [r0]
    bx lr

buzzer_on:
    ldr r0, =GPIOC_ODR
    ldr r1, [r0]
    ldr r2, =(1 << 13)
    orr r1, r1, r2
    str r1, [r0]
    bx lr

buzzer_off:
    ldr r0, =GPIOC_ODR
    ldr r1, [r0]
    ldr r2, =(1 << 13)
    bic r1, r1, r2
    str r1, [r0]
    bx lr


uart_init:
    @enable GPIOA clock
    ldr r0, =RCC_AHB1ENR
    ldr r1, [r0]
    ldr r2, =(1 << 0)
    orr r1, r1, r2
    str r1, [r0]

    @set PA2 as af number 7 (tx)
    ldr r0, =GPIOA_MODER
    ldr r1, [r0]
    ldr r2, =(3 << 4)
    bic r1, r1, r2

    ldr r2, =(2 << 4)
    orr r1, r1, r2
    str r1, [r0]

    ldr r0, =GPIOA_AF
    ldr r1, [r0]
    ldr r2, =0x00000F00
    bic r1, r1, r2

    ldr r2, =0x00000700
    orr r1, r1, r2
    str r1, [r0]

   @enable uart2 clock
    ldr r0, =RCC_APB1ENR
    ldr r1, [r0]
    ldr r2, =(1 << 17)
    orr r1, r1, r2
    str r1, [r0]

   @set baud rate
    ldr r0, =UART2_BRR  @ USART_BRR address
    ldr r1, =0x00002100
    str r1, [r0]         @ Store in BRR register

   @manipulate uart control register 1
    ldr r0, =UART2_CR1
    ldr r1, [r0]
    @enable transmitter, uart
    @set bit 15, 3, 13
    ldr r2, =((1 << 3) | (1 << 13))
    orr r1, r1, r2
    str r1, [r0]
    bx lr

uart_send_byte:
    ldr r1, =UART2_DR   @ Load USART_TDR address
    str r0, [r1]        @ Write byte (R0) to USART_TDR

wait_tx_complete:
    ldr r1, =UART2_SR   @ Load USART_SR address
    ldr r2, [r1]        @ Read status register
    mov r3, #0x80       @ TXE flag mask (bit 7)
    tst r2, r3          @ Check TXE flag
    beq wait_tx_complete

    bx lr


uart_send_16bit:
    push {r1, r2, lr}   @ Save registers

    and r1, r0, #0xFF   @ Extract lower 8 bits
    lsr r2, r0, #8      @ Extract upper 8 bits

    bl uart_send_byte   @ Send lower byte (LSB first)
    mov r0, r2          @ Move upper byte to R0
    bl uart_send_byte   @ Send upper byte (MSB)

    pop {r1, r2, lr}    @ Restore registers
    bx lr               @ Return

init_bar_display:
    @A5-A12
    @init GPIOA clock
    ldr r0, =RCC_AHB1ENR
    ldr r1, [r0]
    ldr r2, =(1 << 0)
    orr r1, r1, r2
    str r1, [r0]

    @set A5-A12 in output mode
    ldr r0, =GPIOA_MODER
    ldr r1, [r0]
    ldr r2, =((3 << 0) | (3 << 2) | (3 << 10) | (3 << 12) | (3 << 14) | (3 << 16) | (3 << 18) | (3 << 20) | (3 << 22) | (3 << 24))
    bic r1, r1, r2

    ldr r2, =((1 << 0) | (1 << 2) | (1 << 10) | (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18) | (1 << 20) | (1 << 22) | (1 << 24))
    orr r1, r1, r2
    str r1, [r0]

    bx lr

all_bar_high:
    ldr r0, =GPIOA_ODR
    ldr r1, [r0]
    ldr r2, =((1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 0) | (1 << 1) | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12))
    orr r1, r1, r2
    str r1, [r0]
    bx lr

all_bar_low:
    ldr r0, =GPIOA_ODR
    ldr r1, [r0]
    ldr r2, =((1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 0) | (1 << 1) | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12))
    bic r1, r1, r2
    str r1, [r0]
    bx lr

startup_counter:
    ldr r4, =GPIOA_ODR      @ Load GPIOA output register address
    mov r5, #0              @ Clear the register for initial state
    str r5, [r4]            @ Clear all bars initially

    mov r6, #4              @ Total bars (A5 - A8)

    mov r1, #5              @ Start from bit 5

@ from 5 - 8
loop_startup_1:
    ldr r3, [r4]            @ Read current GPIOA output state
    mov r0, #1              @ Start with bit 0
    lsl r0, r0, r1          @ Shift to the correct bar position
    orr r3, r3, r0          @ Turn on one bar at a time
    str r3, [r4]            @ Write back to output register

    bl delay                @ Call delay function

    add r1, r1, #1          @ Move to next bit (A6, A7, A8)
    cmp r1, #10              @ Stop after A8 (bit 8)
    blt loop_startup_1        @ If not, continue the loop

@ 0 and 1
     mov r1, #0

loop_startup_2:
    ldr r3, [r4]            @ Read current GPIOA output state
    mov r0, #1              @ Start with bit 0
    lsl r0, r0, r1          @ Shift to the correct bar position
    orr r3, r3, r0          @ Turn on one bar at a time
    str r3, [r4]            @ Write back to output register

    bl delay                @ Call delay function

    add r1, r1, #1          @ Move to next bit (A6, A7, A8)
    cmp r1, #2              @ Stop after A8 (bit 8)

    blt loop_startup_2

@ 0 and 1
     mov r1, #10

loop_startup_3:
    ldr r3, [r4]            @ Read current GPIOA output state
    mov r0, #1              @ Start with bit 0
    lsl r0, r0, r1          @ Shift to the correct bar position
    orr r3, r3, r0          @ Turn on one bar at a time
    str r3, [r4]            @ Write back to output register

    bl delay                @ Call delay function

    add r1, r1, #1          @ Move to next bit (A6, A7, A8)
    cmp r1, #13              @ Stop after A8 (bit 8)

    blt loop_startup_3

    bx lr

main:
    bl change_system_clock

    @init pll for i2s
    bl init_i2s_pll

    @init i2s mic
    bl init_i2s_mic

    @init i2s, speaker
    bl init_i2s_speaker

    bl send_square_wave

    @initial, enable clock for test led
    bl enable_test_led_clock

    @ put test led in output mode
    bl set_test_led_mode

    @ put button pin in input mode
    bl set_button_mode

    @ set buzzer mode
    bl set_buzzer_mode

    bl buzzer_on
    bl delay
    bl buzzer_off

    @ bar display initialise
    bl init_bar_display

    @ startup counter
    @bl startup_counter     @ Call startup_counter

    bl uart_init

    ldr r8, =delay_buffer  @ Load delay buffer base address
    mov r9, #0             @ Circular buffer index
loop:
    ldr r0, =I2S_DR_      @ Load I2S1 (Mic) Data Register address

wait_for_mic_data:
    ldr r1, =I2S_SR_      @ Load I2S1 Status Register
    ldr r2, [r1]         @ Read Status Register
    ldr r3, =(1 << 0)    @ RXNE (Receive Not Empty flag, bit 0)
    tst r2, r3           @ Check if data is ready
    beq wait_for_mic_data    @ Wait until RXNE is set

    ldr r4, [r0]         @ Read 32-bit audio data from I2S mic
    lsr r4, r4, #8       @ Align 16-bit sample (Right justify 24-bit data)


    @ ---- Implement Reverb ----
    @ldr r6, =delay_buffer     @ Load delay buffer base address
    @ldr r7, [r6]             @ Get old sample (D samples ago)
    @add r4, r4, r7           @ Apply reverb: y(n) = x(n) + x(n - D)
    @str r4, [r6]             @ Store new sample in buffer
    @add r6, r6, #4           @ Move to next buffer index (circular buffer effect)

    cmp r4, #0           @ Check if value is negative
    bge check_noise      @ If positive, continue
    rsb r4, r4, #0       @ Take absolute value if negative

check_noise:
    cmp r4, #10        @ Check if absolute value is below noise threshold
    blo mute_audio       @ If so, mute it by setting to zero

     @ ---- LED Bar Control ----
     @ turn all leds off

     ldr r5, =GPIOA_ODR
     ldr r6, [r5]
     ldr r7, =((1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 0) | (1 << 1) | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12))
     bic r6, r6, r7
     str r6, [r5]

     @check amplitude and according to amplitude, turn that much leds
     ldr r7, =10
     cmp r4, r7
     blt led_1


led_1:
     ldr r5, =GPIOA_ODR
     ldr r6, [r5]
     ldr r7, =((1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 0) | (1 << 1) | (1 << 9) | (1 << 10))
     bic r6, r6, r7
     str r6, [r5]

     ldr r7, =((1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 0) | (1 << 1) | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12))
     orr r6, r6, r7
     str r6, [r5]


wait_for_speaker_ready:
    ldr r1, =I2S_SR_SP      @ Load I2S2 Status Register
    ldr r2, [r1]         @ Read Status Register
    ldr r3, =(1 << 1)    @ TXE (Transmit Empty flag, bit 1)
    tst r2, r3
    beq wait_for_speaker_ready  @ Wait until TXE is set

    ldr r0, =I2S_DR_SP      @ Load I2S2 (Speaker) Data Register address
    str r4, [r0]         @ Send aligned 16-bit data to speaker

    @lsl r4, r4, #1

    mov r0, r4
    bl uart_send_16bit    @send 16 bit audio data via uart
    b loop



mute_audio:
    mov r4, #0           @ Mute small signals
    b wait_for_speaker_ready

    bx lr


delay:
    ldr r5, =10199999           @ Simple delay loop (adjust as needed)
delay_loop:
    subs r5, r5, #1
    bne delay_loop
    bx lr


multiplex_delay:
    ldr r5, =100000
mul_delay_loop:
    subs r5, r5, #1
    bne mul_delay_loop
    bx lr
