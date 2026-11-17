/*
 * Chroma 74H bootloader (CC1310F64): flash sector 0 plus the CCFG.
 * The app (oepl_CC1310, --define=OEPL_APP) owns 0x1000-0xEFFF.
 */
--retain=gVectors
--stack_size=512
--heap_size=0
--entry_point ResetISR
--diag_suppress=10063

MEMORY
{
    BOOT (RX) : origin = 0x00000000, length = 0x00001000
    CCFG (R)  : origin = 0x0000FFA8, length = 0x00000058
    SRAM (RWX): origin = 0x20000000, length = 0x00004000
}

SECTIONS
{
    .intvecs        :   > 0x00000000
    .text           :   > BOOT
    .const          :   > BOOT
    .constdata      :   > BOOT
    .rodata         :   > BOOT
    .cinit          :   > BOOT
    .pinit          :   > BOOT
    .init_array     :   > BOOT
    .ccfg           :   > CCFG

    .data           :   > SRAM
    .bss            :   > SRAM
    .sysmem         :   > SRAM
    .stack          :   > SRAM (HIGH)
}
