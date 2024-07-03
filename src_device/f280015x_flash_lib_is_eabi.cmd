MEMORY
{
   BEGIN            : origin = 0x00080000, length = 0x00000002
   BOOT_RSVD        : origin = 0x00000002, length = 0x00000126

   RAMM0            : origin = 0x00000128, length = 0x000002D8
   RAMM1            : origin = 0x00000400, length = 0x000003F8
   // RAMM1_RSVD       : origin = 0x000007F8, length = 0x00000008 /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   RAMLS0           : origin = 0x00008000, length = 0x00002000
   RAMLS1           : origin = 0x0000A000, length = 0x00001FF8
   // RAMLS1_RSVD      : origin = 0x0000BFF8, length = 0x00000008 /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   RESET            : origin = 0x003FFFC0, length = 0x00000002

   /* Flash sectors */
   FLASH_BANK0_SEC_0_7     : origin = 0x080002, length = 0x1FFE  /* on-chip Flash */
   FLASH_BANK0_SEC_8_15    : origin = 0x082000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_16_23   : origin = 0x084000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_24_31   : origin = 0x086000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_32_29   : origin = 0x088000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_40_47   : origin = 0x08A000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_48_55   : origin = 0x08C000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_56_63   : origin = 0x08E000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_64_71   : origin = 0x090000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_72_79   : origin = 0x092000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_80_87   : origin = 0x094000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_88_95   : origin = 0x096000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_96_103  : origin = 0x098000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_104_111 : origin = 0x09A000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_112_119 : origin = 0x09C000, length = 0x2000  /* on-chip Flash */
   FLASH_BANK0_SEC_120_127 : origin = 0x09E000, length = 0x1FF0  /* on-chip Flash */

   // FLASH_BANK0_SEC_127_RSVD : origin = 0x0A0FF0, length = 0x0010  /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */


}


SECTIONS
{
    codestart        : > BEGIN

   .text            : >> FLASH_BANK0_SEC_8_15 | FLASH_BANK0_SEC_16_23 | FLASH_BANK0_SEC_24_31, ALIGN(8)

   .cinit           : > FLASH_BANK0_SEC_0_7, ALIGN(8)
   .switch          : > FLASH_BANK0_SEC_0_7, ALIGN(8)

   .reset           : > RESET,  TYPE = DSECT /* not used, */

   .stack           : > RAMM0

   .bss             : > RAMLS0
   .bss:output      : > RAMLS0
   .init_array      : >> FLASH_BANK0_SEC_0_7, ALIGN(8)
   .const           : >> FLASH_BANK0_SEC_32_29, ALIGN(8)
   .data            : > RAMLS0
   .sysmem          : > RAMLS0
  .bss:cio          : > RAMLS0

   GROUP
   {
       .TI.ramfunc {
#if defined(SFRA_ENABLE)
         -l sfra_f32_tmu_eabi.lib<sfra_f32_tmu_collect.obj> (.text)
         -l sfra_f32_tmu_eabi.lib<sfra_f32_tmu_inject.obj> (.text)
#endif
       }
       ramfuncs
       /* Digital Controller Library functions */
       dclfuncs
       dcl32funcs
                      }
                LOAD = FLASH_BANK0_SEC_0_7,
                RUN = RAMLS0,
                LOAD_START(RamfuncsLoadStart),
                LOAD_SIZE(RamfuncsLoadSize),
                LOAD_END(RamfuncsLoadEnd),
                RUN_START(RamfuncsRunStart),
                RUN_SIZE(RamfuncsRunSize),
                RUN_END(RamfuncsRunEnd),
                ALIGN(8)

    ctrlfuncs : {
                }
	          LOAD = FLASH_BANK0_SEC_0_7
              RUN = RAMLS0,
              LOAD_START(ctrlfuncsLoadStart),
              LOAD_SIZE(ctrlfuncsLoadSize),
              LOAD_END(ctrlfuncsLoadEnd),
              RUN_START(ctrlfuncsRunStart),
              RUN_SIZE(ctrlfuncsRunSize),
              RUN_END(ctrlfuncsRunEnd),
              ALIGN(2)

    est_data       : > RAMLS1

    prms_data       : > FLASH_BANK0_SEC_0_7

   GROUP
   {
      user_data
      foc_data
   }
        LOAD = RAMM1
        LOAD_START(ctrlVarsLoadStart),
        LOAD_SIZE(ctrlVarsLoadSize),
        LOAD_END(ctrlVarsLoadEnd)

   GROUP
   {
      sys_data
      motor_data
   }
        LOAD = RAMLS1
        LOAD_START(motorVarsLoadStart),
        LOAD_SIZE(motorVarsLoadSize),
        LOAD_END(motorVarsLoadEnd)

   GROUP
   {
      vibc_data
      datalog_data
      sfra_data
      SFRA_F32_Data
   }
        LOAD = RAMLS1
        LOAD_START(extVarsLoadStart),
        LOAD_SIZE(extVarsLoadSize),
        LOAD_END(extVarsLoadEnd)
}
