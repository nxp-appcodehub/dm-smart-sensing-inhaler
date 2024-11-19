/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2024 NXP
 * Generated linker script file for MCXW716CxxxA
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v11.10.0 [Build 3148] [2024-07-03] on Nov 19, 2024, 2:27:53 PM
 */

MEMORY
{
  /* Define each memory region */
  PROGRAM_FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x100000 /* 1M bytes (alias Flash) */  
  TCM_SYS (rwx) : ORIGIN = 0x20000000, LENGTH = 0x1c000 /* 112K bytes (alias RAM) */  
  TCM_CODE (rwx) : ORIGIN = 0x4000000, LENGTH = 0x4000 /* 16K bytes (alias RAM2) */  
}

  /* Define a symbol for the top of each memory region */
  __base_PROGRAM_FLASH = 0x0  ; /* PROGRAM_FLASH */  
  __base_Flash = 0x0 ; /* Flash */  
  __top_PROGRAM_FLASH = 0x0 + 0x100000 ; /* 1M bytes */  
  __top_Flash = 0x0 + 0x100000 ; /* 1M bytes */  
  __base_TCM_SYS = 0x20000000  ; /* TCM_SYS */  
  __base_RAM = 0x20000000 ; /* RAM */  
  __top_TCM_SYS = 0x20000000 + 0x1c000 ; /* 112K bytes */  
  __top_RAM = 0x20000000 + 0x1c000 ; /* 112K bytes */  
  __base_TCM_CODE = 0x4000000  ; /* TCM_CODE */  
  __base_RAM2 = 0x4000000 ; /* RAM2 */  
  __top_TCM_CODE = 0x4000000 + 0x4000 ; /* 16K bytes */  
  __top_RAM2 = 0x4000000 + 0x4000 ; /* 16K bytes */  
