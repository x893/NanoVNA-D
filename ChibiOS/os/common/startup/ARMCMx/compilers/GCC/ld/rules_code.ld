/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

ENTRY(Reset_Handler)

SECTIONS
{
    vectors (READONLY): ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(.vectors))
    } > VECTORS_FLASH AT > VECTORS_FLASH_LMA

    xtors (READONLY): ALIGN(4) SUBALIGN(4)
    {
        __init_array_start = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        __init_array_end = .;
        __fini_array_start = .;
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        __fini_array_end = .;
    } > XTORS_FLASH AT > XTORS_FLASH_LMA

    .text (READONLY): ALIGN(4) SUBALIGN(4)
    {
        *(.text)
        *(.text.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
    } > TEXT_FLASH AT > TEXT_FLASH_LMA

    .rodata (READONLY): ALIGN(4)
    {
        . = ALIGN(4);
        __rodata_base__ = .;
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(4);
        __rodata_end__ = .;
    } > RODATA_FLASH AT > RODATA_FLASH_LMA

    .ARM.extab (READONLY):
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .ARM.exidx (READONLY): {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
     } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .eh_frame_hdr (READONLY):
    {
        *(.eh_frame_hdr)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA

    .eh_frame (READONLY): ONLY_IF_RO
    {
        *(.eh_frame)
    } > VARIOUS_FLASH AT > VARIOUS_FLASH_LMA
}
