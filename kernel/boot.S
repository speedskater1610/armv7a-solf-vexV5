.section .text
.global _start
_start:
    b _boot

_boot: @ when the user program starts


        @ Use an arduino like system for starting a program.
_setup: 
    b setup 
    
_loop:
    add r0, r0, #1
    b loop
    b _loop
