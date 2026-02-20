# ARMv7-Compatible Soft CPU - Cmod S7 + ESP32 Memory Controller + Cmod S6 IO Controller
## Project Overview

A 3-stage pipelined ARMv7-compatible processor implemented on the **Digilent Cmod S7-25**
FPGA, using an **ESP32 as external memory**. The CPU executes ARMv7-A instructions with
correct output behavior (ISA-accurate, not microarchitecture-accurate).

---

## Architecture

```
┌─────────────────────────────────────────────┐
│              Cmod S7-25 FPGA  (CPU1)        │
│                                             │
│  ┌──────────┐    ┌──────────┐               │
│  │  FETCH   │───>│  DECODE  │               │
│  │  Stage   │    │  Stage   │               │
│  └──────────┘    └───┬──────┘               │
│       ▲              │                      │
│       │         ┌────▼──────┐               │
│       │         │  EXECUTE  │               │
│       │         │  Stage    │               │
│       │         └────┬──────┘               │
│       │              │                      │
│  ┌────┴──────────────▼────┐        ┌─────┐  │
│  │      MEM BUS           │        | GND |  │
│  │  (handshake FSM)       │        └─────┘  │
│  └───────────────────────┬┘           |     │────┐
│                          │            |     │    |
└──────────────────────────┼────────────┼─────┘    |
                           │            |          |
                           │            |          |
┌──────────────────────────┼────────────┼─────┐    |
│         ESP32            │            |     │    |  ┌──────┐
│                          │            |     │    |  | UART |
|                          |            |     |    |  | RxTx |
│  ┌───────────────────────┴────┐    ┌─────┐  │    |  └──────┘
│  │  Memory Controller         │    | GND |  │    |
│  │  (65536 × 32-bit words)    │    └─────┘  │    |
│  │  = 256KB address space     │       |     │    |
│  └───────────────────────┼────┘       |     │    |
└──────────────────────────┼────────────|─────┘    |
                           |            |          |
┌──────────────────────────┼────────────┼─────┐    |
│         Cmod-S6          └─────┐      |     │    | 
│         ┌─────────────────┐    |      |     │    |  
│         |   Connects to   |    |      |     │    | 
│         |  all of the IO  |    |   ┌─────┐  |    |
|         |                 |    |   | GND |  │    |
│         |(Screen included)|    |   └─────┘  │    |
│         |                 |    |            │    |
│         └─────────────────┘────|────────────|────┘
└─────┼┼─────────────────────────┼────────────┘
      ||  I2C                    |
┌─────┼┼─────────────┐───────────┘ 80 bytes of memory for the screen from 0x0
|                    |
|      2004 LCD      |
|                    |
|                    |
└────────────────────┘
```

### Memory map
| Start Address | End Address | Size (words) | Description |
| --- | --- | --- | ---  |
| 0x00000000    | 0x0003FFFF  | 256000       | This is the entire raw memory of the device |
| 0x00000000    | 0x00000050  | 80           | This is all of the data for the screen. |
| 0x0x00000051  | 0x | | Area of memeory dedicated to metadata of devices on the devicce |


### Pipeline Stages
| Stage   | Description                                      |
|---------|--------------------------------------------------|
| FETCH   | Puts word address on bus, waits for ESP32 ACK   |
| DECODE  | Decodes instruction, reads registers, checks cond|
| EXECUTE | ALU op / branch / load-store, writes result back |

Memory accesses (LDR/STR) stall the pipeline until ESP32 responds.
Branches flush 2 pipeline stages (ARM PC+8 convention).

---

## Device structure
1 word for type of device (enum like)
1 word for the port it is connected to
1 word for the the 

---
<!--
## File Structure

```
armv7_cmod_s7/
├── rtl/
│   ├── top.v           ← Top-level module, pin assignments
│   ├── cpu_core.v      ← 3-stage pipeline, main FSM
│   ├── decoder.v       ← ARMv7 instruction decoder, condition codes
│   ├── alu.v           ← ALU + barrel shifter (all ARMv7 ops + NZCV)
│   ├── reg_file.v      ← R0–R15 register file (R15=PC)
│   ├── mem_bus.v       ← External bus FSM (handshake with ESP32)
│   └── cpsr.v          ← CPSR register (N,Z,C,V,T,M,I,F bits)
├── constraints/
│   └── cmod_s7_armv7.xdc ← Vivado pin constraints
├── esp32_memory.ino    ← ESP32 Arduino sketch (memory controller)
└── README.md           ← This file
```

---
-->

## Pin Wiring — FPGA ↔ ESP32

### Address Bus (FPGA → ESP32, 16 pins)
| FPGA Pin | PIO# | Addr Bit | Suggested ESP32 GPIO |
|----------|------|----------|----------------------|
| L1       | 1    | addr[0]  | 34                   |
| M4       | 2    | addr[1]  | 35                   |
| M3       | 3    | addr[2]  | 32                   |
| N2       | 4    | addr[3]  | 33                   |
| M2       | 5    | addr[4]  | 25                   |
| P3       | 6    | addr[5]  | 26                   |
| N3       | 7    | addr[6]  | 27                   |
| P1       | 8    | addr[7]  | 14                   |
| N1       | 9    | addr[8]  | 12                   |
| P14      | 16   | addr[9]  | 13                   |
| P15      | 17   | addr[10] | 15                   |
| N13      | 18   | addr[11] | 2                    |
| N15      | 19   | addr[12] | 0                    |
| N14      | 20   | addr[13] | 4                    |
| M15      | 21   | addr[14] | 16                   |
| M14      | 22   | addr[15] | 17                   |

### Data Bus (bidirectional, 32 pins)
> **Note:** The ESP32 has ~30 usable GPIOs. For full 32-bit data, use a GPIO
> expander (MCP23017 via I2C) for the upper 16 bits, or start with 16-bit data
> and zero-extend the upper half. The RTL supports both.

| FPGA Pin | PIO# | Data Bit |
|----------|------|----------|
| L15      | 23   | data[0]  |
| L14      | 26   | data[1]  |
| K14      | 27   | data[2]  |
| J15      | 28   | data[3]  |
| L13      | 29   | data[4]  |
| M13      | 30   | data[5]  |
| J11      | 31   | data[6]  |
| C5       | 40   | data[7]  |
| A2       | 41   | data[8]  |
| B2       | 42   | data[9]  |
| B1       | 43   | data[10] |
| C1       | 44   | data[11] |
| B3       | 45   | data[12] |
| B4       | 46   | data[13] |
| A3       | 47   | data[14] |
| A4       | 48   | data[15] |

### Handshake (PMOD JA)
| FPGA Pin | PMOD | Signal    | Direction    | ESP32 GPIO |
|----------|------|-----------|--------------|------------|
| J2       | JA1  | MEM_REQ   | FPGA → ESP32 | 6          |
| H2       | JA2  | MEM_ACK   | ESP32 → FPGA | 7          |
| H4       | JA3  | MEM_WR    | FPGA → ESP32 | 8          |
| F3       | JA4  | DIR_CTRL  | FPGA → ESP32 | 9          |

### UART Debug
| FPGA Pin | Signal  | Connect to           |
|----------|---------|----------------------|
| L12      | uart_tx | USB-UART adapter RX  |

> Cmod S7 has onboard USB-UART — connect with a terminal at **115200 8N1**.
> Output: `PC=00000000 FX` where X = NZCV nibble, printed every ~65536 cycles.

---

## Bus Protocol Timing

```
FPGA CLK    ─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─
             └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘

ADDR        ──╔═══════════════════════════╗─────
              ║  valid address            ║

MEM_REQ     ────────┐                    ┌──────
                    └────────────────────┘
                    (FPGA asserts)

MEM_ACK     ──────────────────────┐   ┌────────
                                  └───┘
                            (ESP32 responds)

DATA        ──────────────────╔══╗─────────────
                              ║ valid data      ║
                              (ESP32 drives)

DIR_CTRL    LOW (read) ────────────────────────
```

**Read timing (ESP32 memory latency):**
- At 12MHz, each FPGA clock = 83ns
- ESP32 `digitalRead` x16 + `digitalWrite` x32 ≈ 2–5μs typical
- This gives ~25–60 stall cycles per memory access
- Effective execution rate: ~200K–480K instructions/sec
- To improve: use ESP32 SPI slave mode for faster transfers

---

## Building in Vivado

1. Create a new RTL project targeting `xc7s25csga225-1`
2. Add all `.v` files from `rtl/` as design sources
3. Add `constraints/cmod_s7_armv7.xdc` as a constraint
4. Set `top` as the top module
5. Run Synthesis -> Implementation -> Generate Bitstream -> Hardware Manager to upload.
6. Expected resource usage:
   - LUTs: ~4,000–6,000 (25–35% of S7-25)
   - FFs:  ~1,500–2,500
   - BRAM: 0 (memory is external)

### Bidirectional Data Pins (pretty important)
Vivado does not allow the same pin to be both `input` and `output` ports.
Replace the data port split in `top.v` with `IOBUF` primitives:

```verilog
genvar i;
generate
  for (i = 0; i < 16; i = i+1) begin : data_iobuf
    IOBUF #(.DRIVE(12), .SLEW("SLOW")) data_buf (
      .O  (bus_data_in[i]),     // to FPGA fabric (read)
      .IO (data_pin[i]),        // physical pin
      .I  (bus_data_out[i]),    // from FPGA fabric (write)
      .T  (~bus_data_dir)       // T=1 → high-Z (input mode)
    );
  end
endgenerate
```

---

## Loading Your ARM Program

### Toolchain setup
```bash
# Aarch based OS (like me)
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib arm-none-eabi-binutils

# Ubuntu/Debian
sudo apt install gcc-arm-none-eabi

# macOS
brew install arm-none-eabi-gcc
```

### Minimal bare-metal example (test.S)
```asm
.section .text
.global _start
_start:
    mov r0, #0          @ counter
loop:
    add r0, r0, #1      @ increment
    b   loop            @ loop forever
```

### Link script (link.ld)
This is just a super simple bare metal linker script just so all of the instructions are in the correct format.
```
SECTIONS {
    . = 0x00000000;
    .text : { *(.text) }
    .data : { *(.data) }
    .bss  : { *(.bss)  }
}
```

### Build and convert
```bash
arm-none-eabi-as -mcpu=cortex-a9 test.S -o test.o
arm-none-eabi-ld -T link.ld test.o -o test.elf
arm-none-eabi-objcopy -O binary test.elf test.bin

# Convert to C array for ESP32
xxd -i test.bin > program_data.h
```

### Load into ESP32
In `esp32_memory.ino`, replace `load_test_program()` with:
```cpp
#include "program_data.h"   // generated by xxd
void load_test_program() {
    memset(flash_mem, 0, sizeof(flash_mem));
    memcpy(flash_mem, test_bin, test_bin_len);
}
```

---

## Supported Instructions

| Class              | Instructions                                      | Status |
|--------------------|---------------------------------------------------|--------|
| Data Processing    | AND, EOR, SUB, RSB, ADD, ADC, SBC, RSC, ORR, BIC | YES |
| Move               | MOV, MVN                                          | YES |
| Compare            | TST, TEQ, CMP, CMN                                | YES |
| Branch             | B, BL, BX                                         | YES |
| Load/Store         | LDR, STR, LDRB, STRB (pre/post-index)            | YES |
| Load/Store Multiple| LDM, STM                                          | YES |
| Multiply           | MUL, MLA                                          | YES |
| Status Register    | MRS, MSR                                          | YES |
| Condition Codes    | EQ,NE,CS,CC,MI,PL,VS,VC,HI,LS,GE,LT,GT,LE,AL    | YES |
| Barrel Shifter     | LSL, LSR, ASR, ROR, RRX                           | YES |
| Long Multiply      | UMULL, SMULL, UMLAL, SMLAL                        | NO (NOP)|
| VFP/NEON           | Floating point                                    | NO     |
| Thumb              | 16-bit Thumb instructions                         | NO     |
| CP15               | Cache/MMU control                                 | NO     |

---

## CPSR Flags

| Bit | Flag | Meaning                    |
|-----|------|----------------------------|
| 31  | N    | Negative result            |
| 30  | Z    | Zero result                |
| 29  | C    | Carry out / borrow         |
| 28  | V    | Overflow (signed)          |
| 7   | I    | IRQ disable (set at reset) |
| 6   | F    | FIQ disable (set at reset) |
| 5   | T    | Thumb mode (always 0 here) |
| 4:0 | M   | Processor mode (SVC=10011) |

---

## Address Space

| Word Address | Byte Address      | Suggested Use         |
|--------------|-------------------|-----------------------|
| 0x0000–0x0FFF| 0x00000–0x0FFFF  | Code (16KB)           |
| 0x1000–0x1FFF| 0x10000–0x1FFFF  | Data (16KB)           |
| 0x2000–0x3FFF| 0x20000–0x3FFFF  | Stack (grows down)    |
| 0x4000–0xFFFF| 0x40000–0xFFFFF  | Extended / future use |

Initial SP: `0x0003FFFC`

---

## Debug Output (UART)

Connect to Cmod S7 USB at 115200 baud. Output example:
```
PC=00000000 F0
PC=00000004 F0
PC=00000008 F1     ← Z flag set
PC=00000004 F1
```
`F` nibble = `NZCV` (bit3=N, bit2=Z, bit1=C, bit0=V)

---

## Known Limitations

1. **No Thumb support** — assemble with `-marm` flag
2. **16-bit address bus** — 256KB max address space (word-addressed)
3. **Memory latency** — each access takes ~25–60 cycles (ESP32 GPIO speed)
4. **No interrupts** — IRQ/FIQ inputs not connected
5. **No cache** — every instruction fetch hits the ESP32
6. **Upper 16 data bits** — require GPIO expander for full 32-bit (see wiring note)
