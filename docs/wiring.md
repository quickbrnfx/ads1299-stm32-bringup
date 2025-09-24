# ADS1299 ↔ STM32 Wiring (Nucleo-F446RE)

Power: MMB0 supplies analog/digital. *Do not* back-feed 3V3 from Nucleo. Common GND required.

| Signal  | ADS1299 FE Header | STM32 Pin | Notes |
|---------|--------------------|-----------|-------|
| CS̄     | J3-7               | PA4       | JP21 = 2–3 (external CS) |
| DRDY    | J3-15              | PB0       | EXTI, falling edge, Pull-Up |
| START   | J3-14              | PB2       | JP22 = 2–3 |
| RESETB  | J3-8               | PB1       | Active-low reset, keep high |
| SCK     | —                  | PA5 (AF5) | SPI1 SCK |
| MISO    | —                  | PA6 (AF5) | SPI1 MISO |
| MOSI    | —                  | PA7 (AF5) | SPI1 MOSI |
| UART TX | —                  | PA2       | 115200 |
| UART RX | —                  | PA3       | 115200 |

**Jumpers:** JP21=2–3, JP22=2–3.
