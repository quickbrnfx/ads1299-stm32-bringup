# Week 1 Notes

## Goals
- Bring up ADS1299; prove DRDY-paced frames.
- Build firmware I/O path: EXTI (DRDY) → SPI DMA (27 bytes) → UART print.

## Key decisions
- SPI Mode-1 (CPOL=0, CPHA=2nd edge) to match ADS1299 timing.
- Software-controlled CS (PA4).
- EXTI on PB0 (falling) to sync with DRDY low.
- DMA TxRx with zero buffer to guarantee SCK clocks.

## Lessons learned
- RESETB held low = no DRDY. Releasing RESETB (PB1) fixed it.
- START high + RDATAC required for continuous frames.
- Printing in ISRs is fragile; throttle prints from main loop.

## Next
- CSV logging per frame.
- Python live plot.
- Circular DMA or double buffering.
