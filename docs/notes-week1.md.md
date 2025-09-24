# Week 1 Notes

## Goals
- Set up repo for transparency.
- First bring-up of STM32F446RE + ADS1299.
- Verify EXTI → SPI DMA → UART data path.

## Key Learnings
- `RESETB` being held low prevented DRDY from toggling.
- CubeMX GPIO speed settings affect actual toggle frequency (verified).
- SPI Mode-1 (CPOL=0, CPHA=2) required for ADS1299.
- UART printing must not happen in ISRs (moved to main loop).

## Milestone
- Captured first 27-byte ADS1299 frame through SPI DMA.
- Verified DRDY interrupt rate ~250 Hz matches expected sample rate.

## Next Steps
- Structured CSV logging.
- Simple visualization (Python/Excel).
- Begin Phase 2 prep: BLDC control path.
