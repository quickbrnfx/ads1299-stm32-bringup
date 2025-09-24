# ADS1299 + STM32 Bring-up (Public, NDA-Safe)

Personal project to demonstrate embedded bring-up of an ADS1299 EEG/EMG front-end on an STM32F446RE:
- DRDY → EXTI interrupt
- SPI1 DMA (27B frames)
- UART summaries (status + 8 channels)

## Repo layout
- `/firmware` — CubeIDE project (CubeMX .ioc included)
- `/docs` — wiring, notes, roadmaps
- `/tools` — scripts

## Quick start
- Open `/firmware/ads1299_f446re_bringup` in STM32CubeIDE
- Confirm wiring per `/docs/wiring.md`
- Flash; open serial @ 115200; expect 1 Hz summaries

## Roadmap
- Phase 1: capture/visualize bio-signals
- Phase 2: add BLDC control (FOC/trap)
- Phase 3: closed-loop demo with regulatory-minded practices
