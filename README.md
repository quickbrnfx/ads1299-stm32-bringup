# ADS1299 + STM32 Bring-Up

This project is my personal exploration of embedded systems, outside of my professional work.  
The goal: bring up an **ADS1299 EEG/EMG front-end** on an **STM32F446RE**, document the process, and build toward a closed-loop system.

## Roadmap
- **Phase 1** → Bring up ADS1299, capture and visualize bio-signals  
- **Phase 2** → Add BLDC motor control (FOC/trapezoidal) alongside streaming  
- **Phase 3** → Closed loop (sensing + actuation), incorporating regulatory and system resilience practices

## Repo Layout
- `/firmware` — STM32CubeIDE project (includes `.ioc`)  
- `/docs` — wiring, notes, lessons learned  
- `/tools` — scripts for data logging and visualization (future)

## Quick Start
1. Open CubeIDE project in `/firmware`  
2. Confirm wiring per `/docs/wiring.md`  
3. Flash to Nucleo-F446RE  
4. Open serial terminal at 115200 → live frames from ADS1299

