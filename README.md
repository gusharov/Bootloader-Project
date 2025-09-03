# STM32 Bootloader with Shared API Memory

## Overview
This project demonstrates a minimal STM32 bootloader that can hand control to an application firmware while sharing a dedicated memory region (API) accessible to both the bootloader and the application. The shared API is designed to allow safe communication or command passing between the two programs.

---

## Features
- Bootloader runs on reset and indicates activity via LED blink.  
- Jump from bootloader to application firmware using a function pointer.  
- Shared memory section (`API_SHARED`) accessible to both bootloader and application.  
- Foundation laid for future features such as firmware updates via UART or other interfaces.  

---

## Hardware
- STM32 microcontroller (tested on STM32F4)  
- Onboard LED for bootloader status indication
  
---

## Next Steps / Stretch Goals
- Implement UART-based firmware update handled by the bootloader.  
- Add CRC or checksum verification for application images.  
