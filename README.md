# Anjoman Firmware - Motion System Branch

This branch (`feat/motion-system`) focuses on the low-level implementation of the motion system for the **Anjoman** robots.

## Current Status
This branch includes:
*   **Motor Drivers:** H-Bridge control logic (DRV8833).
*   **Odometry:** Absolute Magnetic Encoder drivers (AS5600 over I2C).
*   **System Identification:** Calibration scripts in `examples/` to calculate motor transfer functions and gearbox ratio.

## Next Steps
This code will be merged into the `main` branch to serve as the foundation for the IMU and control logic implementation.
