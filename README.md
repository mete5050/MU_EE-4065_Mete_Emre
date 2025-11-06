Image Preparation:
A 100×100 grayscale image was converted into a C header file (baboon100.h) using a Python script (img2header.py). This file contains a const uint8_t g_img[] array stored in STM32 flash memory.


Firmware Integration:
The header was included in main.c, and UART3 was configured (115200 baud) for transmitting image data in PGM format.


Image Processing:
The STM32 performs four pixel-based transformations sequentially — Negative, Threshold, Gamma correction (γ = 3 and 1/3), and Piecewise linear transformation — each sent every 5 seconds.


Data Reception:
A Python script continuously listens to the serial port, receives each processed PGM image, and saves it as a PNG file for visualization on the PC.

