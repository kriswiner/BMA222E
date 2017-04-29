# BMA222E

Bosch's small 8-bit accelerometer useful for gesture control and tap detection. Simple sketch to configure the accelerometer, correct for offset bias, get scaled accelerations, and detect single and double taps. Interrupt 1 configured to detect data ready. Interrupt 2 is configured to detect taps. Interrupts can also be configured to detect hi-g or low-g motion, free fall, slope, orientation (landscape or portrait). This is a pretty versatile sensor that can even be configured to interrupt even when not connected to an MCU.
