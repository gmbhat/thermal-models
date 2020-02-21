# Thermal models for Odroid XU3

The cpufreq_ondemand.c file is integrated into to the Linux kernel to perform runtime prediction of the temperature.
Specifically, the function "predict_temp_for_1s()" is used to perform the prediction. Note that all the values are scaled, as floating point arithmetic is not available in the kernel. 

Please use this file as a reference implementation.

Please refer to the following papers for additional information:

Ganapati Bhat, Suat Gumussoy, and Umit Y. Ogras. "Power-temperature stability and safety analysis for multiprocessor systems." ACM Transactions on Embedded Computing Systems (TECS) 16, no. 5s (2017): 145.

Ganapati Bhat, Gaurav Singla, Ali K. Unver, and Umit Y. Ogras. "Algorithmic optimization of thermal and power management for heterogeneous mobile platforms." IEEE Transactions on Very Large Scale Integration (VLSI) Systems 26, no. 3 (2018): 544-557.

