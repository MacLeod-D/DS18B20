# DS18B20
High resolution temperature measurement

![Test Image 1](3DTest.png)

 There are a lot of programs around to read temperatures from 1 wire DS1820, DS18B20, DS18S20 devices

 Why another one ?

 1. This program does NOT use any library !
    Libraries are good to get simple access to devices. But they hide what is going on behind the scene. And sometimes
    you want to understand what they are doing.
    Normally Wire- and Dallas-libraries are used.

 2. This program searches for connected one-wire slaves and uses all found DS18(B)20-devices.
    Perfect for measuring temperatures in different roms.
    It will run even if one device breaks !

 3. Normally temperatures are changing slowly compared to the speed of measurements.
    But you always get noise at least of +/- 1 bit.   
    This program shows a simple but efficient method to filter the measured values to get smooth curves and rise the overall resolution.

 3. Most libraries just work with fixed delays (depending on resolution) internaly.
    Here we call a state machine function [ DoJob2() ] while waiting for measurements to complete (about 600 ms for 12 bit resolution!)

    DoJob2() is called about 16000 times per second !
    It is devided in 4 parts - it is possible to call 4 Functions in a round robin fashion.
    It is garanteed, that the maximum delay from one call to another is less than 400 µs.

    Using this simple multitasking most of the CPU time may be used for other jobs than measuring temperatures.

 4. 

    Enjoy it.
