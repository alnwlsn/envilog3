# envilog3
2022 version of my enviromental logger board - original version documented on [Hackaday.io](https://hackaday.io/project/160740-low-power-environment-monitor-logger) or [my website](https://alnwlsn.com/projectrepository/index.php/Electronic_Time_Capsule).

This is a little board that I packed with batteries, some sensors, a Flash chip, and an RTC. When the RTC wakes up the board on some interval (programmable; minutes to an hour), it reads the sensors and stores the data in the Flash chip. The rest of the time, the board is in a low power sleep state, drawing <10uA. With 4xAA cells of power, the whole thing should run for several years.

I added a bunch of sensor footprints to this version because I had space and wasn't sure which ones I'd be able to get in the 2022 chip shortage. But, the firmware provided is for this configuration:
* BME280 (pressure, temperature, humidity)
* SHT31-DIS (temperature, humidity)
* BH1730FVC (light sensor; visible & IR)
* DS3232M (Real-time clock chip, also contains temperature sensor)
* Battery voltage (using Atmega ADC & voltage divider)
* 16MB of Flash; enough for 512K samples.
  * This is about 1 year of data at a 1 minute interval. Scale as needed - I'm going for 6 minute interval --> 6 years. 
