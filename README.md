# thisisfine
dust

just no
this is hell
help me


# FunFacts
* BT_RX (PA2 or PA3, depending if swapped or not), is... to slow? for 115200; shoots USART2_IRQHandler like it's nothing and loses bytes n shit (lost a day of work due to this); but for 57600 it works fine. That's why it's at only 56k. sending at 115200 is ok though (...),
* DHT11 sensor doesn't reply if the line is held low at all times (so it has to be up -> down (time) -> up -> switch to input),
* MQ-7 requires calibration. Calibration requires higher quality equipment, which I don't have. It also should be done on per-module basis, so I'll leave it to the smartphone app to interpret the results (makes sense, reprogramming the STM is not as easy as changing a simple setting).
