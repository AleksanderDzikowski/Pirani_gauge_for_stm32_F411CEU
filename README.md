Pirani_gauge_for_stm32_F411CEU

## What is it?

  This project is part of my diploma thesis.
  The Pirani sensor is a resistive low pressure sensor and in this project it is implemented in a constant current system. The current source is built on an LM358 operational amplifier that works in a negative feedback configuration. The specific current value for this source is given by specifying the PWM signal on the low-pass filter which averages the signal. This signal is transmitted to the non-inverting input of the amplifier. The signal given to the inverting input is taken from the reference resistor. In this place, voltage is measured by the ADC to determine the actual value of the current flowing in the power track. The maximum current depends on the resistance value of the reference resistor.
  
  The active element of the Pirani sensor is a platinum wire placed in the measuring volume, i.e. the chamber from which the gas is pumped out. With the pressure drop, the efficiency of the heat dissipated from the wire decreases, and thus the resistance of the wire increases. In a constant current system in which the head works, the change in the voltage drop at the head means that the resistance is changed according to the Ohm's law. This decrease is measured by a differential amplifier. The amplifier amplifies the signal and gives it to the ADC input of the microcontroller.
  
  The system is controlled via a computer that communicates with the microcontroller via the UART interface.

## License

This project is licensed under the MIT License
