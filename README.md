# Game Boy Link communication, stm32f4 side

This is the microcontroller side of a project that allows communication between a physical Game Boy Game Link Cable and a computer.

The code is written in C and uses the [libopencm3](https://github.com/libopencm3/libopencm3) library to control the STM32F4.  I have used a NUCLEO-F411RE, but other STM32 microcontroller should work too.

Three main modes for the serial protocol are implemented:

- Sniffing
- Master
- Slave

## Details

You can read the complete details of this project in my blog posts:

- [Sniffing Game Boy serial traffic with an STM32F4](https://dhole.github.io/post/gameboy_serial_1/)
- [Virtual Game Boy Printer with an STM32F4](https://dhole.github.io/post/gameboy_serial_2/)
- [Printing on the Game Boy Printer using an STM32F4](https://dhole.github.io/post/gameboy_serial_3/)


## License

The code is released with the 3-clause BSD License.
