# ARM Controller Based Data Logging System

A data acquisition and logging system using ARM Cortex-M0 microcontroller with LabVIEW GUI.

## Project Overview

This project implements a complete data logging system using the NUCLEO-F070RB development board featuring an STM32F070RBT6 microcontroller. The system captures analog data through ADC, processes it through the ARM microcontroller, and displays the results through a custom LabVIEW GUI.

## Features

- Configurable sampling rates (10ms, 100ms, 500ms, 1000ms)
- Real-time data visualization
- User-friendly GUI interface
- Start/stop functionality
- Serial communication between microcontroller and PC

## Hardware Requirements

- NUCLEO-F070RB development board
- Analog signal source (for testing)
- USB cable for programming and communication
- PC for running the LabVIEW GUI

## Software Requirements

- STM32Cube IDE (for microcontroller programming)
- LabVIEW (for GUI)
- Serial terminal (optional for debugging)

## Setup Instructions

1. Connect the NUCLEO-F070RB board to your PC via USB
2. Connect analog input to PA0 pin
3. Open and build the project in STM32Cube IDE
4. Flash the firmware to the microcontroller
5. Open the LabVIEW VI file
6. Select the appropriate COM port in the LabVIEW interface
7. Run the VI and use the GUI to control data acquisition

## Usage

1. Click "Start" to begin data acquisition
2. Select desired sampling rate from the dropdown menu
3. View real-time data on the graph
4. Click "Stop" to end data acquisition

## Code Structure

- `main.c`: Contains the main program logic, ADC configuration, timer setup, and UART communication
- `adc_output()`: Function to transmit ADC values
- `end_logging()`: Function to stop data logging
- LabVIEW VI: Provides GUI interface and handles serial communication with the microcontroller

## Applications

This data logging system can be used in various fields including:
- Environmental monitoring
- Industrial process control
- Research and development
- Educational demonstrations
- Prototype development

## License

This project is provided as-is under standard open-source terms.

## Acknowledgements

Developed at the Institute for Plasma Research as an academic project.
