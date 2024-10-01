# J1939 DTC Parser Library

## Overview

The **J1939 DTC Parser Library** is a C library designed to parse J1939 Diagnostic Trouble Code (DTC) messages from CAN frames. It processes both single-frame and multi-frame DTC messages using the BAM transport protocol and manages lists of candidate and active DTCs. 

The library is optimized for use in bare-metal microcontroller systems, where it can be called within a CAN interrupt handler. To ensure thread safety, mutex protection is included. If a CAN frame arrives while the DTC list is being accessed, the frame may be skipped to maintain system stability.

## Features

- Parse both single-frame and multi-frame J1939 DTC messages.
- Maintain lists of candidate and active DTCs.
- Handle DTC transitions using a debouncing mechanism.
- Automatic removal of inactive DTCs after a configurable timeout.
- Thread-safe access to DTC lists with built-in mutex protection.
- User-defined callback functions for when active DTCs are updated.

## Project Structure
```shell
.
├── dtc_parser                # Folder containing the library
│   ├── dtc_parser.h          # Header file for the J1939 DTC parser library
│   └── dtc_parser.c          # Source file for the J1939 DTC parser library
├── canalyzer_logs            # Folder containing CANalyzer logs in .ASC format
│   ├── VWConstel2024_1.asc   # Example log file
│   ├── VWConstel2024_2.asc   # Example log file
│   └── ...                   # Other log files
└── test.c                    # Test application for the J1939 DTC parser library
```

## Getting Started

### Prerequisites

- GCC or another C compiler
- CANalyzer log files in `.ASC` format (provided in the `canalyzer_logs` folder)

### Build

To compile and build the test application that uses the J1939 DTC parser library, run the following command:

```bash
gcc -o test test.c dtc_parser/dtc_parser.c
```

After running the command, a a file named `test.exe` will be available to be executed.
