This project uses ESP-IDF v6.1. You need to install ESP-IDF to be able to build/flash.

# COMMON COMMANDS

In the ESP-IDF environment:

Build project: `idf.py build`

Flash project to board: `idf.py flash` (this also builds if not up to date)

View serial communicaiton from esp: `idf.py monitor` (to exit monitor use `ctrl` + `]`)

Configure project settings (partitions, compiler options, ect) `idf.py menuconfig`

Generate project binary `idf.py dfu`

# FLASHING BOARD

Three primary methods for flashing the board.

## USB JTAG
Preferred for first time programming and debug on custom PCB.
Plug into the native USB port on the devkit.

As long as the device enumerates as Espressif USB JTAG/serial debug unit, you can just use the normal `idf.py flash` and `idf.py monitor`

## USB-UART bridge - not preferred
If using a devkit with the UART usb port, then you can use the `idf.py flash` and `idf.py monitor` commands.

if idf.py cannot determine which port to use, you can call out `idf.py flash -p COM3` as an example.

## USB DFU
Not suitable for first time programming since you need to burn a fuse for this to work. We may decide against ever using this method.
[Online Guide](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32s3/api-guides/dfu.html)

### 1 Build binary
You first need to build the project binary `idf.py dfu`.

### 2 Enter USB bootloader mode
Pull GPIO0 low (press and hold the Boot button). Cycle the EN/RESET pin (press and release Reset while keeping Boot pressed, or toggle power). Release GPIO0. You are now in boot mode.

### 3 Flash
Run `idf.py dfu-flash`. If this doesn't work, you may need to set up dfu-util on your computer. See the guide page link.


# SETUP ENVIRONMENT

[Download ESP-IDF here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#installation). Installing ESP-IDF requires the EIM or EIM-CLI which will install ESP-IDF. Please be sure to install ESP-IDF v6.1.


Anytime you want to use ESP-IDF, you must first activate it. The path or command to activate it depends on your system, so it is recomended to create your own

activate_idf.sh
or
activate_idf.cmd

shortcut in this directory, which can activate your ESP-IDF environment quickly. For linux, once setup run `source activate_idf.sh`. On windows, simply run `activate_idf.cmd` once you have setup your script.

VSCode - you may get squiggly lines on certain `#include`. Just give the following prompt to an LLM to fix:

```
I have squiggly lines on my #include statements for my ESP-IDF project. I just want to point VSCode to my ESP-IDF components folder with a hard path. Please generate the appropriate .vscode/c_cpp_properties.json for my OS. I already ran echo $IDF_PATH and it returned...
```

You also may need to run `idf.py reconfigure`