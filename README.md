# Driver for 24LC256 EEPROM
The goal of this driver was to understand how to use EEPROM in conjunction with an algorithm that provides high memory endurance.

**Index:**
 - [Implementation](https://github.com/UladShumeika/24LC256/tree/main#implementation)
 - [Peculiarities](https://github.com/UladShumeika/24LC256/tree/main#peculiarities)
 - [How to use](https://github.com/UladShumeika/24LC256/tree/main#how-to-use)
 - [Further possible improvements](https://github.com/UladShumeika/24LC256/tree/main#further-possible-improvements)
 - [Additional materials](https://github.com/UladShumeika/24LC256/tree/main#additional-materials)
 
## Implementation
The algorithm of parallel O-buffers described in [[1]](https://github.com/UladShumeika/24LC256/tree/main#additional-materials) was taken as the basis of the algorithm. 

Briefly, the algorithm is implemented as follows:
1) Initializing memory before using it, namely reading system buffers. First, search in the status buffer for the largest record sequence number and calculate the index of the array where this number is stored. And after the calculation through the buffer parameter of the address of the actual data in memory;
2) Based on the address of the actual data and the size of the data packet, the address where the new data will be written is calculated;
3) System buffers are updated, which are stored in RAM, and then written to EEPROM;

In this regard, I divided the memory space into the following parts:
- static data space (one page), data is stored here that is written to memory only once and never changes;
- system status buffer (half a page), stores the number of records in memory;
- buffer system parameter (half a page), stores the memory address where the actual data is stored;
- dynamic data space (remaining space), all data that is overwritten during the operation of the main application is stored here.

## Peculiarities
The main peculiarity is that the recording takes place in packets of a predetermined length. That is, in the application, it is necessary to combine data into a structure, or a buffer, which must be periodically written to memory.
Also, when changing the boundaries of memory spaces, you must adhere to the following restrictions:
- the static space must be located within one page of memory, since the memory has a write limit, no more than one page can be written in one frame, and the algorithm for writing data to the next memory page is not implemented in the functions of writing to the static space.
- system buffers must be of the same length and located within the page, and it doesn't matter both on one or each on its own. This restriction is similar to the static space;
- this driver is based on my I2C driver [[2]](https://github.com/UladShumeika/24LC256/tree/main#additional-materials), but it won't be too hard to add support for the same HAL or something else. It is enough to replace the I2C driver functions and message processing buffers

## How to use
- connect the memory driver and the I2C driver to the project.
- specify in **24lc256.h** which I2C will be used and its speed, as well as, if necessary, a pin for write protection.
- initialize in your application GPIO, DMA, I2C and ***be sure to interrupt both DMA and I2C*** (because when sending data via DMA, synchronization with I2C is required to send a stop signal). After DMA initialization, it is necessary ***to link dma streams and send/receive processing structures*** via I2C by calling the **prj_eeprom_24lc256_dma_handlers_set** function
- specify in **24lc256.h** the length of the message that will be written to memory (for dynamic memory space)

## Further possible improvements
- redesign the I2C driver so that this EEPROM driver only receives a pointer to an I2C instance (such a mechanism is implemented in HAL);
- add the ability to write more than one page to the static space memory;
- expand the static space memory and add a mechanism to prevent data loss in memory with the passage of time
- add updating only changed bytes of system buffers (in this version, buffers are completely overwritten, even if only one byte has been changed);
- add CRC calculation and recording;

## Additional materials
1 - [AVR101: High Endurance EEPROM Storage](https://ww1.microchip.com/downloads/en/Appnotes/doc2526.pdf)

2 - [STM32F4xx_customLibrary](https://github.com/UladShumeika/STM32F4xx_customLibrary)

3 - [256-Kbit I2C Serial EEPROM Data Sheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MPD/ProductDocuments/DataSheets/24AA256-24LC256-24FC256-256-Kbit-I2C-Serial-EEPROM-20001203X.pdf)




