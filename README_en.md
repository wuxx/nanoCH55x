[中文](./README.md) [English](./README_en.md)
## nanoCH55x dev board
CH55x series chips are 8051-core single-chip microcomputers manufactured by Nanjing Qinheng Microelectronics Co., Ltd., including CH551, CH552, CH554, CH559, nanoCH55x is development board made by the MuseLab based on CH552, supports up to 24MHz system frequency, built-in 16K program memory ROM and 256 bytes of internal iRAM and 1K bytes of on-chip xRAM, the xRAM supports DMA direct memory access. CH552 include perpherals such as ADC analog-to-digital conversion, capacitance touch detection, 3 sets of timers and signal capture and PWM, dual asynchronous serial ports, SPI, USB device controller and full-speed transceiver.  
![board-top](https://github.com/wuxx/nanoCH55x/blob/master/doc/board-top.png)
### How to Flash
- install the WCHISPTool_Setup.exe under software directory.  
![wchisptool](https://github.com/wuxx/nanoCH55x/blob/master/doc/wchisptool.png)
- keep the button on-board pressed, plug the board into PC USB port.  
- open the wchisptool, switch to the CH55x tab, select the firmware and click the download  
![how-to-flash](https://github.com/wuxx/nanoCH55x/blob/master/doc/how-to-flash.png)

### Tips
1. The stack of 51 series MCU is extremely small. For CH552, the stack is only 128Bytes, so you must pay attention to the level of control function calls and the definition of local variables when programming. Once the stack overflows, program crashes or other unpredictable consequences will occur.

1. Judging from the official manual, the CH552's ROM can only be programmed about 200 times under 5V power supply due to the use of iFlash technology: it can be said that there are quite a few, which should be the difference between the cost and function considerations of Qinheng design. The trade-off is that in actual products, it is generally only necessary to program the chip once to leave the factory, and subsequent upgrades are rarely required. However, this number is a relatively conservative value given by the manufacturer. The actual test is more than 200 times. However, it is still necessary to pay attention when using it as a development board. It is recommended to make or purchase multiple development boards for development and testing.

### Reference
- http://club.szlcsc.com/article/details_9799_1.html
- https://bbs.21ic.com/icview-2537566-1-1.html?ordertype=2
- http://www.wch.cn/products/CH552.html
