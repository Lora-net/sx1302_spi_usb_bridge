	  ______                              _
	 / _____)             _              | |
	( (____  _____ ____ _| |_ _____  ____| |__
	 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
	 _____) ) ____| | | || |_| ____( (___| | | |
	(______/|_____)_|_|_| \__)_____)\____)_| |_|
	  (C)2021 Semtech

Communication protocol between the USB-SPI bridge and the Host
==============================================================


## 1. Introduction

This document describes the communication protocol used by the Host to
communicate with the USB-SPI bridge.

It describes the commands that can be sent from the Host to the bridge and the
expected acknoledge answers that the host will get.

The communication protocol is synchronous. This means that when the host sends
a command/request, it has to read the acknoledge answer before sending another
command.

## 2. Command frame format

A command is formatted as follows (byte oper byte):

| Command ID | Size MSB | Size LSB | Command | Param 1 | ... | Param N |

where the command header is:
* Command ID (1 byte): a random command identifier token
* Size MSB (1 byte): Most Significant Bits of the 16-bits command size
* Size LSB (1 byte): Least Significant Bits of the 16-bits command size
* Command (1 byte): The command to be issued to the bridge

The available commands (4th byte in the header) are:
* ORDER_ID__REQ_PING            : 0x00
* ORDER_ID__REQ_GET_STATUS      : 0x01
* ORDER_ID__REQ_BOOTLOADER_MODE : 0x02
* ORDER_ID__REQ_RESET           : 0x03
* ORDER_ID__REQ_WRITE_GPIO      : 0x04
* ORDER_ID__REQ_MULTIPLE_SPI    : 0x05

Each command also has its corresponding ACK answer:
* ORDER_ID__ACK_PING            : 0x40
* ORDER_ID__ACK_GET_STATUS      : 0x41
* ORDER_ID__ACK_BOOTLOADER_MODE : 0x42
* ORDER_ID__ACK_RESET           : 0x43
* ORDER_ID__ACK_WRITE_GPIO      : 0x44
* ORDER_ID__ACK_MULTIPLE_SPI    : 0x45

## 3. REQ_PING / ACK_PING

The REQ_PING command has no parameter. Only the command header has to be sent,
and information about the bridge MCU and firmware will be returned in the
ACK_PING answer.

### 3.1. REQ_PING frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | random command ID [0..255]
 1-2    | 0x0000
 3      | 0x00 (ORDER_ID__REQ_PING)

### 3.2. ACK_PING frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | command ID (same as REQ)
 1-2    | ACK_PING_SIZE
 3      | 0x40 (ORDER_ID__ACK_PING)
 4-15   | Bridge MCU unique ID
 16-24  | Bridge firmware version string "Vxx.xx.xx"

## 4. REQ_GET_STATUS / ACK_GET_STATUS

The REG_GET_STATUS command can be called at any time in order to get the current
temperature from the Corecell sensor, and the current MCU system time. This
command has no parameter, only the command header has to be sent.

### 4.1. REQ_GET_STATUS frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | random command ID [0..255]
 1-2    | 0x0000
 3      | 0x01 (ORDER_ID__REQ_GET_STATUS)

### 4.2. ACK_GET_STATUS frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | command ID (same as REQ)
 1-2    | ACK_GET_STATUS_SIZE
 3      | 0x41 (ORDER_ID__ACK_GET_STATUS)
 4-7    | Current system time
 8-9    | Current temperature in °C multiplied by 100

## 5. REQ_BOOTLOADER_MODE / ACK_BOOTLOADER_MODE

The REQ_BOOTLOADER_MODE command can be called to switch the bridge MCU in DFU
bootloader mode, in order to program a new firmware. No parameter is required,
only the command header has to be sent. Receiving the ACK_BOOTLOADER_MODE answer
will confirm that the request has been taken into account.

### 5.1. REQ_BOOTLOADER_MODE frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | random command ID [0..255]
 1-2    | 0x0000
 3      | 0x02 (ORDER_ID__REQ_BOOTLOADER_MODE)

### 5.2. ACK_BOOTLOADER_MODE frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | random command ID [0..255]
 1-2    | 0x0000
 3      | 0x42 (ORDER_ID__ACK_BOOTLOADER_MODE)

## 6. REQ_RESET / ACK_RESET

The REQ_RESET command can be used in order to trigger the MCU to reset. This
command takes 1 byte as parameter for the reset type. Only RESET_TYPE__GTW (0x00)
is supported for now.
The ACK_RESET will contain 1 byte for the reset status.

### 6.1. REQ_RESET frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | random command ID [0..255]
 1-2    | REQ_RESET_SIZE
 3      | 0x03 (ORDER_ID__REQ_RESET)
 4      | 0x00 (RESET_TYPE__GTW)

### 6.2. ACK_RESET frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | random command ID [0..255]
 1-2    | ACK_RESET_SIZE
 3      | 0x43 (ORDER_ID__ACK_RESET)
 4      | Reset status (0: reset done, 1: reset failed)

## 7. REQ_WRITE_GPIO / ACK_WRITE_GPIO

The REQ_WRITE_GPIO command can be used to set a GPIO of the Corecell to a
particular value. It will typically be used by the host to set the POWER_EN,
and set the sx1302 and sx1261 out from reset state.
The ACK_WRITE_GPIO answer will inform about the write status.

### 7.1. REQ_WRITE_GPIO frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | random command ID [0..255]
 1-2    | REQ_WRITE_GPIO_SIZE
 3      | 0x04 (ORDER_ID__REQ_WRITE_GPIO)
 4      | GPIO port
 5      | GPIO pin
 6      | GPIO state

### 7.2. ACK_WRITE_GPIO frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | random command ID [0..255]
 1-2    | ACK_GPIO_WRITE_SIZE
 3      | 0x44 (ORDER_ID__ACK_WRITE_GPIO)
 4      | Write status (0: success, 1: failed)

## 8. REQ_MULTIPLE_SPI / ACK_MULTIPLE_SPI

The REQ_MULTIPLE_SPI command can be used to send multiple SPI transfer requests
with only one USB request. The maximum size for the complete command is 4200
bytes (including command header).

### 8.1. REQ_MULTIPLE_SPI frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | random command ID [0..255]
 1-2    | SPI request(s) buffer size
 3      | 0x05 (ORDER_ID__REQ_MULTIPLE_SPI)
 4-N    | SPI request frames

The REQ_MULTIPLE_SPI command high level payload will look like:

| SPI req 1 | SPI req 2 | SPI req 3 | ... | SPI req N |

With each SPI request being formatted as described below.

#### 8.1.1. SPI request frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | SPI request index [1..255]
 1      | SPI request type (0x01: read/write, 0x02: read-modify-write)
 2-N    | SPI request payload

The SPI request payload is different depending on the SPI request type.

#### 8.1.2. read/write SPI request frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | SPI target (0x00: sx1302, 0x01: sx1261)
 1-2    | SPI request size
 3-N    | SPI request raw payload, formatted as required by sx1302 or sx1261

#### 8.1.3. read-modify-write SPI request frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0-1    | Register address
 2      | Register bitmask
 3      | Data to write

### 8.2. ACK_MULTIPLE_SPI frame

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | random command ID [0..255]
 1-2    | SPI request ACKs buffer size
 3      | 0x45 (ORDER_ID__ACK_MULTIPLE_SPI)
 4-N    | ACK frames

 All SPI transactions will be acknowledged atfer the last SPI transaction is
 completed. It will use the SPI request index used in the request to identify
 each transaction.

The ACK_MULTIPLE_SPI command payload will look like:

| ACK req 1 | ACK req 2 | ACK req 3 | ... | ACK req N |

#### 8.2.1. ACK request frame

Each request acknowledge is formatted as follows:

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | SPI request index
 1      | SPI request type (0x01: read/write, 0x02: read-modify-write)
 2      | SPI request status (0: success, 1: failed, 2: wrong parameter)
 3-N    | ACK payload

The format of the ACK payload is different depending on the SPI request type.

#### 8.2.2. ACK payload for read/write request

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0-1    | payload size
 2-N    | payload as returned on the SPI bus

#### 8.2.3. ACK payload for read-modify-write request

If the ACK status is OK, the ACK payload will be:

 Bytes  | Function
:------:|---------------------------------------------------------------------
 0      | read value
 1      | modified value

If the ACK status is Failed or Wrong Parameter, there is no payload.