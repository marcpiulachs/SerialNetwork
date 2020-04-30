// Uncomment this macro to deactivate debugging output on serial port.
#define DEBUG_NETWORK 0

#pragma once

#ifndef SERIAL_NETWORK_H
#define SERIAL_NETWORK_H

#include <Arduino.h>

/*
// This defines the maximum size of any message that can be sent
// between chips.  Reduce this to conserve memory at the cost
// of smaller packet sizes.  Note that this only affects the
// size of packets that can be received - you can always send
// up to 255 bytes.  If the remote end can't receive them all
// the packet will be silently discarded.
#define MAX_MESSAGE 250
*/

// The maximum number of registered commands.  If you're not
// going to have many commands reducing this can save memory.
// If you want lots of commands you may need to increase this
// value.  Note that this ony affects the commands registerable
// at the receiving end
#define MAX_COMMANDS 15

//Data buffer is predefined, if you need more than 10bytes of data per packet, increase this
//#define DATA_BUFFER_SIZE 10

/* The expected message length */
#define PROTOCOL_MESSAGE_BYTES 8

#define DEVICE_ALL_DEVICES 0x00
#define DEVICE_WATERING_STATION  0x01
#define DEVICE_LOCKER_CABINET  0x02

#define NO_CONTROL 0

#ifdef DEBUG_NETWORK
#define DEBUG_NETWORK_PRINT(x)   Serial.print (x)
#define DEBUG_NETWORK_PRINTLN(x) Serial.println (x)
#else
#define DEBUG_NETWORK_PRINT(x)
#define DEBUG_NETWORK_PRINTLN(x)
#endif

typedef struct _STATS
{
    unsigned long rx_packets;
    unsigned long rx_bytes;
    unsigned long tx_packets;
    unsigned long tx_bytes;
    unsigned long cmd_run;
    unsigned long cmd_not_found;
    unsigned long msg_crc_error;
    unsigned long msg_bad_formatted;
    unsigned long msg_discarded_packet;
    unsigned long msg_discarded_byte;
} STATS, *PSTATS;

typedef struct _PACKAGE
{
	byte stx; // Start of transmission
	byte deviceType; // The device type this message is addressed.
	byte sender; // Receiver device address.
	byte receiver; // Receiver device address.
	byte command; // The command to be executed
	byte commandType; // The type of package
	byte argument; // Optional parameter, NULL if empty.
	byte crc; // CRC check
	byte etx; // End of transmission
	bool isBroadcast();
} PACKAGE, *PPACKAGE;

// Format of command callback functions
typedef void(*callbackFunction)(PPACKAGE);

// Network protocol defines
#define STX                 (byte)0x02
#define ETX                 (byte)0x03
#define NULL_VAR            (byte)0x00

//When this is used during registerCommand all message will pushed the callback function
#define ALL_ADDRESSES       (byte)0x00
#define ALL_COMMANDS        (byte)0x00
#define ALL_DEVICES         (byte)0x00

#define SERVER_ADDRESS      (byte)0xFF

/*
 * The command type.
 */
enum COMMAND_TYPE : byte
{
	COMMAND_REQUEST = 0x00,
	COMMAND_RESPONSE = 0x01
};

/*
 * The possible responses
 */
enum COMMAND_RESPONSE : byte
{
	RESPONSE_ACK = 0x00,
	RESPONSE_NAK = 0x01
};

/*
 * The commands list.
 */
enum COMMAND_REQUEST : byte
{
	CMD_Start = 0x00,
	CMD_Stop = 0x01,
	CMD_Disable = 0x02,
	CMD_Enable = 0x03,
	CMD_SetHour = 0x04,
	CMD_SetMinute = 0x05,
	CMD_SetSecond = 0x06,
	CMD_EngineStartup = 0x07,
	CMD_Pause = 0x08,
	CMD_Ping = 0x09,
	CMD_EngineShutdown = 0x10,
	CMD_Open = 0x11,
	CMD_Close = 0x12,
	CMD_SelfTest = 0x13,
	CMD_GetParameter = 0x14,
	CMD_SetParameter = 0x15,
	CMD_Available = 0x16,
	CMD_UnAvailable = 0x17,
	CMD_Resume = 0x18
};

/*
 * The command parsing state.
 */
enum PARSING_STATE
{
	WAITING_FOR_HEADER = 0x00,
	WAITING_PACKET_TYPE = 0x01,
	WAITING_DEVICE_TYPE = 0x02,
	WAITING_SENDER = 0x03,
	WAITING_RECIPIENT = 0x04,
	WAITING_COMMAND = 0x05,
	WAITING_DATA_LENGHT = 0x06,
	WAITING_FOR_STX = 0x07,
	WAITING_FOR_DATA = 0x08,
	WAITING_FOR_ETX = 0x09,
	WAITING_FOR_CHECKSUM = 0x10,
	WAITING_FOR_EOT = 0x11
};

#define UNUSED_PARAMETER(x) (void)(x)

#define EMPTY_COMMAND       (byte)0x99

/*
 * Structure to store command code / function pairs
 */
typedef struct {
    byte commandCode = EMPTY_COMMAND;
    callbackFunction commandCallback;
} COMMAND, *PCOMMAND;

/*
 * Encapsulates the RiegoMatic serial network protocol.
 */
class SerialNetwork
{
    private:

        /*
         * The registered commands.
         */
        COMMAND _commands[MAX_COMMANDS];

        /*
         * Serial device in use.
         */
        Stream *_dev;

        /*
         * The address ID of this device.
         */
        byte _address = ALL_ADDRESSES;

        /*
         * The device type.
         */
        byte _device = ALL_DEVICES;

        /*
         * Statistics gathering.
         */
        STATS _stats;

        /*
         * The request messaging.
         */
        PACKAGE _package;

        /*
         * The state machine state.
         */
        PARSING_STATE _state;

        /*
         * Gets the registered command by it's command ID.
         */
        PCOMMAND getCommand(byte command);

        /*
         * Callback function for every command received.
         */
        callbackFunction _commandCatchAll = NULL;

        /*
         * Callback function to call for any un existing command requested.
         */
        callbackFunction _commandNotFound = NULL;

        /*
         * The control pin for the RS485 serial.
         */
        byte _dePin = NO_CONTROL;

        /*
         * CRC checking is an error-checking mechanism, similar to a checksum.
         */
        bool packet_crc_check(byte deviceType, byte sender, byte receiver, byte command, byte argument, byte crc);

        /*
         * Returns a byte that is the value of all other bytes XORed together
         */
        int getCRC(byte deviceType, byte sender, byte receiver, byte command, byte argument);

        /*
         * Send a command response (NAK or ACK) with or without parameter to the remote machine.
         */
        boolean sendResponse(byte, byte);

		/*
		 * Resets the state machine used for process and internal packet
		 */
		void reset();

		/*
         * Start transmitting. Sets the rx/tx enable pins
         */
        void startTX();

        /*
         * Stop transmitting. Sets the rx/tx enable pins
         */
        void stopTX();

        /*
         * Writes binary data to the serial port.
         */
        size_t writeToSerial(byte);

        /*
         * Reads incoming serial data.
         */
        byte readFromSerial();

    public:

        /*
         * Creates a new instance of the SerialNetwork class.
         * @serial : A hardware serial port
         */
        SerialNetwork(Stream &serial);

        /*
         * Creates a new instance of the SerialNetwork class.
         * @serial : A hardware serial port
         * @control : The pin controlling the transcriber, RX when LOW, TX when HIGH
         */
        SerialNetwork(Stream &serial, const int &control);

        /*
         * Destructor of this class
         */
        ~SerialNetwork();

        /*
         *
         */
        void begin();

        /*
         * Sets the device local address.
         * @address : The node address
         */
        void setDeviceAddress(uint8_t address);

        /*
         * Sets the device type.
         * @device : The device type.
         */
        void setDeviceType(uint8_t device);

        /*
         * Sets a function that is called on each command received.
         */
        void setCommandCatchAll(callbackFunction func);

        /*
         * Sets a function that is called when the command is not found.
         */
        void setCommandNotFound(callbackFunction func);

        /*
         * Process any pending incoming data on the serial port.
         */
        boolean process();

        /*
         * Registers a callback function to be executed when a valid packet is received.
         */
        void registerCommand(byte command, callbackFunction func);

        /*
         * Un-registers a callback function for a specified command.
         */
        void unregisterCommand(byte command);

        /*
         * The ACK signal is sent by the receiving station (destination) back to the sending station (source) after the receipt of a recognizable block of data of specific size.
         */
        boolean sendACK();

        /*
         * The ACK signal is sent by the receiving station (destination) back to the sending station (source) after the receipt of a recognizable block of data of specific size.
         * @argument : An optional parameter with information about the request result.
         */
        boolean sendACK(byte argument);

        /*
         * The NAK signal is typically sent by the recipient of a message (destination) back to the sender (source) after the receipt of an unrecognizable block of data.
         */
        boolean sendNAK();

        /*
         * Gets a pointer to the internal network stats.
         */
        PSTATS getStats();

        /*
         * Returns true if the value of a parameter if it exists.
         */
        bool hasParameter();

        /*
         * Reset networks statistics.
         */
        void resetStats();
};

#endif
