/*
 * SerialNetwork.cpp - library for RiegoMatic serial communications protocol
 */

#include <SerialNetwork.h>

/*
 * Destructor of the class.
 */
SerialNetwork::~SerialNetwork()
{

}

/*
 * Creates a new instance of the SerialNetwork class.
 * @serial : A hardware serial port
 */
SerialNetwork::SerialNetwork(Stream &serial)
{
	_dev = &serial;
	_dePin = NO_CONTROL;

	reset();
	resetStats();
}

/*
 * Creates a new instance of the SerialNetwork class.
 * @serial : A hardware serial port
 * @control : The pin controlling the transcriber, RX when LOW, TX when HIGH
 */
SerialNetwork::SerialNetwork(Stream &serial, const int &control)
{
	_dev = &serial;
	_dePin = control;

	reset();
	resetStats();
}

/*
 * Resets the state machine used for process and internal packet
 */
void SerialNetwork::reset()
{
	/* Resets the machine state */
	_state = WAITING_FOR_HEADER;

	// Initialize the structure values with default values.
	memset(&_package, 0, sizeof(_package));
}

/*
 * Initializes all the communications protocol statistics.
 */
void SerialNetwork::begin()
{
	if (_dePin != NO_CONTROL)
	{
	  /* Set RS485 RX and TX controls pins to OUTPUT pins */
	  pinMode(_dePin, OUTPUT);
	}

	// Resets statistics.
	resetStats();
}

/*
 * Reset networks statistics.
 */
void SerialNetwork::resetStats()
{
	_stats.tx_packets = 0;
	_stats.tx_bytes = 0;
	_stats.rx_packets = 0;
	_stats.rx_bytes = 0;
	_stats.cmd_run = 0;
	_stats.cmd_not_found = 0;
	_stats.msg_crc_error = 0;
	_stats.msg_bad_formatted = 0;
	_stats.msg_discarded_packet = 0;
	_stats.msg_discarded_byte = 0;
}

/*
 * Sets the device address.
 */
void SerialNetwork::setDeviceAddress(uint8_t address)
{
	_address = address;
}

/*
 * Is this message intended to be received by all nodes
 */
bool PACKAGE::isBroadcast()
{
  return (receiver == ALL_ADDRESSES);
}

/*
 * Sets the device type.
 */
void SerialNetwork::setDeviceType(uint8_t device)
{
	_device = device;
}

/*
 * Statistics.  Just return a pointer to the internal statistics structure.
 */
PSTATS SerialNetwork::getStats()
{
    return &_stats;
}

/*
 * Main library method. It process the information received by the
 * serial network interface.
 *
 * If all the data is in the right format, and the calculated checksum
 * matches the received checksum, AND the destination station is our
 * station ID, then look for a registered command that matches the
 * command code. If all the above is true, execute the command's function.
 *
 */
boolean SerialNetwork::process()
{
	/* The last readed byte from the serial port */
	//byte readedByte;

	/* Only start if there is any data available */
	if (_dev->available())
	{
		/* Look to see if the first byte is a start character, otherwise nudge it off */
		if (_dev->peek() == STX)
		{
			/* If there is a full packet waiting */
			if (_dev->available() >= PROTOCOL_MESSAGE_BYTES)
			{
				reset();

				/* Read in a whole packet of data */
				byte stx = readFromSerial(); // Start of transmission
				byte dev = readFromSerial(); // The device type this message is addressed.
				byte snd = readFromSerial(); // Receiver device address.
				byte adr = readFromSerial(); // Receiver device address.
				byte cmd = readFromSerial(); // The command to be executed
				byte var = readFromSerial(); // Optional parameter, NULL if empty.
				byte crc = readFromSerial(); // CRC check
				byte etx = readFromSerial(); // End of transmission

				/* Check that the first and last chars are valid */
				if ((stx == STX) && (etx == ETX))
				{
					/* Increment the packets received counter */
					_stats.rx_packets++;

					/*
					 * Stop if CRC error, this means that the package
					 * has been modified since it left the server.
					 */
					if (packet_crc_check(dev, snd, adr, cmd, var, crc) == false)
					{
						_stats.msg_crc_error++;
						return false;
					}

					// Stop if devices doesn't match
					if (((dev != ALL_DEVICES) && (dev != _device)) && _device != ALL_DEVICES)
					{
						_stats.msg_discarded_packet++;
						return true;
					}

					// Stop if address doesn't match
					if (((adr != ALL_ADDRESSES) && (adr != _address)) && _address != ALL_ADDRESSES)
					{
						_stats.msg_discarded_packet++;
						return true;
					}

					/* Store last received parameter, address, device and command  */
					_package.receiver = adr;
					_package.command = cmd;
					_package.crc = crc;
					_package.deviceType = dev;
					_package.etx = etx;
					_package.sender = snd;
					_package.stx = stx;
					_package.argument = var;

					/* Fires the callback that runs for every command received */
					if (_commandCatchAll != NULL)
						_commandCatchAll(&_package);

					// Get the appropriate command to execute based on the registered commands.
					PCOMMAND command = getCommand(cmd);
					if (command)
					{
						/* Execute the command call back */
						command->commandCallback(&_package);

						/* Increment statistics of executed commands */
						_stats.cmd_run++;
					}
					else
					{
						if (_commandNotFound != NULL)
							_commandNotFound(&_package);

						/* Increment statistics of 'not found' command */
						_stats.cmd_not_found++;
					}
				}
				else
				{
					/* The received packet did not meet the expected format */
					/* Increment statistics of bad formated packets */
					_stats.msg_bad_formatted++;

					return false;
				}
			}
		}
		else
		{
			/* read and discard the most recent byte */
			readFromSerial();

			/* Increment the dropped byte statistics */
			_stats.msg_discarded_byte++;
		}
	}

	return true;
}

/*
 * Returns the checksum.
 */
int SerialNetwork::getCRC(byte dev, byte snd, byte adr, byte cmd, byte var)
{
	return (STX ^ dev ^ snd  ^ adr ^ cmd ^ var ^ ETX);
}

/**
 * CRC checking is an error-checking mechanism, similar to a checksum,
 * that enables an application to determine whether the information in
 * a packet has been modified.
 */
bool SerialNetwork::packet_crc_check(byte dev, byte snd, byte adr, byte cmd, byte var, byte crc)
{
	return (crc == getCRC(dev, snd, adr, cmd , var));
}

/*
 * Gets a command based on the supplied command ID
 */
PCOMMAND SerialNetwork::getCommand(byte command)
{
    unsigned char i;

	for (i = 0; i < MAX_COMMANDS; i++)
	{
		if (_commands[i].commandCode != EMPTY_COMMAND)
		{
			if (_commands[i].commandCode == command)
			{
				return &_commands[i];
			}
		}
	}

	return NULL;
}

/*
 * Add a new command code / function pair into the list of
 * registered commands.  If there is no room, fail silently.
 */
void SerialNetwork::registerCommand(byte command, callbackFunction func)
{
    unsigned char i;

	for (i = 0; i < MAX_COMMANDS; i++)
	{
		if (_commands[i].commandCode == EMPTY_COMMAND)
		{
			_commands[i].commandCode = command;
			_commands[i].commandCallback = func;
			return;
		}
	}
}

/*
 * Look for a registered command and delete it from the
 * list if found.  If not found, silently fail.
 */
void SerialNetwork::unregisterCommand(byte command)
{
    unsigned char i;

	for (i = 0; i < MAX_COMMANDS; i++)
	{
		if (_commands[i].commandCode == command)
		{
			_commands[i].commandCode = EMPTY_COMMAND;
			_commands[i].commandCallback = NULL;
			return;
		}
	}
}

/*
 * The ACK signal is sent by the receiving station (destination) back to the sending station (source) after the receipt of a recognizable block of data of specific size.
 */
boolean SerialNetwork::sendACK()
{
	return sendResponse(RESPONSE_ACK, NULL_VAR);
}

/*
 * The ACK signal is sent by the receiving station (destination) back to the sending station (source) after the receipt of a recognizable block of data of specific size.
 */
boolean SerialNetwork::sendACK(byte var)
{
	return sendResponse(RESPONSE_ACK, var);
}

/*
 * The NAK signal is typically sent by the recipient of a message (destination) back to the sender (source) after the receipt of an unrecognizable block of data.
 */
boolean SerialNetwork::sendNAK()
{
	return sendResponse(RESPONSE_NAK, NULL_VAR);
}

/*
 * Start transmitting. Sets the rx/tx enable pins
 */
void SerialNetwork::startTX()
{
	if (_dePin != NO_CONTROL)
	{
		/* sets the control pin as transmit mode */
		digitalWrite(_dePin, HIGH);
	}
}

/*
 * Stop transmitting. Sets the rx/tx enable pins
 */
void SerialNetwork::stopTX()
{
	if (_dePin != NO_CONTROL)
	{
		digitalWrite(_dePin, LOW);
	}
}

/*
 * Returns true if the value of a parameter if it exists.
 */
boolean SerialNetwork::hasParameter()
{
	return (bool)(&_package.argument != NULL);
}

/*
 * Send a message to a remote station. Origin is the originating station
 * "station" is the ID assigned to the remote station, "command" is a
 * command code that has been programmed into the remote station.
 *
 * The checksum is calculated as the sum of all the variable content of the packet
 * everything except the framing characters SOH/STX/ETX/EOT and
 * the checksum itself.
 *
 */
boolean SerialNetwork::sendResponse(byte res, byte var)
{
	/* if the original request is a broadcast don't send any response
	 * to avoid collisions with other devices on the networking trying
	 * to reply the message at the very same time.
	 */
    if (!_package.isBroadcast())
    {
    	// Calculate the CRC
        int crc = getCRC(_device, _address, SERVER_ADDRESS, res, var);

		/* set to transmit mode */
		startTX();

		/* Start of header by writing STX */
		writeToSerial(STX);

		/* Source device */
		writeToSerial(_device);

		/* Source address */
		writeToSerial(_address);

		/* destination address */
		writeToSerial(SERVER_ADDRESS);

		/* result code */
		writeToSerial(res);

		/* the resulting variable */
		writeToSerial(var);

		/* Checksum */
		writeToSerial(crc);

		/* End of header writing ETX */
		writeToSerial(ETX);

		/* Waits for the transmission of outgoing serial data to complete.  */
		_dev->flush();

		/* set to receive mode */
		stopTX();

		/* Increment the packet sent counter */
		_stats.tx_packets++;
    }

    /* The packet was written to the network stream successfully */
    return true;
}

byte SerialNetwork::readFromSerial()
{
	/* read from the serial port */
	byte data = (byte)_dev->read();

	/* increment bytes readed counter */
	_stats.rx_bytes++;

	/* return the data */
	return data;
}

size_t SerialNetwork::writeToSerial(byte data)
{
	/* write to the serial port */
	size_t size =_dev->write(data);

	/* Increment the packet sent counter */
	_stats.tx_bytes++;

	/* return the size of the written data */
	return size;
}

void SerialNetwork::setCommandCatchAll(callbackFunction func)
{
	_commandCatchAll = func;
};

void SerialNetwork::setCommandNotFound(callbackFunction func)
{
	_commandNotFound = func;
};
