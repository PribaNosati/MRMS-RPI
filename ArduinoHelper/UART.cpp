#include "UART.h"
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>

using namespace std;

Message::Message(){}

/** A byte in message
@param index
*/
uint8_t& Message::operator[](uint8_t index) {
	return buffer[index];
}

/** Continue building message by appending to the tail
@param data - data to be appended
*/
void Message::append(uint8_t data) {
	if (nextBufferPos == MAXIMUM_MESSAGE_SIZE - 1)//Overflow
		exit(75);
	buffer[nextBufferPos] = data;
	nextBufferPos++;
	bufferTypes[nextTypesPos] = UINT8;
	nextTypesPos++;
}

/** Continue building message by appending to the tail
@param data - data to be appended
*/
void Message::append(uint16_t data) {
	if (nextBufferPos == MAXIMUM_MESSAGE_SIZE - 2)//Overflow
		exit(76);
	Mix mix;
	mix.int16 = data;
	buffer[nextBufferPos++] = mix.bytes[0];
	buffer[nextBufferPos++] = mix.bytes[1];
	bufferTypes[nextTypesPos] = UINT16;
	nextTypesPos++;
}

/** Continue building message by appending to the tail
@param data - data to be appended
*/
void Message::append(string data) {
	for (uint8_t i = 0; i < data.length(); i++)
		append((uint8_t)data[i]);
	append((uint8_t)0);
}

/** Buffer
@return - buffer
*/
uint8_t* Message::bytes() {
	return buffer;
}

/** Display content
*/
void Message::print() {
	cout << "message " << (int)size() << " bytes: ";
	for (uint8_t i = 0; i < size(); i++) {
		if (i != 0)
			cout << ", ";
		cout << (int)buffer[i];
	}
}

/** Read
@return - next
*/
uint8_t Message::readUInt8() {
	return buffer[nextReadPos++];
}

/** Read
@return - next
*/
uint16_t Message::readUInt16() {
	Mix mix;
	mix.bytes[0] = buffer[nextReadPos++];
	mix.bytes[1] = buffer[nextReadPos++];
	return mix.uint16;
}

/** Read
@return - next
*/
string Message::readString() {
	string str;
	while (uint8_t byte = buffer[nextReadPos++])
		str += byte;
	return str;
}

/** Clear message in order to start building a new one
*/
void Message::reset() {
	nextBufferPos = 0;
	nextTypesPos = 0;
}

/** Size
@return - number of bytes
*/
uint8_t Message::size() {
	return nextBufferPos;
}




/**Constructor
@param speed - Sets the data rate in bits per second (baud) for serial data transmission. Use one of these rates: 300, 600, 1200, 2400, 4800,
9600, 14400, 19200, 28800, 38400, 57600, or 115200. The other party must use the same speed.
*/
UART::UART(uint32_t speed)
{
    char dev[] = "/dev/serial0";
	if ((_handle = serialOpen(dev, speed)) < 0) {
		cout << "Error opening " << dev << ". Rights?" << endl;
	}
	else
		cout << dev << " opened." << endl;
}

UART::~UART()
{
	serialClose(_handle);
}

/**Returns the number of characters available for reading, or -1 for any error condition, in which case errno will be set appropriately.
@return - number of available bytes
*/
int UART::available() {
	return serialDataAvail(_handle) > 0;
}

/**Reads first byte of the incoming serial data.
@return - the first byte of incoming serial data available (or -1 if no data is available).
*/
uint8_t UART::read()
{
	try {
		uint8_t ch = serialGetchar(_handle);
		return ch;
	}
	catch (...) {
		cerr << " Error in readBlock().";
		throw;
	}
}

/** Reads characters from the serial port into a buffer. The function terminates if the determined length has been read, or it times out.
@param buffer - the buffer to store the bytes in.
@param length - the number of bytes to read.
@return - the number of bytes placed in the buffer.
*/
int UART::read(uint8_t size, uint8_t * data)
{

	try {
		int bytesReadCount = ::read(_handle, data, size);
		return bytesReadCount;
	}
	catch (...) {
		cerr << " Error in readBlock().";
		throw;
	}
}

/** Reads a message
@return - the message
@param verbose - print details
*/
Message UART::readMessage(bool verbose) {
	Message message;
	while (available())
		message.append((uint8_t)read());
    if (verbose) {
        cout << "Inbound ";
		message.print();
		cout << endl;
	}
	return message;
}

/** Writes a single byte to the serial port.
@param byte - a byte to send.
*/
void UART::write(uint8_t byte)
{
	try {
		serialPutchar(_handle, byte);
	}
	catch (...) {
		cerr << " Error in write().";
		throw 86;
	}
}

/** Sends the nul-terminated string to the serial device identified by the given file descriptor.
*/
void UART::write(char *string){
    try {
		serialPuts(_handle, string);
	}
	catch (...) {
		cerr << " Error in write().";
		throw 89;
	}
}

/** Writes series of bytes to the serial port; to send the characters representing the digits of a number use the print() function instead.
@param size - buffer's size
@param buffer - data
*/
void UART::write(uint8_t size, uint8_t *bytes) {
    try{
		::write(_handle, bytes, size);
    }
	catch (...) {
		cerr << " Error in write().";
		throw;
	}
}

/** Writes a message to serial port.
@param message
@param verbose - print details
*/
void UART::write(Message message, bool verbose) {
	if (verbose) {
        cout << "Outbound ";
		message.print();
		cout << endl;
	}
	write(message.size(), message.bytes());
}
