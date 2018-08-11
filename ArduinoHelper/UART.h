#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <string>

#define MAXIMUM_MESSAGE_SIZE 16

using namespace std;

class Message {
	typedef enum {INT8, INT16, INT32, UINT8, UINT16, UINT32 } Types;

	uint8_t buffer[MAXIMUM_MESSAGE_SIZE];
	Types bufferTypes[MAXIMUM_MESSAGE_SIZE];
	uint8_t nextBufferPos = 0;
	uint8_t nextReadPos = 0;
	uint8_t nextTypesPos = 0;

	union Mix
	{
		uint8_t bytes[4];
		short int16;
		unsigned short uint16;
		int int32;
		unsigned int uint32;
	};

public:
	Message();

	/** A byte in message
	@param index
	*/
	uint8_t& operator[](uint8_t index);

	/** Continue building message by appending to the tail
	@param data - data to be appended
	*/
	void append(uint8_t data);

	/** Continue building message by appending to the tail
	@param data - data to be appended
	*/
	void append(uint16_t data);

	/** Continue building message by appending to the tail
	@param data - data to be appended
	*/
	void append(string data);

	/** Buffer
	@return - buffer
	*/
	uint8_t* bytes();

    /** Display content
	*/
	void print();

	/** Read
	@return - next
	*/
	uint8_t readUInt8();

	/** Read
	@return - next
	*/
	uint16_t readUInt16();

	/** Read
	@return - next
	*/
	string readString();

	/** Clear message in order to start building a new one
	*/
	void reset();

	/** Size
	@return - number of bytes
	*/
	uint8_t size();
};


class UART
{
    private:
        int _handle; /// Reference to the serial port.

    public:
        /**Constructor
        @param speed - Sets the data rate in bits per second (baud) for serial data transmission. Use one of these rates: 300, 600, 1200, 2400, 4800,
            9600, 14400, 19200, 28800, 38400, 57600, or 115200. The other party must use the same speed.
        */
        UART(uint32_t speed = 115200);

        virtual ~UART();

        /**Returns the number of characters available for reading, or -1 for any error condition, in which case errno will be set appropriately.
        @return - number of available bytes
        */
        int available();

        /**Reads first byte of the incoming serial data.
        @return - the first byte of incoming serial data available (or -1 if no data is available).
        */
        uint8_t read();

        /** Reads characters from the serial port into a buffer. The function terminates if the determined length has been read, or it times out.
        @param buffer - the buffer to store the bytes in.
        @param length - the number of bytes to read.
        @return - the number of bytes placed in the buffer.
        */
        int read(uint8_t size, uint8_t *bytes);

        /** Reads a message
        @return - the message
        @param verbose - print details
        */
        Message readMessage(bool verbose = false);

        /** Writes a single byte to the serial port.
        @param byte - a byte to send.
        */
        void write(uint8_t byte);

        /** Sends the nul-terminated string to the serial device identified by the given file descriptor.
        */
        void write(char *string);

        /** Writes series of bytes to the serial port; to send the characters representing the digits of a number use the print() function instead.
        @param size - buffer's size
        @param buffer - data
        */
        void write(uint8_t size, uint8_t *bytes);

        /** Writes a message to serial port.
        @param message
        @param verbose - print details
        */
        void write(Message message, bool verbose = false);
};

#endif // UART_H
