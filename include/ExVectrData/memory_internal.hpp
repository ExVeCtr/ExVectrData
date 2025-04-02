#ifndef EXVECTRDATA_MEMORYINTERNAL_HPP_
#define EXVECTRDATA_MEMORYINTERNAL_HPP_

#include "stdint.h"
#include "stddef.h"

#include "memory_interface.hpp"


namespace VCTR
{

    namespace Data
    {

        /**
         * @brief   A static memory based on a simple array. Memory wil be stored in standard RAM.
         */
        template<size_t SIZE>
        class Memory_Internal : public Memory_Interface
        {
        private:

            uint8_t internalMem_[SIZE]; ///< Memory to be used. This is a static array of size SIZE.


        public:

            /**
             * @brief Reads data from the memory.
             * @param bufferPtr Pointer to the buffer where the data will be written to.
             * @param numBytes Number of bytes to be read.
             * @param index Index from where the data will be read from the memory
             * @param bufferIndex Index to where the data will be written in the buffer.
             * @return Number of bytes read from the memory and placed into the buffer.
             */
            size_t readMem(uint8_t* bufferPtr, size_t numBytes, size_t index, size_t bufferIndex = 0) override {
                if (index + numBytes > SIZE) return 0; // Out of bounds.
                if (bufferIndex + numBytes > SIZE) return 0; // Out of bounds.
                memcpy(bufferPtr + bufferIndex, internalMem_ + index, numBytes); // Copy the data from the memory to the buffer.
                return numBytes;
            }
            
            /**
             * @brief Writes data to the memory.
             * @param bufferPtr Pointer to the buffer where the data will be read from.
             * @param numBytes Number of bytes to be written.
             * @param index Index to where the data will be written to the memory.
             * @param bufferIndex Index from where the data will be read in the buffer.
             * @return Number of bytes written to the memory from the buffer.
             */
            size_t writeMem(uint8_t const* bufferPtr, size_t numBytes, size_t index, size_t bufferIndex = 0) override {
                if (index + numBytes > SIZE) return 0; // Out of bounds.
                if (bufferIndex + numBytes > SIZE) return 0; // Out of bounds.
                memcpy(internalMem_ + index, bufferPtr + bufferIndex, numBytes); // Copy the data from the buffer to the memory.
                return numBytes;
            }

            /**
             * @brief Transfers data from given memory to this memory.
             * @param memory Memory to receive data from.
             * @param numBytes Number of bytes to be transferred.
             * @param toIndex Index to where the data will be written to this memory.
             * @param fromIndex Index from where the data will be read in the given memory.
             * @return Number of bytes written to this memory from the given memory.
             */
            size_t transferFrom(Memory_Interface& memory, size_t numBytes, size_t toIndex = 0, size_t fromIndex = 0) override {
                if (toIndex + numBytes > SIZE) return 0; // Out of bounds.
                if (fromIndex + numBytes > SIZE) return 0; // Out of bounds.
                size_t bytesRead = memory.readMem(internalMem_, numBytes, fromIndex, toIndex); // Read the data from the given memory to this memory.
                return bytesRead;
            }

            /**
             * @returns the length of the memory in bytes.
             */
            size_t size() override {
                return SIZE; // Return the size of the memory.
            }
            

        };

    }

}











#endif