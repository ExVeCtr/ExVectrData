#ifndef EXVECTRDATA_MEMORYCACHE_HPP_
#define EXVECTRDATA_MEMORYCACHE_HPP_

#include "stdint.h"
#include "stddef.h"

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrData/memory_interface.hpp"


namespace VCTR
{

    namespace Data
    {

        /**
         * @brief This class implements the Memory_Interface for the AT24CX series of EEPROMs.
         */
        class Memory_AT24CX : public Memory_Interface
        {
        private:
            
            static constexpr uint8_t AT24CX_ID = 0x50; //AT24CX ID

            bool initialised_ = false; //If the memory has been initialised and is in working condition

            HAL::DigitalIO *ioBus_ = nullptr;

            size_t memorySize_ = 0; //Size of the memory in bytes
            size_t pageSize_ = 0; //Size of the memory page in bytes
            

        public:

            /**
             * @brief Constructor.
             * @param memorySize Size of the memory in bytes.
             * @param pageSize Size of the memory page in bytes. Normally 32 bytes.
             */
            Memory_AT24CX(size_t memorySize, size_t pageSize = 32);

            /**
             * @brief Initialises the memory.
             * @param ioBus IO bus to use for communications.
             */
            bool initMemory(HAL::DigitalIO &ioBus);
            
            /**
             * @brief Reads data from the memory.
             * @param bufferPtr Pointer to the buffer where the data will be written to.
             * @param numBytes Number of bytes to be read.
             * @param index Index from where the data will be read from the memory
             * @param bufferIndex Index to where the data will be written in the buffer.
             * @return Number of bytes read from the memory and placed into the buffer.
             */
            size_t readMem(uint8_t* bufferPtr, size_t numBytes, size_t index, size_t bufferIndex = 0) override;
            
            /**
             * @brief Writes data to the memory.
             * @param bufferPtr Pointer to the buffer where the data will be read from.
             * @param numBytes Number of bytes to be written.
             * @param index Index to where the data will be written to the memory.
             * @param bufferIndex Index from where the data will be read in the buffer.
             * @return Number of bytes written to the memory from the buffer.
             */
            size_t writeMem(uint8_t const* bufferPtr, size_t numBytes, size_t index, size_t bufferIndex = 0) override;

            /**
             * @brief Transfers data from given memory to this memory.
             * @param memToReceiveData Memory to receive data from.
             * @param numBytes Number of bytes to be transferred. Defaults to SIZE_MAX, which means transfer all the data from the given memory.
             * @param toIndex Index to where the data will be written to this memory.
             * @param fromIndex Index from where the data will be read in the given memory.
             * @return Number of bytes written to this memory from the given memory.
             */
            size_t transferFrom(Memory_Interface& memToReceiveData, size_t numBytes = SIZE_MAX, size_t toIndex = 0, size_t fromIndex = 0) override;

            /**
             * @returns the length of the memory in bytes.
             */
            size_t size() override;

        private:

            /**
             * @brief Writes data to the device on the bus.
             */
            bool writeBus(const uint8_t* bufferPtr, size_t numBytes, size_t address);

            /**
             * @brief Reads data from the device on the bus.
             */
            bool readBus(uint8_t* bufferPtr, size_t numBytes, size_t address);


        };

    }

}











#endif