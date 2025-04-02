#ifndef EXVECTRDATA_MEMORYINTERFACE_HPP_
#define EXVECTRDATA_MEMORYINTERFACE_HPP_

#include "stdint.h"
#include "stddef.h"


namespace VCTR
{

    namespace Data
    {

        /**
         * @brief   Generic memory interface. Used to read and write data to a memory.
         * @note    Use setIndex and the template functions to read and write data in a more programmer friendly way.
         */
        class Memory_Interface
        {
        private:

            /**
             * @brief Current access index to the memory. This is used to read and write data to the memory in a more convenient way.
             *        This is set to the next index after the last access to the memory.
             */
            uint32_t accessIndex_ = 0;


        public:

            virtual ~Memory_Interface() {}

            /**
             * @brief Reads data from the memory.
             * @param bufferPtr Pointer to the buffer where the data will be written to.
             * @param numBytes Number of bytes to be read.
             * @param index Index from where the data will be read from the memory
             * @param bufferIndex Index to where the data will be written in the buffer.
             * @return Number of bytes read from the memory and placed into the buffer.
             */
            virtual size_t readMem(uint8_t* bufferPtr, size_t numBytes, size_t index, size_t bufferIndex = 0) = 0;
            
            /**
             * @brief Writes data to the memory.
             * @param bufferPtr Pointer to the buffer where the data will be read from.
             * @param numBytes Number of bytes to be written.
             * @param index Index to where the data will be written to the memory.
             * @param bufferIndex Index from where the data will be read in the buffer.
             * @return Number of bytes written to the memory from the buffer.
             */
            virtual size_t writeMem(uint8_t const* bufferPtr, size_t numBytes, size_t index, size_t bufferIndex = 0) = 0;

            /**
             * @brief Transfers data from given memory to this memory.
             * @param memory Memory to receive data from.
             * @param numBytes Number of bytes to be transferred.
             * @param toIndex Index to where the data will be written to this memory.
             * @param fromIndex Index from where the data will be read in the given memory.
             * @return Number of bytes written to this memory from the given memory.
             */
            virtual size_t transferFrom(Memory_Interface& memory, size_t numBytes, size_t toIndex = 0, size_t fromIndex = 0) = 0;

            /**
             * @returns the length of the memory in bytes. NOT the amount of memory used.
             */
            virtual size_t size() = 0;

            /**
             * @brief Sets the access index to the given index.
             * @param index Index to be set.
             */
            size_t accessIndex(size_t index = SIZE_MAX) {
                if (index != SIZE_MAX) accessIndex_ = index;
                return accessIndex_;
            };


            /**
             * Below are functions to help access the memory in a more programmer friendly way.
             */

            /**
             * * @brief Reads data from the memory in a more programmer friendly way.
             * * @param dataReceive reference to the object to receive the memory data.
             * * @param index index from where the data will be read from the memory. Defaults to SIZE_MAX, which means follow the current access index.
             * * @return Number of bytes read from the memory and placed into the buffer.
             */
            template<typename TYPE>
            size_t readMem(TYPE& dataReceive, size_t index = SIZE_MAX) {
                if (index == SIZE_MAX) index = accessIndex_;
                auto ret = readMem((uint8_t*)&dataReceive, sizeof(TYPE), index);
                accessIndex_ += ret;
                return ret;
            };

            /**
             * * @brief Writes data to the memory in a more programmer friendly way.
             * * @param dataSend reference to the object to be written to the memory.
             * * @param index index to where the data will be written to the memory. Defaults to SIZE_MAX, which means follow the current access index.
             * * @return Number of bytes written to the memory from the buffer.
             */
            template<typename TYPE>
            size_t writeMem(const TYPE& dataSend, size_t index = SIZE_MAX) {
                if (index == SIZE_MAX) index = accessIndex_;
                auto ret = writeMem((uint8_t*)&dataSend, sizeof(TYPE), index);
                accessIndex_ += ret;
                return ret;
            }; 

            
        };


    }

}











#endif