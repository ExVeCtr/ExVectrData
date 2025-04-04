#include "stdint.h"
#include "stddef.h"

#include "ExVectrCore/print.hpp"

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrData/memory_interface.hpp"

#include "ExVectrData/external_chips/at24cx.hpp"


namespace VCTR
{

    namespace Data
    {

        

        Memory_AT24CX::Memory_AT24CX(size_t memorySize, size_t pageSize) {

            memorySize_ = memorySize;
            pageSize_ = pageSize;

        }


        bool Memory_AT24CX::initMemory(HAL::DigitalIO &ioBus) {

            if (ioBus.getInputType() != HAL::IO_TYPE_t::BUS_I2C)
            {
                VCTR::Core::printE("QMC5883 given incorrect input type. Must be I2C. Given type: %d.\n", ioBus.getInputType());
                return false;
            }

            if (ioBus.getOutputType() != HAL::IO_TYPE_t::BUS_I2C)
            {
                VCTR::Core::printE("QMC5883 given incorrect output type. Must be I2C. Given type: %d.\n", ioBus.getOutputType());
                return false;
            }

            ioBus_ = &ioBus;

        }
            
            
        size_t Memory_AT24CX::readMem(uint8_t* bufferPtr, size_t numBytes, size_t index, size_t bufferIndex) {
            if (bufferIndex + numBytes > memorySize_) numBytes = memorySize_ - bufferIndex; // Out of bounds. Reduce the number of bytes to be read.
            if (index + numBytes > memorySize_) numBytes = memorySize_ - index; // Out of bounds. Reduce the number of bytes to be read.
            if (readBus(bufferPtr, numBytes, index)) {
                VCTR::Core::printE("AT24CX readMem(): failed to read data from bus!\n");
                return 0;
            }
            return numBytes;
        }
        
        
        size_t Memory_AT24CX::writeMem(uint8_t const* bufferPtr, size_t numBytes, size_t index, size_t bufferIndex) {
            if (bufferIndex + numBytes > memorySize_) numBytes = memorySize_ - bufferIndex; // Out of bounds. Reduce the number of bytes to be written.
            if (index + numBytes > memorySize_) numBytes = memorySize_ - index; // Out of bounds. Reduce the number of bytes to be written.
            if (writeBus(bufferPtr, numBytes, index)) {
                VCTR::Core::printE("AT24CX writeMem(): failed to write data to bus!\n");
                return 0;
            }
            return numBytes;
        }

        
        size_t Memory_AT24CX::transferFrom(Memory_Interface& memory, size_t numBytes, size_t toIndex, size_t fromIndex) {
            if (toIndex + numBytes > memorySize_) numBytes = memorySize_ - toIndex; // Out of bounds. Reduce the number of bytes to be written.
            if (fromIndex + numBytes > memory.size()) numBytes = memory.size() - fromIndex; // Out of bounds. Reduce the number of bytes to be written.
            uint8_t buffer_[numBytes];
            if (memory.readMem(buffer_, numBytes, fromIndex) != numBytes) {
                VCTR::Core::printE("AT24CX transferFrom(): failed to read data from given memory!\n");
                return 0;
            }
            size_t bytesWritten = numBytes;
            while (bytesWritten < numBytes) {

                auto bytesToWrite = numBytes > pageSize_ ? pageSize_ : numBytes;
                if (writeBus(buffer_ + bytesWritten, bytesToWrite, toIndex + bytesWritten)) {
                    VCTR::Core::printE("AT24CX transferFrom(): failed to write data to bus!\n");
                    return 0;
                }

                bytesWritten += bytesToWrite;

            }
            return numBytes;
        }

        
        size_t Memory_AT24CX::size() {
            return memorySize_;
        }


        /**
         * @brief Writes data to the device on the bus.
         */
        bool Memory_AT24CX::writeBus(const uint8_t* bufferPtr, size_t numBytes, size_t address) {
            if (ioBus_->writeData(&address, 2, false) != 2) {
                VCTR::Core::printE("AT24CX writeBus(): failed to write address to bus!\n");
                ioBus_->writeData(&address, 0, true);
                return false;
            }
            if (ioBus_->writeData(bufferPtr, numBytes, true) != numBytes) {
                VCTR::Core::printE("AT24CX writeBus(): failed to write data to bus!\n");
                return false;
            }
            return true;
        }

        /**
         * @brief Reads data from the device on the bus.
         */
        bool Memory_AT24CX::readBus(uint8_t* bufferPtr, size_t numBytes, size_t address) {
            if (ioBus_->writeData(&address, 2, false) != 2) {
                VCTR::Core::printE("AT24CX writeBus(): failed to write address to bus!\n");
                ioBus_->writeData(&address, 0, true);
                return false;
            }
            if (ioBus_->readData(bufferPtr, numBytes, true) != numBytes) {
                VCTR::Core::printE("AT24CX writeBus(): failed to read data from bus!\n");
                return false;
            }
            return true;
        }

        

    }

}

