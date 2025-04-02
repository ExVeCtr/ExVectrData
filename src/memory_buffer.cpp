#include "stdint.h"
#include "stddef.h"


#include "ExVectrData/memory_interface.hpp"

#include "ExVectrData/memory_buffer.hpp"


namespace VCTR
{

    namespace Data
    {

        
        Memory_Buffer::Memory_Buffer(Memory_Interface& primaryMemory, Memory_Interface& secondaryMemory) : 
            primaryMemory_(primaryMemory), 
            secondaryMemory_(secondaryMemory) 
        {};

        size_t Memory_Buffer::syncSecondaryToPrimary(size_t startIndex, size_t endIndex, bool doAll) {
            if (endIndex > primaryMemory_.size()) endIndex = primaryMemory_.size() - 1;
            if (startIndex > endIndex) return 0;

            size_t numBytes = endIndex - startIndex + 1;
            //uint8_t buffer[numBytes];// = new uint8_t[numBytes];
            //size_t bytesRead = secondaryMemory_.read(buffer, numBytes, startIndex);
            //size_t bytesWritten = primaryMemory_.write(buffer, bytesRead, startIndex);

            if (startIndex == 0 && endIndex == primaryMemory_.size() - 1) {
                matching_ = true;
            }

            if (doAll) {
                return primaryMemory_.transferFrom(secondaryMemory_, numBytes, startIndex, startIndex);
            } //If doAll, the following will be skipped.

            size_t bytesWritten = 0;
            for (size_t i = startIndex; i <= endIndex; i++)
            {
                uint8_t bytePri = 0;
                uint8_t byteSec = 0;
                secondaryMemory_.readMem(&byteSec, 1, i);
                primaryMemory_.readMem(&bytePri, 1, i);
                if (bytePri != byteSec) {
                    primaryMemory_.writeMem(&byteSec, 1, i);
                    bytesWritten++;
                }
            }

            return bytesWritten;
        }

        size_t Memory_Buffer::sync(size_t startIndex, size_t endIndex, bool doAll) {

            if (endIndex > primaryMemory_.size()) endIndex = primaryMemory_.size() - 1;
            if (startIndex > endIndex) return 0;

            size_t numBytes = endIndex - startIndex + 1;
            //uint8_t buffer[numBytes];// = new uint8_t[numBytes];
            //size_t bytesRead = primaryMemory_.read(buffer, numBytes, startIndex);
            //size_t bytesWritten = secondaryMemory_.write(buffer, bytesRead, startIndex);

            if (startIndex == 0 && endIndex == primaryMemory_.size() - 1) {
                matching_ = true;
            }

            if (doAll) {
                return secondaryMemory_.transferFrom(primaryMemory_, numBytes, startIndex, startIndex);
            } //If doAll, the following will be skipped.

            size_t bytesWritten = 0;
            for (size_t i = startIndex; i <= endIndex; i++)
            {
                uint8_t bytePri = 0;
                uint8_t byteSec = 0;
                primaryMemory_.readMem(&bytePri, 1, i);
                secondaryMemory_.readMem(&byteSec, 1, i);
                if (bytePri != byteSec) {
                    secondaryMemory_.writeMem(&bytePri, 1, i);
                    bytesWritten++;
                }
            }

            return bytesWritten;

        }


    }

}