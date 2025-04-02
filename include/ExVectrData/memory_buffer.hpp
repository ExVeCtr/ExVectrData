#ifndef EXVECTRDATA_MEMORYCACHE_HPP_
#define EXVECTRDATA_MEMORYCACHE_HPP_

#include "stdint.h"
#include "stddef.h"


#include "memory_interface.hpp"


namespace VCTR
{

    namespace Data
    {

        /**
         * @brief Class to help synchronise data between two memories. The primary memory is used for changes. Once sync is called, changes are written to the secondary memory.
         * @note Both memories must be of the same size.
         */
        class Memory_Buffer
        {
        private:
            /// @brief Primary memory. Used for changes.
            Memory_Interface& primaryMemory_;
            /// @brief Secondary memory.
            Memory_Interface& secondaryMemory_;

            bool matching_ = false; //If the two memories have identical data.
            

        public:
            
            /**
             * @brief Constructor.
             * @param primaryMemory Primary memory. Used for changes.
             * @param secondaryMemory Secondary memory. This is where changes are written to.
             */
            Memory_Buffer(Memory_Interface& primaryMemory, Memory_Interface& secondaryMemory);

            /**
             * @brief Will copy data from the secondary memory to the primary memory. In cases like EEPROM, we want to use this at startup to initialise the primary memory with the secondary memory.
             * @param startIndex Starting index of a block of data to be copied.
             * @param endIndex Ending index of a block of data to be copied. (inclusive)
             * @return Number of bytes actually copied to the primary memory.
             */
            virtual size_t syncSecondaryToPrimary(size_t startIndex = 0, size_t endIndex = SIZE_MAX, bool doAll = false);

            /**
             * @brief Synchronises the memory with another memory. 
             * @note Synchronisation can be slow depending on the types of memories used. Syncing literally goes byte by byte, only writting when the data is different.
             * @param startIndex Starting index of a block of data to be synchronised.
             * @param endIndex Ending index of a block of data to be synchronised. (inclusive)
             * @param doAll If true, the data will not be checked byte by byte. The data from primary will be copied into a buffer and written to the secondary memory. This can be faster and reduces read calls, but increases the number of writes. Leave at false for EEPROM memorys.
             * @return Number of bytes actually written to the secondary memory.
             */
            virtual size_t sync(size_t startIndex = 0, size_t endIndex = SIZE_MAX, bool doAll = false);

            /**
             * @brief Returns the primary memory.
             * @return Primary memory.
             */
            Memory_Interface& getPrimaryMemory() { return primaryMemory_; };

            /**
             * @brief Returns the secondary memory.
             * @return Secondary memory.
             */
            Memory_Interface& getSecondaryMemory() { return secondaryMemory_; };

            /**
             * @returns if the memories are matching. This is set to true when the two memories are identical.
             */
            bool isMatching() { return matching_; };

        };

    }

}











#endif