#ifndef EXVECTRDATA_MEMORYMANAGER_HPP_
#define EXVECTRDATA_MEMORYMANAGER_HPP_

#include "stdint.h"
#include "stddef.h"

#include "memory_interface.hpp"


namespace VCTR
{

    namespace Data
    {

        /**
         * @brief Memory Manager takes care of allocating and deallocating memory in the given Memory_Interface.
         * @note Keep in mind that the memory manager is slower than direct memory.
         */
        class Memory_Manager
        {
        private:

            constexpr static uint8_t VERSION_ = 1; ///< Update to version number means previous versions are not compatable.

            Memory_Interface& memory; ///< Memory to be managed.

            uint32_t allocatedItems = 0; ///< Number of allocated items in the memory.
        

        public:

            Memory_Manager(Memory_Interface& memory);

            /**
             * * @brief Allocates memory in the given memory.
             * * @param value Standard value to be written to the memory.
             * * @return Key to the allocated memory. Will be 0 if allocation failed.
             */
            template<typename TYPE>
            uint32_t allocateItem(const TYPE& value = TYPE(), uint32_t key = 0) {
                auto key = allocateSpace(sizeof(TYPE), key);
                if (key != 0) {
                    writeItem(value, key); // Write the value to the memory.
                }
                return key;
            }

            /**
             * * @brief Deallocates the memory occupied by the given key.
             */
            void deallocateItem(uint32_t key);

            /**
             * * @brief Will find the item with the given key and write the data from the memory to the given value.
             * * @param value Reference to the object to receive the memory data. 
             * * @param key Key to the item in the memory. If zero, then a random key will be generated.
             * * @return true if the item was found and the data was written to the value. false if the item was not found or failed to read memory.
             */
            template<typename TYPE>
            bool readItem(TYPE& value, uint32_t key) {

                uint32_t lastIndex, foundIndex;
                findItem(lastIndex, foundIndex, key, sizeof(TYPE));

                if (foundIndex == 0) return false; // Item not found.

                memory.accessIndex(foundIndex + 10); // Set the access index to the beginning of the item data.
                if (!memory.readMem(value)) return false; // Failure to read memory.

                return true; // Success.

            }

            /**
             * * @brief Will find the item with the given key and write the data from the value to the memory.
             * * @param value Reference to the object to be written to the memory.
             * * @param key Key to the item in the memory. If zero, then a random key will be generated.
             * * @return true if the item was found and the data was written to the memory. false if the item was not found or failed to read memory.
             */
            template<typename TYPE>
            bool writeItem(const TYPE& value, uint32_t key) {

                uint32_t lastIndex, foundIndex;
                findItem(lastIndex, foundIndex, key, sizeof(TYPE));

                if (foundIndex == 0) return false; // Item not found.

                memory.accessIndex(foundIndex + 10); // Set the access index to the beginning of the item data.
                if (!memory.writeMem(value)) return false; // Failure to read memory.

                return true; // Success.

            }

            /**
             * @brief Will remove empty space between allocated items. 
             * @warning This is an extremely slow operation and should be used only when needed (memory full as an attempt to allocate new item).
             */
            //void compressMemory();

            /**
             * * @brief Clears the memory and deallocates all items while also reinitialising it. This is a very cheap o(1) operation.
             */
            void clearMemory();

            /**
             * * @brief Returns a maximum of the number of bytes used. In fragmented memory, this will not count free space between allocated items.
             */
            //size_t usedMemory();


        private:

            /**
             * * @brief Finds the index of the item in the memory with the given key. Will also check if the memory is valid for manager.
             * * @param lastIndex The index of the previous item. This could be the same as the index of the item found if there is only one item in the memory.
             * * @param indexFound If key is 0, then the index of the first free space or 0 if none found or failure. If key is not 0, then the index of the item with the given key will be returned or 0 if not found.
             * * @param key Key to the item in the memory. If zero, then the index of first free space for the given size will be returned.
             * * @param numBytes Number of bytes to be read from the memory. Used to verify if the item matches the expected size or size of free space. If zero, then the first item with the given key will be returned.
             */
            void findItem(uint32_t& lastIndex, uint32_t& indexFound, uint32_t key, uint32_t numBytes = 0);

            /**
             * * @brief Allocates memory in the given memory with the given size in bytes
             * * @param size Size of the item in bytes.
             * * @param key Key to the item in the memory. If zero, then a random key will be generated.
             * * @return Key to the allocated memory. Will be 0 if allocation failed.
             */
            uint32_t allocateSpace(uint32_t size, uint32_t key);

            /**
             * * @brief Allocates memory for an array in the given memory with the given size of an item in bytes and number of items.
             * * @param sizePerItem Size of the item in bytes.
             * * @param numberItems Number of items in the array.
             * * @param count Number of items to be allocated.
             * * @return Key to the allocated memory. Will be 0 if allocation failed.
             */
            //uint32_t allocateSpace(uint32_t sizePerItem, uint32_t numberItems, uint32_t count);

            
        };

    }

}











#endif