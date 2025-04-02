#include "stdint.h"
#include "stddef.h"

#include "ExVectrData/memory_interface.hpp"

#include "ExVectrData/memory_manager.hpp"


namespace VCTR
{

    namespace Data
    {
        /**
         * The memory is formed according to the following structure:
         * - 0xABFA Header byte 1 and 2 to identify if the memory actualy contains memory manager data.
         * - version byte to identify the version of the memory manager.
         * - Allocated items byte L to identify the number of allocated items in the memory.
         * - Allocated items byte H to identify the number of allocated items in the memory.
         * - Byte index 20: Beginning of the first item in the memory.
         * 
         * Each Memory Item is formed as the following:
         * - 0xEFA6 Header byte 1 and 2 to identify if this item is valid.
         * - Index byte L to the next Item in the memory. If final item, then points to next free space (Which will be corrupt item)
         * - above byte 2.
         * - Key byte L of the key to the item.
         * - above byte 2.
         * - Number of bytes per item L.
         * - above byte 2.
         * - Byte index 10: Beginning of the item data.
         * 
         */

        Memory_Manager::Memory_Manager(Memory_Interface& memory) :
            memory(memory)
        {
            allocatedItems = 0;
        }

        void Memory_Manager::deallocateItem(uint32_t key) {

            if (key == 0) return; // No key given.

            uint16_t allocatedItems = 0; //Used to make sure we dont go into infinite loop.
            if (!memory.readMem(allocatedItems)) return;

            uint32_t indexFound = 20; //Used to find the first free space in the memory.
            uint32_t lastIndex = 20; //Used to find the last index in the memory.
            findItem(lastIndex, indexFound, 0); // Find the first free space in the memory.

            if (indexFound == 0) return; // No free space found in the memory.
            
            if (indexFound == 20) { // If the first item is the one we are looking for, then we simply need to remove the header.
                
                memory.accessIndex(20); // Set the access index to the beginning of the memory.
                memory.writeMem((uint16_t)0); // Header set 0;
                
                return; // Memory is cleared.
            }

            uint16_t nextItemIndex, nextItemHeader;
            memory.accessIndex(indexFound + 2); // We want to retrieve the next item index.
            if (!memory.readMem(nextItemIndex)) return;
            memory.accessIndex(nextItemIndex); // Now we want the next items header.
            memory.readMem(nextItemHeader);
            if (nextItemHeader == 0xEFA6) { //There is a next item.

                memory.accessIndex(indexFound + 2); // Set the access index to the last item in the memory.
                memory.writeMem(nextItemIndex); // Write the index to the next item in the memory.

            } else { //There is no next item

                    
                memory.accessIndex(indexFound + 2); // Set the access index to the last item in the memory.
                memory.writeMem((uint16_t)0); // Set the next items index to 0 to indicate that this is the last item.

            }

            allocatedItems--; // Decrement the number of allocated items in the memory and update memory.
            memory.accessIndex(3);
            memory.writeMem(allocatedItems);

        }

        //void compressMemory();

        void Memory_Manager::clearMemory() {

            //Set memory to the default values. This will clear the memory and set it to the default values.
            memory.accessIndex(0);
            memory.writeMem((uint16_t)0xABFA);
            memory.writeMem((uint8_t)VERSION_); // Version byte
            memory.writeMem((uint8_t)0); // Allocated items byte L
            memory.writeMem((uint8_t)0); // Allocated items byte H

            //Make initial item corrupt (wrong header byte). To indicate empty memory.
            memory.accessIndex(20); // Beginning of the first item in the memory.
            memory.writeMem((uint8_t)0); // Header byte 1
            
        }

        //size_t Memory_Manager::usedMemory();

        void Memory_Manager::findItem(uint32_t& lastIndex, uint32_t& indexFound, uint32_t key, uint32_t numBytes) {

            /**
             * Currently the allocation algorithm ignores free memory between items when searching for free space.
             */

            uint8_t byte8 = 0;
            uint16_t byte16 = 0;

            lastIndex = indexFound = 0;

            //if (numBytes == 0) return; // Zero length makes no sense.
            memory.accessIndex(0); // Set the access index to the beginning of the memory.
            if (memory.readMem(byte16) && byte16 != 0xABFA) clearMemory(); // Memory is not initialized, clear it.
            if (memory.readMem(byte8) && byte8 != VERSION_) clearMemory(); // Memory is a different version.

            uint16_t allocatedItems = 0; //Used to make sure we dont go into infinite loop.
            if (!memory.readMem(allocatedItems)) return;

            indexFound = 20; //Used to find the first free space in the memory.
            lastIndex = 20; //Used to find the last index in the memory.
            uint32_t itemsFound = 0; //The number of items we have traversed in the memory.
            uint16_t header, nextIndex, keyFound, size;
            while (indexFound < memory.size()) {

                memory.accessIndex(indexFound);
                if (!memory.readMem(header)) break; // Item is not valid. This should be a free space.
                if (!memory.readMem(nextIndex)) break; //Failure to read memory.
                if (!memory.readMem(keyFound)) break; //Failure to read memory.
                if (!memory.readMem(size)) break; //Failure to read memory.
                
                if (header != 0xEFA6) break; // Item is not valid. Something is wrong

                //Cases for when the we have reached the end of the memory.
                if (nextIndex == 0 && key != 0) break; // That was the final item. If we are looking for a key, then it was not found.
                if (nextIndex == 0 && key == 0) return; // If we are looking for the final item, then we found it. Return the index.

                //Per item checking
                if (key != 0 && key == keyFound && (size == numBytes || numBytes == 0)) return; // Item we are searching for.

                if (itemsFound >= allocatedItems) break; // We have traversed all items in the memory. Something is wrong...

                lastIndex = indexFound; // Set the last index to the current item.
                indexFound = nextIndex; // Set the index to the next item in the memory.
                itemsFound++; // Increment the number of items found.

            }

            lastIndex = indexFound = 0; // Something is wrong...
            return; // No item found.

        }

        uint32_t Memory_Manager::allocateSpace(uint32_t size, uint32_t key) {

            uint32_t indexFound = 20; //Used to find the first free space in the memory.
            uint32_t lastIndex = 20; //Used to find the last index in the memory.
            findItem(lastIndex, indexFound, 0, size); // Find the first free space in the memory.

            if (indexFound == 0) return 0; // No free space found in the memory.

            //Setup the item in the memory.
            memory.accessIndex(indexFound);
            memory.writeMem((uint16_t)0xEFA6); // Write the header byte to the item.
            memory.writeMem((uint16_t)0); // Set the index to zero as this is the last item.
            memory.writeMem((uint16_t)(key == 0 ? indexFound : key)); // If no key is given, use the index as the key.
            memory.writeMem((uint16_t)size); // Write the size of the item.

            //Set the previous items index to the new item.
            if (lastIndex != indexFound) {
                memory.accessIndex(lastIndex + 2); // Set the access index to the last item in the memory.
                memory.writeMem((uint16_t)indexFound); // Write the index to the next item in the memory.
            }

            allocatedItems++; // Increment the number of allocated items in the memory.
            memory.accessIndex(3); // Set the access index to the beginning of the memory.
            memory.writeMem((uint16_t)allocatedItems); // Write the new number allocated items

            return key == 0 ? indexFound : key; // Return the key of the item. if no key is given, use the index as the key.
            
        }

        //uint32_t Memory_Manager::allocateSpace(uint32_t sizePerItem, uint32_t numberItems, uint32_t count);

    }

}

