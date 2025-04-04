#include "stdint.h"
#include "stddef.h"

#include "ExVectrCore/print.hpp"

#include "ExVectrData/memory_interface.hpp"

#include "ExVectrData/memory_manager.hpp"


namespace VCTR
{

    namespace Data
    {
        /**
         * The memory is formed according to the following structure:
         * - 0-1  0xABFA Header to identify if the memory actually contains memory manager data.
         * - 2-3  version byte to identify the version of the memory manager.
         * - 4-5  Allocated items to identify the number of allocated items in the memory.
         * - 6-9 Index to the first item in the memory.
         * - 20-... Beginning of the Items allocation section
         * 
         * Each Memory Item is formed as the following:
         * - 0-1  0xEFA6 Header byte 1 and 2 to identify if this item is valid.
         * - 2-5  Index to the next Item in the memory. If final item, then this will be 0.
         * - 6-7  Key of the the item.
         * - 8-9  Number of bytes of data.
         * - 10-... Beginning of the item data.
         * 
         */

        Memory_Manager::Memory_Manager(Memory_Interface& memory) :
            memory(memory)
        {
            allocatedItems = 0;
        }

        void Memory_Manager::deallocateItem(uint32_t key) {

            //LOG_MSG("Deallocating item with key %d\n", key);

            if (key == 0) return; // No key given.

            uint16_t allocatedItems = 0; //Used to make sure we dont go into infinite loop.
            memory.accessIndex(4); // Set the access index to the beginning of the memory.
            if (!memory.readMem(allocatedItems)) return;

            uint32_t indexFound = 20; //Used to find the first free space in the memory.
            uint32_t lastIndex = 20; //Used to find the last index in the memory.
            uint32_t nextIndex = 0;
            findItem(lastIndex, indexFound, nextIndex, key); // Find the first free space in the memory.

            //LOG_MSG("Item found at Index: %d, lastIndex: %d\n", indexFound, lastIndex);

            if (indexFound == 0) return; // Item was not found.

            if (lastIndex == 0) { // Item to be removed is the first item in the memory.

                memory.accessIndex(6);
                memory.writeMem(nextIndex);

                return;

            }

            //memory.accessIndex(nextItemIndex); // Now we want the next items header.
            //memory.readMem(nextItemHeader);
            if (nextIndex != 0) { //There is a next item.

                memory.accessIndex(lastIndex + 2); // Set the access index to the last item in the memory.
                memory.writeMem(nextIndex); // Write the index to the next item in the memory.

            } else { //There is no next item
                    
                memory.accessIndex(lastIndex + 2); // Set the access index to the next item section of the last item in the memory.
                memory.writeMem((uint32_t)0); // Set the next items index to 0 to indicate that this is the last item.

            }


            allocatedItems--; // Decrement the number of allocated items in the memory and update memory.
            memory.accessIndex(4);
            memory.writeMem(allocatedItems);

        }

        //void compressMemory();

        void Memory_Manager::clearMemory() {

            //LOG_MSG("Clearing memory...\n");

            //Set memory to the default values. This will clear the memory and set it to the default values.
            memory.accessIndex(0);
            memory.writeMem((uint16_t)0xABFA);
            memory.writeMem((uint16_t)VERSION_); // Version byte
            memory.writeMem((uint16_t)0); // Allocated items
            memory.writeMem((uint32_t)0); // Index to the first item in the memory.

            //Make initial item corrupt (wrong header byte). To indicate empty memory.
            //memory.accessIndex(20); // Beginning of the first item in the memory.
            //memory.writeMem((uint16_t)0); // Set header.
            //memory.writeMem((uint32_t)0); // Set index to 0. This is the last item.

            uint16_t version;
            uint16_t header;
            memory.accessIndex(0);  
            memory.readMem(header); // Read the header byte.
            memory.readMem(version); // Read the version byte.

            if (header != 0xABFA) { // Memory is not initialized
                //LOG_MSG("Memory failed to initialise. Header is wrong after writing.\n");
                return;
            }
            if (version != VERSION_) { // Memory is a different version. We do not want to simply clear the memory, as this will delete all data.
                //LOG_MSG("Memory failed to initialise. Version is wrong after writing.\n");
                return;
            }
            
        }

        //size_t Memory_Manager::usedMemory();

        void Memory_Manager::findItem(uint32_t& lastIndex, uint32_t& indexFound, uint32_t& nextIndex, uint32_t key, uint32_t numBytes) {

            /**
             * Currently the allocation algorithm ignores free memory between items when searching for free space.
             */

            uint16_t version = 0;
            uint16_t memoryHeader = 0;
            uint16_t allocatedItems = 0; //Used to make sure we dont go into infinite loop.
            uint32_t firstItemIndex = 0; //Used to find the first item in the memory.
            lastIndex = indexFound = nextIndex = 0;

            //First read all the memory header data.
            memory.accessIndex(0); // Set the access index to the beginning of the memory.
            if (!memory.readMem(memoryHeader)) return; // Read the header byte.
            if (!memory.readMem(version)) return; // Read the version byte.
            if (!memory.readMem(allocatedItems)) return;
            if (!memory.readMem(firstItemIndex)) return; // Read the index to the first item in the memory.

            //LOG_MSG("Memory Header: %x, Version: %d, Allocated Items: %d, First Item Index: %d. Current memory Index: %d\n", memoryHeader, version, allocatedItems, firstItemIndex, memory.accessIndex());

            if (memoryHeader != 0xABFA) { // Memory is not initialized
                //LOG_MSG("Memory not initialized. Incorrect Header. Expected 0xABFA, got %x\n", memoryHeader);
                return;
            }
            if (version != VERSION_) { // Memory is a different version. We do not want to simply clear the memory, as this will delete all data.
                //LOG_MSG("Memory version mismatch. Expected %d, got %d\n", VERSION_, version);
                return;
            }

            //Looks like the memory is valid and version matches.
            if (firstItemIndex == 0) { // No items in the memory.
                //LOG_MSG("No items in the memory.\n");
                lastIndex = 0;
                indexFound = 20; // Set the index to the first item in the memory.
                return; // No items in the memory. We can allocate space at the first item.
            }

            ////LOG_MSG("First item index: %d\n", firstItemIndex);

            indexFound = firstItemIndex; //Set to the first item in the memory.
            lastIndex = 0; //Used to find the last index in the memory.
            nextIndex = 0; //Used to find the next index in the memory.
            uint32_t itemsFound = 0; //The number of items we have traversed in the memory.
            uint16_t header, keyFound, size;
            while (indexFound < memory.size()) {

                memory.accessIndex(indexFound);
                if (!memory.readMem(header)) break; // Item is not valid. This should be a free space.
                if (!memory.readMem(nextIndex)) break; //Failure to read memory.
                if (!memory.readMem(keyFound)) break; //Failure to read memory.
                if (!memory.readMem(size)) break; //Failure to read memory.

                //LOG_MSG("Searching for item. Index: %d, Header: %x, Next Index: %d, Key: %d, Size: %d\n", indexFound, header, nextIndex, keyFound, size);
                
                if (header != 0xEFA6 && indexFound == 20) return; // Item is not valid. Something is wrong

                //Per item checking
                if (key != 0 && key == keyFound && (size == numBytes || numBytes == 0)) return; // Item we are searching for.

                //Cases for when the we have reached the end of the memory.
                if (nextIndex == 0 && key != 0) break; // That was the final item. If we are looking for a key, then it was not found.
                if (nextIndex == 0 && key == 0) { // If we are looking for the final item, then we found it. Return the index.
                    lastIndex = indexFound; // Set the last index to the current item.
                    indexFound = indexFound + 10 + size; // Set the index to the end of the item.
                    return;
                }

                if (itemsFound >= allocatedItems) break; // We have traversed all items in the memory. Something is wrong...

                lastIndex = indexFound; // Set the last index to the current item.
                indexFound = nextIndex; // Set the index to the next item in the memory.
                itemsFound++; // Increment the number of items found.

            }

            lastIndex = indexFound = 0; // Something is wrong...
            return; // No item found.

        }

        uint32_t Memory_Manager::allocateSpace(uint32_t size, uint32_t key) {

            ////LOG_MSG("Allocating space\n");

            uint32_t indexFound = 20; //Used to find the first free space in the memory.
            uint32_t lastIndex = 20; //Used to find the last index in the memory.
            uint32_t nextIndex = 0; //Used to find the next index in the memory.
            findItem(lastIndex, indexFound, nextIndex, 0, size); // Find the first free space in the memory.

            if (indexFound == 0) return 0; // No free space found in the memory.

            //LOG_MSG("Found free space at index %d, lastindex: %d\n", indexFound, lastIndex);

            //If no key given, set key to found index.
            if (key == 0) key = indexFound; // Set the key to the index found.

            //Setup the item in the memory.
            memory.accessIndex(indexFound);
            memory.writeMem((uint16_t)0xEFA6); // Write the header byte to the item.
            memory.writeMem((uint32_t)0); // Set the next index to zero as this is the last item.
            memory.writeMem((uint16_t)key); // If no key is given, use the index as the key.
            memory.writeMem((uint16_t)size); // Write the size of the item.

            //Set the previous items index to the new item.
            if (lastIndex == 0) { // If this is the first item in the memory, then we need to set the index to the first item in the memory.
                //LOG_MSG("Setting initial index\n");
                memory.accessIndex(6); // Set the access index to the beginning of the memory.
                memory.writeMem((uint32_t)indexFound); // Write the index to the first item in the memory.
            } else if (lastIndex != indexFound) {
                //LOG_MSG("Setting previous item index to %d\n", indexFound);
                memory.accessIndex(lastIndex + 2); // Set the access index to the last item in the memory.
                memory.writeMem((uint32_t)indexFound); // Write the index to the next item in the memory.
            }

            allocatedItems++; // Increment the number of allocated items in the memory.
            memory.accessIndex(4); // Set the access index to the beginning of the memory.
            memory.writeMem((uint16_t)allocatedItems); // Write the new number allocated items

            return key; // Return the key of the item. if no key is given, use the index as the key.
            
        }

        uint8_t Memory_Manager::getVersionSaved() {

            uint16_t version = 0;
            uint16_t header = 0;
            memory.accessIndex(0); // Set the access index to the beginning of the memory.
            if (!memory.readMem(header)) return 0; // Failure to read memory.
            if (!memory.readMem(version)) return 0; // Failure to read memory.
            if (header != 0xABFA) return 0; // Memory is not initialized.
            return version; // Return the version of the memory manager.

        }

    }

}

