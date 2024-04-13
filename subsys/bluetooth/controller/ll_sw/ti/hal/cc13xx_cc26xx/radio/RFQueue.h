#ifndef RF_QUEUE_H
#define RF_QUEUE_H

/* clang-format off */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)
/* clang-format on */

#define RF_QUEUE_DATA_ENTRY_HEADER_SIZE 8 // Contant header size of a Generic Data Entry

#define RF_QUEUE_QUEUE_ALIGN_PADDING(length)                                                       \
	(4 - ((length + RF_QUEUE_DATA_ENTRY_HEADER_SIZE) % 4)) // Padding offset

#define RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(numEntries, dataSize)                                      \
	(numEntries *                                                                              \
	 (RF_QUEUE_DATA_ENTRY_HEADER_SIZE + dataSize + RF_QUEUE_QUEUE_ALIGN_PADDING(dataSize)))

uint8_t RFQueue_nextEntry(dataQueue_t *queue);
uint8_t RFQueue_defineQueue(dataQueue_t *queue, uint8_t *buf, uint16_t buf_len, uint8_t numEntries,
			    uint16_t length);

#endif /* RF_QUEUE_H */