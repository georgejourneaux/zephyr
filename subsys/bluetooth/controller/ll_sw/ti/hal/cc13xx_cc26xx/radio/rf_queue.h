#ifndef RF_QUEUE_H
#define RF_QUEUE_H

#include <stdbool.h>

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

bool rf_queue_is_full(dataQueue_t *queue, rfc_dataEntryGeneral_t *current_entry);
rfc_dataEntryGeneral_t *rf_queue_next_entry(rfc_dataEntryGeneral_t *current_entry);
rfc_dataEntryGeneral_t *rf_queue_define_queue(dataQueue_t *queue, uint8_t *buffer,
					      uint16_t buffer_size, uint8_t number_of_entries,
					      uint16_t length);

#endif /* RF_QUEUE_H */