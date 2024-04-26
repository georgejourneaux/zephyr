#ifndef RF_QUEUE_H
#define RF_QUEUE_H

#include <stdbool.h>

/* clang-format off */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)
/* clang-format on */

#define DATA_ENTRY_EMPTY (UINT8_MAX)

#define RF_QUEUE_DATA_ENTRY_GENERAL_HEADER_SIZE (8)
#define RF_QUEUE_DATA_ENTRY_POINTER_HEADER_SIZE (12)

#define RF_QUEUE_QUEUE_ALIGN_PADDING(header_size, data_size) (4 - ((header_size + data_size) % 4))

#define RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(number_of_entries, header_size, data_size)                 \
	(number_of_entries *                                                                       \
	 (header_size + data_size + RF_QUEUE_QUEUE_ALIGN_PADDING(header_size, data_size)))

rfc_dataEntryGeneral_t *rf_queue_next_entry_general(rfc_dataEntryGeneral_t *current_entry);
rfc_dataEntryGeneral_t *rf_queue_define_queue_general(dataQueue_t *queue, uint8_t *buffer,
						      uint16_t buffer_size,
						      uint8_t number_of_entries,
						      uint16_t max_entry_data_size);

rfc_dataEntryPointer_t *rf_queue_next_entry_pointer(rfc_dataEntryPointer_t *tail_entry);
rfc_dataEntryPointer_t *rf_queue_insert_entry_pointer(rfc_dataEntryPointer_t *head_entry,
						      uint8_t *data_buffer,
						      uint8_t data_buffer_size);
rfc_dataEntryPointer_t *rf_queue_define_queue_pointer(dataQueue_t *queue, uint8_t *buffer,
						      uint16_t buffer_size,
						      uint8_t number_of_entries);

#endif /* RF_QUEUE_H */