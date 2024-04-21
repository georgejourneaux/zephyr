#include <stdint.h>
#include <stdlib.h>

/* clang-format off */
#include "rf_queue.h"
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)
/* clang-format on */

bool rf_queue_is_full(dataQueue_t *queue, rfc_dataEntryGeneral_t *current_entry)
{
	return ((current_entry->status != DATA_ENTRY_PENDING) &&
		((uint8_t *)current_entry ==
		 ((rfc_dataEntryGeneral_t *)queue->pCurrEntry)->pNextEntry));
}

rfc_dataEntryGeneral_t *rf_queue_next_entry(rfc_dataEntryGeneral_t *entry)
{
	/* Set status to pending */
	entry->status = DATA_ENTRY_PENDING;
	return ((rfc_dataEntryGeneral_t *)entry->pNextEntry);
}

rfc_dataEntryGeneral_t *rf_queue_define_queue(dataQueue_t *queue, uint8_t *buffer,
					      uint16_t buffer_size, uint8_t number_of_entries,
					      uint16_t length)
{

	if (buffer_size < RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(number_of_entries, length)) {
		/* queue does not fit into buffer */
		return NULL;
	}

	/* Padding needed for 4-byte alignment? */
	uint8_t pad = RF_QUEUE_QUEUE_ALIGN_PADDING(length);

	uint16_t entry_size = (RF_QUEUE_DATA_ENTRY_HEADER_SIZE + length + pad);

	/* Set the Data Entries common configuration */
	uint8_t *first_entry = buffer;
	rfc_dataEntryGeneral_t *entry = (rfc_dataEntryGeneral_t *)buffer;
	for (uint8_t entry_index = 0; entry_index < number_of_entries; entry_index++) {
		entry = (rfc_dataEntryGeneral_t *)(first_entry + (entry_index * entry_size));
		entry->status = DATA_ENTRY_PENDING;       // Pending - starting state
		entry->config.type = DATA_ENTRY_TYPE_GEN; // General Data Entry
		entry->config.lenSz = 0;                  // No length indicator byte in data
		entry->length = length;                   // Total length of data field

		entry->pNextEntry = &entry->data + length + pad;
	}
	/* Make circular Last.Next -> First */
	entry->pNextEntry = first_entry;

	/* Create Data Entry Queue and configure for circular buffer Data Entries */
	queue->pCurrEntry = first_entry;
	queue->pLastEntry = NULL;

	return ((rfc_dataEntryGeneral_t *)queue->pCurrEntry);
}
