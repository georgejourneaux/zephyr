#include <stdint.h>
#include <stdlib.h>

/* clang-format off */
#include "rf_queue.h"
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)
/* clang-format on */

rfc_dataEntryGeneral_t *rf_queue_next_entry_general(rfc_dataEntryGeneral_t *entry)
{
	entry->status = DATA_ENTRY_PENDING;
	return ((rfc_dataEntryGeneral_t *)entry->pNextEntry);
}

rfc_dataEntryGeneral_t *rf_queue_define_queue_general(dataQueue_t *queue, uint8_t *buffer,
						      uint16_t buffer_size,
						      uint8_t number_of_entries,
						      uint16_t max_entry_data_size)
{

	if (buffer_size < RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(number_of_entries,
							  RF_QUEUE_DATA_ENTRY_GENERAL_HEADER_SIZE,
							  max_entry_data_size)) {
		/* queue does not fit into buffer */
		return NULL;
	}

	uint8_t pad = RF_QUEUE_QUEUE_ALIGN_PADDING(RF_QUEUE_DATA_ENTRY_GENERAL_HEADER_SIZE,
						   max_entry_data_size);
	uint16_t entry_size = (RF_QUEUE_DATA_ENTRY_GENERAL_HEADER_SIZE + max_entry_data_size + pad);

	/* Set the Data Entries common configuration */
	uint8_t *first_entry = buffer;
	rfc_dataEntryGeneral_t *entry = (rfc_dataEntryGeneral_t *)buffer;
	for (uint8_t entry_index = 0; entry_index < number_of_entries; entry_index++) {
		entry = (rfc_dataEntryGeneral_t *)(first_entry + (entry_index * entry_size));
		entry->status = DATA_ENTRY_PENDING;
		entry->config.type = DATA_ENTRY_TYPE_GEN;
		entry->config.lenSz = 0;
		entry->length = max_entry_data_size;

		entry->pNextEntry = (uint8_t *)entry + entry_size;
	}
	/* Make circular Last.Next -> First */
	entry->pNextEntry = first_entry;

	/* Create Data Entry Queue and configure for circular buffer Data Entries */
	queue->pCurrEntry = first_entry;
	queue->pLastEntry = NULL;

	return ((rfc_dataEntryGeneral_t *)queue->pCurrEntry);
}

rfc_dataEntryPointer_t *rf_queue_next_entry_pointer(rfc_dataEntryPointer_t *tail_entry)
{
	tail_entry->status = DATA_ENTRY_EMPTY;
	tail_entry->pData = NULL;
	tail_entry->length = 0;
	return ((rfc_dataEntryPointer_t *)tail_entry->pNextEntry);
}

rfc_dataEntryPointer_t *rf_queue_insert_entry_pointer(rfc_dataEntryPointer_t *head_entry,
						      uint8_t *data_buffer,
						      uint8_t data_buffer_size)
{
	head_entry->status = DATA_ENTRY_PENDING;
	head_entry->pData = data_buffer;
	head_entry->length = data_buffer_size;
	return ((rfc_dataEntryPointer_t *)head_entry->pNextEntry);
}

rfc_dataEntryPointer_t *rf_queue_define_queue_pointer(dataQueue_t *queue, uint8_t *buffer,
						      uint16_t buffer_size,
						      uint8_t number_of_entries)
{

	if (buffer_size < RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(
				  number_of_entries, RF_QUEUE_DATA_ENTRY_POINTER_HEADER_SIZE, 0)) {
		/* queue does not fit into buffer */
		return NULL;
	}

	uint8_t pad = RF_QUEUE_QUEUE_ALIGN_PADDING(RF_QUEUE_DATA_ENTRY_POINTER_HEADER_SIZE, 0);
	uint16_t entry_size = (RF_QUEUE_DATA_ENTRY_POINTER_HEADER_SIZE + pad);

	/* Set the Data Entries common configuration */
	uint8_t *first_entry = buffer;
	rfc_dataEntryPointer_t *entry = (rfc_dataEntryPointer_t *)buffer;
	for (uint8_t entry_index = 0; entry_index < number_of_entries; entry_index++) {
		entry = (rfc_dataEntryPointer_t *)(first_entry + (entry_index * entry_size));
		entry->status = DATA_ENTRY_EMPTY;
		entry->config.type = DATA_ENTRY_TYPE_PTR;
		entry->config.lenSz = 0;
		entry->length = 0;

		entry->pNextEntry = (uint8_t *)entry + entry_size;
	}
	/* Make circular Last.Next -> First */
	entry->pNextEntry = first_entry;

	/* Create Data Entry Queue and configure for circular buffer Data Entries */
	queue->pCurrEntry = first_entry;
	queue->pLastEntry = NULL;

	return ((rfc_dataEntryPointer_t *)queue->pCurrEntry);
}
