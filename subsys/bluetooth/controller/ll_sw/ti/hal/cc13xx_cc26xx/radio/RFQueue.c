#include <stdint.h>
#include <stdlib.h>

/* clang-format off */
#include "RFQueue.h"
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)
/* clang-format on */

bool RFQueue_isFull(dataQueue_t *queue, rfc_dataEntry_t *current_entry)
{
	return ((current_entry->status != DATA_ENTRY_PENDING) &&
		(current_entry == ((rfc_dataEntry_t *)queue->pCurrEntry)->pNextEntry));
}

rfc_dataEntry_t *RFQueue_nextEntry(rfc_dataEntry_t *entry)
{
	/* Set status to pending */
	entry->status = DATA_ENTRY_PENDING;
	return (entry->pNextEntry);
}

rfc_dataEntry_t *RFQueue_defineQueue(dataQueue_t *dataQueue, uint8_t *buf, uint16_t buf_len,
				     uint8_t numEntries, uint16_t length)
{

	if (buf_len < (numEntries * (length + RF_QUEUE_DATA_ENTRY_HEADER_SIZE +
				     RF_QUEUE_QUEUE_ALIGN_PADDING(length)))) {
		/* queue does not fit into buffer */
		return (1);
	}

	/* Padding needed for 4-byte alignment? */
	uint8_t pad = 4 - ((length + RF_QUEUE_DATA_ENTRY_HEADER_SIZE) % 4);

	/* Set the Data Entries common configuration */
	uint8_t *first_entry = buf;
	int i;
	for (i = 0; i < numEntries; i++) {
		buf = first_entry + i * (RF_QUEUE_DATA_ENTRY_HEADER_SIZE + length + pad);
		((rfc_dataEntry_t *)buf)->status = DATA_ENTRY_PENDING; // Pending - starting state
		((rfc_dataEntry_t *)buf)->config.type = DATA_ENTRY_TYPE_GEN; // General Data Entry
		((rfc_dataEntry_t *)buf)->config.lenSz = 0; // No length indicator byte in data
		((rfc_dataEntry_t *)buf)->length = length;  // Total length of data field

		((rfc_dataEntryGeneral_t *)buf)->pNextEntry =
			&(((rfc_dataEntryGeneral_t *)buf)->data) + length + pad;
	}
	/* Make circular Last.Next -> First */
	((rfc_dataEntry_t *)buf)->pNextEntry = first_entry;

	/* Create Data Entry Queue and configure for circular buffer Data Entries */
	dataQueue->pCurrEntry = first_entry;
	dataQueue->pLastEntry = NULL;

	return ((rfc_dataEntry_t *)dataQueue->pCurrEntry);
}
