#include "timssdr.h"
#include <pthread.h>
#include <string.h>

#define TRANSFER_COUNT        4
#define TRANSFER_BUFFER_SIZE  262144
#define DEVICE_BUFFER_SIZE    32768
#define USB_MAX_SERIAL_LENGTH 32

#define USB_CONFIG_STANDARD 0x1

#define RX_ENDPOINT_ADDRESS (LIBUSB_ENDPOINT_IN | 1)
#define TX_ENDPOINT_ADDRESS (LIBUSB_ENDPOINT_OUT | 2)

#ifndef bool
typedef int bool;
	#define true 1
	#define false 0
#endif

struct timssdr_device {
	libusb_device_handle* usb_device;
	struct libusb_transfer** transfers;
	timssdr_sample_block_cb_fn callback;
	volatile bool
		transfer_thread_started; /* volatile shared between threads (read only) */
	pthread_t transfer_thread;
	volatile bool streaming; /* volatile shared between threads (read only) */
	void* rx_ctx;
	void* tx_ctx;
	volatile bool do_exit;
	unsigned char buffer[TRANSFER_COUNT * TRANSFER_BUFFER_SIZE];
	bool transfers_setup;           /* true if the USB transfers have been setup */
	pthread_mutex_t transfer_lock;  /* must be held to cancel or restart transfers */
	volatile int active_transfers;  /* number of active transfers */
	pthread_cond_t all_finished_cv; /* signalled when all transfers have finished */
	bool flush;
	struct libusb_transfer* flush_transfer;
	timssdr_flush_cb_fn flush_callback;
	timssdr_tx_block_complete_cb_fn tx_completion_callback;
	void* flush_ctx;
};

static const uint16_t timssdr_usb_vid = TIMSSDR_VENDOR_ID;
static const uint16_t timssdr_usb_pid = TIMSSDR_PRODUCT_ID;

static uint16_t open_devices = 0;

static int create_transfer_thread(timssdr_device* device);

static libusb_context* g_libusb_context = NULL;
int last_libusb_error = LIBUSB_SUCCESS;

static int detach_kernel_drivers(libusb_device_handle* usb_device_handle)
{
	int i, num_interfaces, result;
	libusb_device* dev;
	struct libusb_config_descriptor* config;

	dev = libusb_get_device(usb_device_handle);
	result = libusb_get_active_config_descriptor(dev, &config);
	if (result < 0) {
		last_libusb_error = result;
		return TIMSSDR_ERROR_LIBUSB;
	}

	num_interfaces = config->bNumInterfaces;
	libusb_free_config_descriptor(config);
	for (i = 0; i < num_interfaces; i++) {
		result = libusb_kernel_driver_active(usb_device_handle, i);
		if (result < 0) {
			if (result == LIBUSB_ERROR_NOT_SUPPORTED) {
				return 0;
			}
			last_libusb_error = result;
			return TIMSSDR_ERROR_LIBUSB;
		} else if (result == 1) {
			result = libusb_detach_kernel_driver(usb_device_handle, i);
			if (result != 0) {
				last_libusb_error = result;
				return TIMSSDR_ERROR_LIBUSB;
			}
		}
	}
	return TIMSSDR_SUCCESS;
}

static int set_timssdr_configuration(libusb_device_handle* usb_device, int config)
{
	int result, curr_config;

	result = libusb_get_configuration(usb_device, &curr_config);
	if (result != 0) {
		last_libusb_error = result;
		return TIMSSDR_ERROR_LIBUSB;
	}

	if (curr_config != config) {
		result = detach_kernel_drivers(usb_device);
		if (result != 0) {
			return result;
		}
		result = libusb_set_configuration(usb_device, config);
		if (result != 0) {
			last_libusb_error = result;
			return TIMSSDR_ERROR_LIBUSB;
		}
	}

	result = detach_kernel_drivers(usb_device);
	if (result != 0) {
		return result;
	}
	return LIBUSB_SUCCESS;
}

static int allocate_transfers(timssdr_device* const device)
{
	if (device->transfers == NULL) {
		uint32_t transfer_index;
		device->transfers = (struct libusb_transfer**) calloc(
			TRANSFER_COUNT,
			sizeof(struct libusb_transfer*));
		if (device->transfers == NULL) {
			return TIMSSDR_ERROR_NO_MEM;
		}

		memset(device->buffer, 0, TRANSFER_COUNT * TRANSFER_BUFFER_SIZE);

		for (transfer_index = 0; transfer_index < TRANSFER_COUNT;
		     transfer_index++) {
			device->transfers[transfer_index] = libusb_alloc_transfer(0);
			if (device->transfers[transfer_index] == NULL) {
				return TIMSSDR_ERROR_LIBUSB;
			}

			libusb_fill_bulk_transfer(
				device->transfers[transfer_index],
				device->usb_device,
				0,
				&device->buffer[transfer_index * TRANSFER_BUFFER_SIZE],
				TRANSFER_BUFFER_SIZE,
				NULL,
				device,
				0);

			if (device->transfers[transfer_index]->buffer == NULL) {
				return TIMSSDR_ERROR_NO_MEM;
			}
		}
		return TIMSSDR_SUCCESS;
	} else {
		return TIMSSDR_ERROR_BUSY;
	}
}

static void* transfer_threadproc(void* arg)
{
	timssdr_device* device = (timssdr_device*) arg;
	int error;
	struct timeval timeout = {0, 500000};

	/*
	 * timssdr_transfer uses pause() and SIGALRM to print statistics and
	 * POSIX doesn't specify which thread must recieve the signal, block all
	 * signals here, so we don't interrupt their reception by
	 * timssdr_transfer or any other app which uses the library (#1323)
	 */

	while (device->do_exit == false) {
		error = libusb_handle_events_timeout(g_libusb_context, &timeout);
		if ((error != 0) && (error != LIBUSB_ERROR_INTERRUPTED)) {
			device->streaming = false;
		}
	}

	return NULL;
}

static int create_transfer_thread(timssdr_device* device)
{
	int result;

	if (device->transfer_thread_started == false) {
		device->streaming = false;
		device->do_exit = false;
		result = pthread_create(
			&device->transfer_thread,
			0,
			transfer_threadproc,
			device);
		if (result == 0) {
			device->transfer_thread_started = true;
		} else {
			return TIMSSDR_ERROR_THREAD;
		}
	} else {
		return TIMSSDR_ERROR_BUSY;
	}

	return TIMSSDR_SUCCESS;
}

static int transfers_check_setup(timssdr_device* device)
{
	if ((device->transfers != NULL) && (device->transfers_setup == true))
		return true;
	return false;
}

static int cancel_transfers(timssdr_device* device)
{
	uint32_t transfer_index;

	// If we're cancelling transfers for any reason, we're shutting down.
	device->streaming = false;

	if (transfers_check_setup(device) == true) {
		// Take lock while cancelling transfers. This blocks the
		// transfer completion callback from restarting a transfer
		// while we're in the middle of trying to cancel them all.
		pthread_mutex_lock(&device->transfer_lock);

		for (transfer_index = 0; transfer_index < TRANSFER_COUNT;
		     transfer_index++) {
			if (device->transfers[transfer_index] != NULL) {
				libusb_cancel_transfer(device->transfers[transfer_index]);
			}
		}

		if (device->flush_transfer != NULL)
			libusb_cancel_transfer(device->flush_transfer);

		device->transfers_setup = false;
		device->flush = false;

		// Now wait for the transfer thread to signal that all transfers
		// have finished, either by completing or being fully cancelled.
		while (device->active_transfers > 0) {
			pthread_cond_wait(
				&device->all_finished_cv,
				&device->transfer_lock);
		}
		pthread_mutex_unlock(&device->transfer_lock);

		return TIMSSDR_SUCCESS;
	} else {
		return TIMSSDR_ERROR_OTHER;
	}
}

static int free_transfers(timssdr_device* device)
{
	uint32_t transfer_index;

	if (device->transfers != NULL) {
		// libusb_close() should free all transfers referenced from this array.
		for (transfer_index = 0; transfer_index < TRANSFER_COUNT;
		     transfer_index++) {
			if (device->transfers[transfer_index] != NULL) {
				libusb_free_transfer(device->transfers[transfer_index]);
				device->transfers[transfer_index] = NULL;
			}
		}
		free(device->transfers);
		device->transfers = NULL;
	}

	libusb_free_transfer(device->flush_transfer);

	return TIMSSDR_SUCCESS;
}

static void LIBUSB_CALL timssdr_libusb_flush_callback(struct libusb_transfer* usb_transfer)
{
	bool success = usb_transfer->status == LIBUSB_TRANSFER_COMPLETED;

	// All transfers have now ended, so proceed with signalling completion.
	timssdr_device* device = (timssdr_device*) usb_transfer->user_data;
	pthread_mutex_lock(&device->transfer_lock);
	device->flush = false;
	device->active_transfers = 0;
	pthread_cond_broadcast(&device->all_finished_cv);
	pthread_mutex_unlock(&device->transfer_lock);

	if (device->flush_callback)
		device->flush_callback(device->flush_ctx, success);
}

static void LIBUSB_CALL
timssdr_libusb_transfer_callback(struct libusb_transfer* usb_transfer)
{
	timssdr_device* device = (timssdr_device*) usb_transfer->user_data;
	bool success, resubmit = false;
	int result;

	timssdr_transfer transfer = {
		.device = device,
		.buffer = usb_transfer->buffer,
		.buffer_length = TRANSFER_BUFFER_SIZE,
		.valid_length = usb_transfer->actual_length,
		.rx_ctx = device->rx_ctx,
		.tx_ctx = device->tx_ctx};

	success = usb_transfer->status == LIBUSB_TRANSFER_COMPLETED;

	if (device->tx_completion_callback != NULL) {
		device->tx_completion_callback(&transfer, success);
	}

	// Take lock to make sure that we don't restart a
	// transfer whilst cancel_transfers() is in the middle
	// of stopping them.
	pthread_mutex_lock(&device->transfer_lock);
	if (success) {
		if (device->streaming && (device->callback(&transfer) == 0) &&
		    (transfer.valid_length > 0)) {
			if ((resubmit = device->transfers_setup)) {
				if (usb_transfer->endpoint == TX_ENDPOINT_ADDRESS) {
					usb_transfer->length = transfer.valid_length;
					// Pad to the next 512-byte boundary.
					uint8_t* buffer = usb_transfer->buffer;
					while (usb_transfer->length % 512 != 0)
						buffer[usb_transfer->length++] = 0;
				}
				result = libusb_submit_transfer(usb_transfer);
			}
		} else if (device->flush) {
			result = libusb_submit_transfer(device->flush_transfer);
			if (result != LIBUSB_SUCCESS) {
				device->streaming = false;
				device->flush = false;
			}
		}
	} else {
		device->streaming = false;
		device->flush = false;
	}

	// If a data transfer was resubmitted successfully, we're done.
	if (!resubmit || result != LIBUSB_SUCCESS) {
		// No further calls should be made to the TX callback.
		device->streaming = false;

		// If this is the last transfer, signal that all are now finished.
		if (device->active_transfers == 1) {
			if (!device->flush) {
				device->active_transfers = 0;
				pthread_cond_broadcast(&device->all_finished_cv);
			}
		} else {
			device->active_transfers--;
		}
	}

	// Now we can release the lock. Our transfer was either
	// cancelled or restarted, not both.
	pthread_mutex_unlock(&device->transfer_lock);
}

static int kill_transfer_thread(timssdr_device* device)
{
	void* value;
	int result;

	if (device->transfer_thread_started != false) {
		/*
		 * Cancel transfers. This call will block until the transfer
		 * thread has handled all completion callbacks.
		 */
		cancel_transfers(device);

		// Set flag to tell the thread to exit.
		device->do_exit = true;

		/*
		 * Interrupt the event handling thread instead of
		 * waiting for timeout.
		 */
		libusb_interrupt_event_handler(g_libusb_context);

		value = NULL;
		result = pthread_join(device->transfer_thread, &value);
		if (result != 0) {
			return TIMSSDR_ERROR_THREAD;
		}
		device->transfer_thread_started = false;
	}

	/*
	 * Reset do_exit; we're now done here and the thread was
	 * already dead or is now dead.
	 */
	device->do_exit = false;

	return TIMSSDR_SUCCESS;
}

static int prepare_transfers(timssdr_device* device, const uint_fast8_t endpoint_address, libusb_transfer_cb_fn callback)
{
	int error;
	uint32_t transfer_index;
	uint32_t ready_transfers = 0;

	if (device->transfers == NULL) {
		// This shouldn't happen.
		return TIMSSDR_ERROR_OTHER;
	}

	// If setting up for TX, call the TX callback to fill each
	// transfer buffer.

	// All transfers must be filled before any are submitted.
	// Otherwise a transfer might complete whilst the others are
	// still being filled, causing the transfer thread to make a
	// concurrent call to the TX callback.

	// We also need to handle the case where the callback returns
	// nonzero to indicate completion, so keep count of how many
	// transfers were made ready to submit at this stage.

	if (endpoint_address == TX_ENDPOINT_ADDRESS) {
		for (transfer_index = 0; transfer_index < TRANSFER_COUNT;
		     transfer_index++) {
			timssdr_transfer transfer = {
				.device = device,
				.buffer = device->transfers[transfer_index]->buffer,
				.buffer_length = TRANSFER_BUFFER_SIZE,
				.valid_length = TRANSFER_BUFFER_SIZE,
				.rx_ctx = device->rx_ctx,
				.tx_ctx = device->tx_ctx,
			};
			if ((device->callback(&transfer) == 0) &&
			    (transfer.valid_length > 0)) {
				device->transfers[transfer_index]->length =
					transfer.valid_length;
				ready_transfers++;
			} else {
				break;
			}
		}

	} else {
		// For RX, all transfers are already ready for use.
		ready_transfers = TRANSFER_COUNT;
	}

	// Now everything is ready, go ahead and submit the ready transfers. We must hold
	// the transfer lock whilst doing this, so that completion callbacks cannot resubmit
	// any transfers until all transfers have been initially submitted.
	pthread_mutex_lock(&device->transfer_lock);

	for (transfer_index = 0; transfer_index < ready_transfers; transfer_index++) {
		struct libusb_transfer* transfer = device->transfers[transfer_index];
		transfer->endpoint = endpoint_address;
		transfer->callback = callback;

		// Pad the size of a short transfer to the next 512-byte boundary.
		if (endpoint_address == TX_ENDPOINT_ADDRESS) {
			while (transfer->length % 512 != 0)
				transfer->buffer[transfer->length++] = 0;
		}

		error = libusb_submit_transfer(transfer);
		if (error != 0) {
			last_libusb_error = error;
			break;
		}
		device->active_transfers++;
	}

	if (error == 0) {
		// We should only continue streaming if all transfers were made ready
		// and submitted above. Otherwise, set streaming to false so that the
		// libusb completion callback won't submit further transfers.
		device->streaming = (ready_transfers == TRANSFER_COUNT);
		device->transfers_setup = true;

		// If we're not continuing streaming, follow up with a flush if needed.
		if (!device->streaming && device->flush) {
			error = libusb_submit_transfer(device->flush_transfer);
			if (error != 0) {
				last_libusb_error = error;
			}
		}
	}

	// Now we can release the transfer lock.
	pthread_mutex_unlock(&device->transfer_lock);

	if (error == 0) {
		return TIMSSDR_SUCCESS;
	} else {
		return TIMSSDR_ERROR_LIBUSB;
	}
}

static int prepare_setup_transfers(timssdr_device* device, const uint8_t endpoint_address,timssdr_sample_block_cb_fn callback)
{
	if (device->transfers_setup == true) {
		return TIMSSDR_ERROR_BUSY;
	}

	device->callback = callback;
	return prepare_transfers(
		device,
		endpoint_address,
		timssdr_libusb_transfer_callback);
}

int timssdr_init()
{
	int libusb_error;
	if (g_libusb_context != NULL) {
		return TIMSSDR_SUCCESS;
	}

	libusb_error = libusb_init(&g_libusb_context);
	if (libusb_error != 0) {
		last_libusb_error = libusb_error;
		return TIMSSDR_ERROR_LIBUSB;
	} else {
		return TIMSSDR_SUCCESS;
	}
}

int timssdr_exit(void)
{
	if (open_devices == 0) {
		if (g_libusb_context != NULL) {
			libusb_exit(g_libusb_context);
			g_libusb_context = NULL;
		}

		return TIMSSDR_SUCCESS;
	} else {
		return TIMSSDR_ERROR_NOT_LAST_DEVICE;
	}
}

#ifndef LIBRARY_VERSION
	#define LIBRARY_VERSION "unknown"
#endif
const char* timssdr_library_version()
{
	return LIBRARY_VERSION;
}

timssdr_device_list_t* timssdr_device_list()
{
	int i;
	libusb_device_handle* usb_device = NULL;
	uint8_t serial_descriptor_index;
	char serial_number[64];
	uint8_t idx, serial_number_length;

	timssdr_device_list_t* list = calloc(1, sizeof(*list));
	if (list == NULL)
		return NULL;

	list->usb_devicecount = (int) libusb_get_device_list(
		g_libusb_context,
		(libusb_device***) &list->usb_devices);

	list->serial_numbers = calloc(list->usb_devicecount, sizeof(void*));
	list->usb_board_ids =
		calloc(list->usb_devicecount, sizeof(timssdr_usb_vid));
	list->usb_device_index = calloc(list->usb_devicecount, sizeof(int));

	if (list->serial_numbers == NULL || list->usb_board_ids == NULL ||
	    list->usb_device_index == NULL) {
		timssdr_device_list_free(list);
		return NULL;
	}
	printf("Devices found : \n");
	for (i = 0; i < list->usb_devicecount; i++) {
		struct libusb_device_descriptor device_descriptor;
		libusb_get_device_descriptor(list->usb_devices[i], &device_descriptor);
		printf("Vendor ID : %x, product ID : %x\n",device_descriptor.idVendor, device_descriptor.idProduct);

		if (device_descriptor.idVendor == timssdr_usb_vid) {
			if (device_descriptor.idProduct == timssdr_usb_pid) {
				idx = list->devicecount++;
				list->usb_board_ids[idx] = device_descriptor.idProduct;
				list->usb_device_index[idx] = i;

				serial_descriptor_index = device_descriptor.iSerialNumber;
				if (serial_descriptor_index > 0) {
					if (libusb_open(
						    list->usb_devices[i],
						    &usb_device) != 0) {
						usb_device = NULL;
						continue;
					}
					serial_number_length =
						libusb_get_string_descriptor_ascii(
							usb_device,
							serial_descriptor_index,
							(unsigned char*) serial_number,
							sizeof(serial_number));
					if (serial_number_length >= USB_MAX_SERIAL_LENGTH)
						serial_number_length =
							USB_MAX_SERIAL_LENGTH;

					serial_number[serial_number_length] = 0;
					list->serial_numbers[idx] = strdup(serial_number);

					libusb_close(usb_device);
					usb_device = NULL;
				}
			}
		}
	}

	return list;
}

void timssdr_device_list_free(timssdr_device_list_t* list)
{
	int i;

	libusb_free_device_list((libusb_device**) list->usb_devices, 1);

	for (i = 0; i < list->devicecount; i++) {
		if (list->serial_numbers[i])
			free(list->serial_numbers[i]);
	}

	free(list->serial_numbers);
	free(list->usb_board_ids);
	free(list->usb_device_index);
	free(list);
}

libusb_device_handle* timssdr_open_usb(const char* const desired_serial_number)
{
	libusb_device_handle* usb_device = NULL;
	libusb_device** devices = NULL;
	const ssize_t list_length = libusb_get_device_list(g_libusb_context, &devices);
	ssize_t match_len = 0;
	ssize_t i;
	char serial_number[64];
	int serial_number_length;

	if (desired_serial_number) {
		/* If a shorter serial number is specified, only match against the suffix.
		 * Should probably complain if the match is not unique, currently doesn't.
		 */
		match_len = strlen(desired_serial_number);
		if (match_len > 32)
			return NULL;
	}

	for (i = 0; i < list_length; i++) {
		struct libusb_device_descriptor device_descriptor;
		libusb_get_device_descriptor(devices[i], &device_descriptor);

		if (device_descriptor.idVendor == timssdr_usb_vid) {
			if (device_descriptor.idProduct == timssdr_usb_pid) {
				if (desired_serial_number != NULL) {
					const uint_fast8_t serial_descriptor_index =
						device_descriptor.iSerialNumber;
					if (serial_descriptor_index > 0) {
						if (libusb_open(
							    devices[i],
							    &usb_device) != 0) {
							usb_device = NULL;
							continue;
						}
						serial_number_length =
							libusb_get_string_descriptor_ascii(
								usb_device,
								serial_descriptor_index,
								(unsigned char*)
									serial_number,
								sizeof(serial_number));
						if (serial_number_length >=
						    USB_MAX_SERIAL_LENGTH)
							serial_number_length =
								USB_MAX_SERIAL_LENGTH;
						serial_number[serial_number_length] = 0;
						if (strncmp(serial_number +
								    serial_number_length -
								    match_len,
							    desired_serial_number,
							    match_len) == 0) {
							break;
						} else {
							libusb_close(usb_device);
							usb_device = NULL;
						}
					}
				} else {
					libusb_open(devices[i], &usb_device);
					break;
				}
			}
		}
	}

	libusb_free_device_list(devices, 1);

	return usb_device;
}

static int timssdr_open_setup(libusb_device_handle* usb_device, timssdr_device** device)
{
	int result;
	timssdr_device* lib_device;

	//int speed = libusb_get_device_speed(usb_device);
	// TODO: Error or warning if not high speed USB?

	result = set_timssdr_configuration(usb_device, USB_CONFIG_STANDARD);
	if (result != LIBUSB_SUCCESS) {
		libusb_close(usb_device);
		return result;
	}

	result = libusb_claim_interface(usb_device, 0);
	if (result != LIBUSB_SUCCESS) {
		last_libusb_error = result;
		libusb_close(usb_device);
		return TIMSSDR_ERROR_LIBUSB;
	}

	lib_device = NULL;
	lib_device = (timssdr_device*) calloc(1, sizeof(*lib_device));
	if (lib_device == NULL) {
		libusb_release_interface(usb_device, 0);
		libusb_close(usb_device);
		return TIMSSDR_ERROR_NO_MEM;
	}

	lib_device->usb_device = usb_device;
	lib_device->transfers = NULL;
	lib_device->callback = NULL;
	lib_device->transfer_thread_started = false;
	lib_device->streaming = false;
	lib_device->do_exit = false;
	lib_device->active_transfers = 0;
	lib_device->flush = false;
	lib_device->flush_transfer = NULL;
	lib_device->flush_callback = NULL;
	lib_device->flush_ctx = NULL;
	lib_device->tx_completion_callback = NULL;

	result = pthread_mutex_init(&lib_device->transfer_lock, NULL);
	if (result != 0) {
		free(lib_device);
		libusb_release_interface(usb_device, 0);
		libusb_close(usb_device);
		return TIMSSDR_ERROR_THREAD;
	}

	result = pthread_cond_init(&lib_device->all_finished_cv, NULL);
	if (result != 0) {
		free(lib_device);
		libusb_release_interface(usb_device, 0);
		libusb_close(usb_device);
		return TIMSSDR_ERROR_THREAD;
	}

	result = allocate_transfers(lib_device);
	if (result != 0) {
		free(lib_device);
		libusb_release_interface(usb_device, 0);
		libusb_close(usb_device);
		return TIMSSDR_ERROR_NO_MEM;
	}

	result = create_transfer_thread(lib_device);
	if (result != 0) {
		free(lib_device);
		libusb_release_interface(usb_device, 0);
		libusb_close(usb_device);
		return result;
	}

	*device = lib_device;
	open_devices++;

	return TIMSSDR_SUCCESS;
}

int timssdr_open(timssdr_device** device)
{
	libusb_device_handle* usb_device;

	if (device == NULL) {
		return TIMSSDR_ERROR_INVALID_PARAM;
	}

	usb_device = libusb_open_device_with_vid_pid(
		g_libusb_context,
		timssdr_usb_vid,
		timssdr_usb_pid);

	if (usb_device == NULL) {
		return TIMSSDR_ERROR_NOT_FOUND;
	}

	return timssdr_open_setup(usb_device, device);
}

int timssdr_open_by_serial(const char* const desired_serial_number, timssdr_device** device)
{
	libusb_device_handle* usb_device;

	if (desired_serial_number == NULL) {
		return timssdr_open(device);
	}

	if (device == NULL) {
		return TIMSSDR_ERROR_INVALID_PARAM;
	}

	usb_device = timssdr_open_usb(desired_serial_number);

	if (usb_device == NULL) {
		return TIMSSDR_ERROR_NOT_FOUND;
	}

	return timssdr_open_setup(usb_device, device);
}

int timssdr_device_list_open(timssdr_device_list_t* list, int idx, timssdr_device** device)
{
	libusb_device_handle* usb_device;
	int i, result;

	if (device == NULL || list == NULL || idx < 0 || idx >= list->devicecount) {
		return TIMSSDR_ERROR_INVALID_PARAM;
	}

	i = list->usb_device_index[idx];

	result = libusb_open(list->usb_devices[i], &usb_device);
	if (result != 0) {
		usb_device = NULL;
		last_libusb_error = result;
		return TIMSSDR_ERROR_LIBUSB;
	}

	return timssdr_open_setup(usb_device, device);
}

int timssdr_close(timssdr_device* device)
{
	int result1, result2;

	result1 = TIMSSDR_SUCCESS;
	result2 = TIMSSDR_SUCCESS;

	if (device != NULL) {
		/*
		 * Finally kill the transfer thread, which will
		 * also cancel any pending transmit/receive transfers.
		 */
		result2 = kill_transfer_thread(device);
		if (device->usb_device != NULL) {
			libusb_release_interface(device->usb_device, 0);
			libusb_close(device->usb_device);
			device->usb_device = NULL;
		}

		free_transfers(device);

		pthread_mutex_destroy(&device->transfer_lock);
		pthread_cond_destroy(&device->all_finished_cv);

		free(device);
	}
	open_devices--;

	if (result2 != TIMSSDR_SUCCESS) {
		return result2;
	}
	return result1;
}

int timssdr_is_streaming(timssdr_device* device)
{
	/* return timssdr is streaming only when streaming, transfer_thread_started are true and do_exit equal false */

	if ((device->transfer_thread_started == true) && (device->streaming == true) &&
	    (device->do_exit == false)) {
		return TIMSSDR_TRUE;
	} else {
		if (device->transfer_thread_started == false) {
			return TIMSSDR_ERROR_STREAMING_THREAD_ERR;
		}

		if (device->streaming == false) {
			return TIMSSDR_ERROR_STREAMING_STOPPED;
		}

		return TIMSSDR_ERROR_STREAMING_EXIT_CALLED;
	}
}

int timssdr_start_rx(timssdr_device* device, timssdr_sample_block_cb_fn callback, void* rx_ctx)
{
	int result;
	const uint8_t endpoint_address = RX_ENDPOINT_ADDRESS;
	device->rx_ctx = rx_ctx;
    result = prepare_setup_transfers(device, endpoint_address, callback);
	return result;
}

int timssdr_stop_rx(timssdr_device* device)
{
	int result;

	result = cancel_transfers(device);
	if (result != TIMSSDR_SUCCESS) {
		return result;
	}

	// return timssdr_stop_cmd(device);
	return result;
}

int timssdr_start_tx(timssdr_device* device, timssdr_sample_block_cb_fn callback, void* tx_ctx)
{
	int result;
	const uint8_t endpoint_address = TX_ENDPOINT_ADDRESS;
	if (device->flush_transfer != NULL) {
		device->flush = true;
	}
    device->tx_ctx = tx_ctx;
    result = prepare_setup_transfers(device, endpoint_address, callback);
	return result;
}

int timssdr_set_tx_block_complete_callback(timssdr_device* device, timssdr_tx_block_complete_cb_fn callback)
{
	device->tx_completion_callback = callback;
	return TIMSSDR_SUCCESS;
}

int timssdr_enable_tx_flush(timssdr_device* device, timssdr_flush_cb_fn callback, void* flush_ctx)
{
	device->flush_callback = callback;
	device->flush_ctx = flush_ctx;

	if (device->flush_transfer) {
		return TIMSSDR_SUCCESS;
	}

	if ((device->flush_transfer = libusb_alloc_transfer(0)) == NULL) {
		return TIMSSDR_ERROR_LIBUSB;
	}

	libusb_fill_bulk_transfer(
		device->flush_transfer,
		device->usb_device,
		TX_ENDPOINT_ADDRESS,
		calloc(1, DEVICE_BUFFER_SIZE),
		DEVICE_BUFFER_SIZE,
		timssdr_libusb_flush_callback,
		device,
		0);

	device->flush_transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

	return TIMSSDR_SUCCESS;
}

int timssdr_disable_tx_flush(timssdr_device* device)
{
	libusb_free_transfer(device->flush_transfer);
	device->flush_transfer = NULL;
	device->flush_callback = NULL;
	device->flush_ctx = NULL;

	return TIMSSDR_SUCCESS;
}

int timssdr_stop_tx(timssdr_device* device)
{
	int result;
	result = cancel_transfers(device);
	if (result != TIMSSDR_SUCCESS) {
		return result;
	}

	// return timssdr_stop_cmd(device);
	return result;
}

const char* timssdr_error_name(enum timssdr_error errcode)
{
	switch (errcode) {
	case TIMSSDR_SUCCESS:
		return "TIMSSDR_SUCCESS";

	case TIMSSDR_TRUE:
		return "TIMSSDR_TRUE";

	case TIMSSDR_ERROR_INVALID_PARAM:
		return "invalid parameter(s)";

	case TIMSSDR_ERROR_NOT_FOUND:
		return "TimsSDR not found";

	case TIMSSDR_ERROR_BUSY:
		return "TimsSDR busy";

	case TIMSSDR_ERROR_NO_MEM:
		return "insufficient memory";

	case TIMSSDR_ERROR_LIBUSB:
#if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x01000103)
		if (last_libusb_error != LIBUSB_SUCCESS)
			return libusb_strerror(last_libusb_error);
#endif
		return "USB error";

	case TIMSSDR_ERROR_THREAD:
		return "transfer thread error";

	case TIMSSDR_ERROR_STREAMING_THREAD_ERR:
		return "streaming thread encountered an error";

	case TIMSSDR_ERROR_STREAMING_STOPPED:
		return "streaming stopped";

	case TIMSSDR_ERROR_STREAMING_EXIT_CALLED:
		return "streaming terminated";

	case TIMSSDR_ERROR_NOT_LAST_DEVICE:
		return "one or more TimsSDRs still in use";

	case TIMSSDR_ERROR_OTHER:
		return "unspecified error";

	default:
		return "unknown error code";
	}
}

int timssdr_board_partid_serialno_read(
	timssdr_device* device,
	read_partid_serialno_t* read_partid_serialno)
{
	// TODO implement to read part and device id from FTDI, currently replying static value
	read_partid_serialno->part_id[0] = 0x60;
	read_partid_serialno->part_id[1] = 0x14;
	read_partid_serialno->serial_no[0] = 0x0;
	read_partid_serialno->serial_no[1] = 0x0;
	read_partid_serialno->serial_no[2] = 0x0;
	read_partid_serialno->serial_no[3] = 0x0;

	return TIMSSDR_SUCCESS;
}
