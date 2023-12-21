#ifndef TIMSSDR_H
#define TIMSSDR_H

#include <stdlib.h>
#include <stdint.h>
#include <libusb-1.0/libusb.h>

#define TIMSSDR_VENDOR_ID 0x0403
#define TIMSSDR_PRODUCT_ID 0x6014

#define TIMSSDR_USB_TIMEOUT 4000

#define TIMSSDR_TX_OUT 0x02
#define TIMSSDR_RX_IN 0x82

enum timssdr_error
{
    TIMSSDR_SUCCESS = 0,
    TIMSSDR_TRUE,
    TIMSSDR_ERROR_INVALID_PARAM,
    TIMSSDR_ERROR_NOT_FOUND,
    TIMSSDR_ERROR_LIBUSB,
    TIMSSDR_ERROR_NOT_LAST_DEVICE,
    TIMSSDR_ERROR_NO_MEM,
    TIMSSDR_ERROR_THREAD,
    TIMSSDR_ERROR_BUSY,
    TIMSSDR_ERROR_OTHER,
    TIMSSDR_ERROR_STREAMING_THREAD_ERR,
    TIMSSDR_ERROR_STREAMING_STOPPED,
    TIMSSDR_ERROR_STREAMING_EXIT_CALLED
};

typedef struct timssdr_device timssdr_device;

typedef struct {
	/**
	 * MCU part ID register value
	*/
	uint32_t part_id[2];
	/**
	 * MCU device unique ID (serial number)
	*/
	uint32_t serial_no[4];
} read_partid_serialno_t;

typedef struct {
	/** TimsSDR USB device for this transfer */
	timssdr_device* device;
	/** transfer data buffer (interleaved 8 bit I/Q samples) */
	uint8_t* buffer;
	/** length of data buffer in bytes */
	int buffer_length;
	/** number of buffer bytes that were transferred */
	int valid_length;
	/** User provided RX context. Not used by the library, but available to transfer callbacks for use. Set along with the transfer callback using @ref timssdr_start_rx or @ref timssdr_start_rx_sweep */
	void* rx_ctx;
	/** User provided TX context. Not used by the library, but available to transfer callbacks for use. Set along with the transfer callback using @ref timssdr_start_tx*/
	void* tx_ctx;
} timssdr_transfer;

enum timssdr_usb_board_id {
	/**
	 * F232R product ID
	 */
	USB_BOARD_ID_F232R = TIMSSDR_PRODUCT_ID,
};

struct timssdr_device_list {
	/**
	 * Array of human-readable serial numbers. Each entry can be NULL!
	 */
	char** serial_numbers;
	/**
	 * ID of each board, based on USB product ID. Can be used for general HW identification without opening the device.
	 */
	enum timssdr_usb_board_id* usb_board_ids;
	/**
	 * USB device index for a given HW entry. Intended for internal use only.
	 */
	int* usb_device_index;

	/**
	 * Number of connected TimsSDR devices, the length of arrays @ref serial_numbers, @ref usb_board_ids and @ref usb_device_index.
	 */
	int devicecount;

	/**
	 * All USB devices (as `libusb_device**` array)
	 */
	void** usb_devices;
	/**
	 * Number of all queried USB devices. Length of array @ref usb_devices.
	 */
	int usb_devicecount;
};

typedef struct timssdr_device_list timssdr_device_list_t;

/**
 * Sample block callback, used in RX and TX (set via @ref timssdr_start_rx, @ref timssdr_start_rx_sweep and @ref timssdr_start_tx). In each mode, it is called when data needs to be handled, meaning filling samples in TX mode or reading them in RX modes.
 * 
 * In TX mode, it should refill the transfer buffer with new raw IQ data, and set @ref timssdr_transfer.valid_length.
 * 
 * In RX mode, it should copy/process the contents of the transfer buffer's valid part.
 * 
 * In RX SWEEP mode, it receives multiple "blocks" of data, each with a 10-byte header containing the tuned frequency followed by the samples. See @ref timssdr_init_sweep for more info.
 * 
 * The callback should return 0 if it wants to be called again, and any other value otherwise. Stopping the RX/TX/SWEEP is still done with @ref timssdr_stop_rx and @ref timssdr_stop_tx, and those should be called from the main thread, so this callback should signal the main thread that it should stop. Signaling the main thread to stop TX should be done from the flush callback in order to guarantee that no samples are discarded, see @ref timssdr_flush_cb_fn
 * @ingroup streaming
 */
typedef int (*timssdr_sample_block_cb_fn)(timssdr_transfer* transfer);

/**
 * Block complete callback. 
 * 
 * Set via @ref timssdr_set_tx_block_complete_callback, called when a transfer is finished to the device's buffer, regardless if the transfer was successful or not. It can signal the main thread to stop on failure, can catch USB transfer errors and can also gather statistics about the transfered data.
 * @ingroup streaming
 */
typedef void (*timssdr_tx_block_complete_cb_fn)(timssdr_transfer* transfer, int);

/**
 * Flush (end of transmission) callback
 * 
 * Will be called when the last samples are transmitted and stopping transmission will result in no samples getting lost. Should signal the main thread that it should stop transmission via @ref timssdr_stop_tx
 * @ingroup streaming
 */
typedef void (*timssdr_flush_cb_fn)(void* flush_ctx, int);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize libtimssdr
 * 
 * Should be called before any other libtimssdr function. Initializes libusb. Can be safely called multiple times.
 * @return @ref TIMSSDR_SUCCESS on success or @ref TIMSSDR_ERROR_LIBUSB
 * @ingroup library
 */
extern int timssdr_init();

/**
 * Exit libtimssdr
 * 
 * Should be called before exit. No other libtimssdr functions should be called after it. Can be safely called multiple times.
 * @return @ref TIMSSDR_SUCCESS on success or @ref TIMSSDR_ERROR_NOT_LAST_DEVICE if not all devices were closed properly.
 * @ingroup library
 */
extern int timssdr_exit();

/**
 * Get library version.
 * 
 * Can be called before @ref timssdr_init
 * @return library version as a human-readable string
 * @ingroup library
 */
extern const char* timssdr_library_version();

/**
 * List connected TimsSDR devices
 * @return list of connected devices. The list should be freed with @ref timssdr_device_list_free
 * @ingroup device
 */
extern timssdr_device_list_t* timssdr_device_list();

/**
 * Open a @ref timssdr_device from a device list
 * @param[in] list device list to open device from
 * @param[in] idx index of the device to open
 * @param[out] device device handle to open
 * @return @ref TIMSSDR_SUCCESS on success, @ref TIMSSDR_ERROR_INVALID_PARAM on invalid parameters or other @ref timssdr_error variant
 * @ingroup device
 */
extern int timssdr_device_list_open(
	timssdr_device_list_t* list,
	int idx,
	timssdr_device** device);

/**
 * Free a previously allocated @ref timssdr_device_list list.
 * @param[in] list list to free
 * @ingroup device
 */
extern void timssdr_device_list_free(timssdr_device_list_t* list);

/**
 * Open first available TimsSDR device
 * @param[out] device device handle
 * @return @ref TIMSSDR_SUCCESS on success, @ref TIMSSDR_ERROR_INVALID_PARAM if @p device is NULL, @ref TIMSSDR_ERROR_NOT_FOUND if no TimsSDR devices are found or other @ref timssdr_error variant
 * @ingroup device
 */
extern int timssdr_open(timssdr_device** device);

/**
 * Open TimsSDR device by serial number
 * @param[in] desired_serial_number serial number of device to open. If NULL then default to first device found.
 * @param[out] device device handle
 * @return @ref TIMSSDR_SUCCESS on success, @ref TIMSSDR_ERROR_INVALID_PARAM if @p device is NULL, @ref TIMSSDR_ERROR_NOT_FOUND if no TimsSDR devices are found or other @ref timssdr_error variant
 * @ingroup device
 */
extern int timssdr_open_by_serial(
	const char* const desired_serial_number,
	timssdr_device** device);

/**
 * Close a previously opened device
 * @param[in] device device to close
 * @return @ref TIMSSDR_SUCCESS on success or variant of @ref timssdr_error
 * @ingroup device
 */
extern int timssdr_close(timssdr_device* device);

/**
 * Start receiving
 * 
 * Should be called after setting gains, frequency and sampling rate, as these values won't get reset but instead keep their last value, thus their state is unknown.
 * 
 * The callback is called with a @ref timssdr_transfer object whenever the buffer is full. The callback is called in an async context so no libtimssdr functions should be called from it. The callback should treat its argument as read-only.
 * @param device device to configure
 * @param callback rx_callback
 * @param rx_ctx User provided RX context. Not used by the library, but available to @p callback as @ref timssdr_transfer.rx_ctx.
 * @return @ref TIMSSDR_SUCCESS on success or @ref timssdr_error variant
 * @ingroup streaming
 */
extern int timssdr_start_rx(
	timssdr_device* device,
	timssdr_sample_block_cb_fn callback,
	void* rx_ctx);

/**
 * Stop receiving
 * 
 * @param device device to stop RX on
 * @return @ref TIMSSDR_SUCCESS on success or @ref timssdr_error variant
 * @ingroup streaming
 */
extern int timssdr_stop_rx(timssdr_device* device);

/**
 * Start transmitting
 * 
 *
 * Should be called after setting gains, frequency and sampling rate, as these values won't get reset but instead keep their last value, thus their state is unknown. 
 * Setting flush function (using @ref timssdr_enable_tx_flush) and/or setting block complete callback (using @ref timssdr_set_tx_block_complete_callback) (if these features are used) should also be done before this.
 * 
 * The callback is called with a @ref timssdr_transfer object whenever a transfer buffer is needed to be filled with samples. The callback is called in an async context so no libtimssdr functions should be called from it. The callback should treat its argument as read-only, except the @ref timssdr_transfer.buffer and @ref timssdr_transfer.valid_length.
 * @param device device to configure
 * @param callback tx_callback
 * @param tx_ctx User provided TX context. Not used by the library, but available to @p callback as @ref timssdr_transfer.tx_ctx.
 * @return @ref TIMSSDR_SUCCESS on success or @ref timssdr_error variant
 * @ingroup streaming
 */
extern int timssdr_start_tx(
	timssdr_device* device,
	timssdr_sample_block_cb_fn callback,
	void* tx_ctx);

/**
 * Setup callback to be called when an USB transfer is completed.
 * 
 * This callback will be called whenever an USB transfer to the device is completed, regardless if it was successful or not (indicated by the second parameter).
 * 
 * @param device device to configure
 * @param callback callback to call when a transfer is completed
 * @return @ref TIMSSDR_SUCCESS on success or @ref timssdr_error variant 
 * @ingroup streaming
 */
extern int timssdr_set_tx_block_complete_callback(
	timssdr_device* device,
	timssdr_tx_block_complete_cb_fn callback);

/**
 * Setup flush (end-of-transmission) callback
 * 
 * This callback will be called when all the data was transmitted and all data transfers were completed. First parameter is supplied context, second parameter is success flag.
 * 
 * @param device device to configure
 * @param callback callback to call when all transfers were completed
 * @param flush_ctx context (1st parameter of callback)
 * @return @ref TIMSSDR_SUCCESS on success or @ref timssdr_error variant 
 * @ingroup streaming
 */
extern int timssdr_enable_tx_flush(
	timssdr_device* device,
	timssdr_flush_cb_fn callback,
	void* flush_ctx);

/**
 * Stop transmission
 * 
 * @param device device to stop TX on
 * @return @ref TIMSSDR_SUCCESS on success or @ref timssdr_error variant 
 * @ingroup streaming
 */
extern int timssdr_stop_tx(timssdr_device* device);

/**
 * Get Error details
 * 
 * @param error device to stop TX on
 * @return @ref TIMSSDR_SUCCESS on success or @ref timssdr_error variant 
 * @ingroup streaming
 */
extern const char* timssdr_error_name(enum timssdr_error error);

/**
 * Query device streaming status
 * 
 * @param device device to query
 * @return @ref TIMSSDR_TRUE if the device is streaming, else one of @ref TIMSSDR_ERROR_STREAMING_THREAD_ERR, @ref TIMSSDR_ERROR_STREAMING_STOPPED or @ref TIMSSDR_ERROR_STREAMING_EXIT_CALLED
 * @ingroup streaming
 */
extern int timssdr_is_streaming(timssdr_device* device);

/**
 * Read board part ID and serial number
 * 
 * Read MCU part id and serial number. See the documentation of the MCU for details!
 * 
 * @param[in] device device to query
 * @param[out] read_partid_serialno result of query
 * @return @ref HACKRF_SUCCESS on success or @ref hackrf_error variant
 * @ingroup device
 */
extern int timssdr_board_partid_serialno_read(
	timssdr_device* device,
	read_partid_serialno_t* read_partid_serialno);

#ifdef __cplusplus
}
#endif

#endif