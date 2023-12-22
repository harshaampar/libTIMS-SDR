#include "include/timssdr.h"
#include "stdio.h"

int main()
{
	libusb_device_handle* usb_device = NULL;

	enum timssdr_error result = timssdr_init();
	if (result != TIMSSDR_SUCCESS) {
		fprintf(stderr,
			"timssdr_init() failed: %s (%d)\n",
			timssdr_error_name(result),
			result);
		return EXIT_FAILURE;
	}

	timssdr_device_list_t* list;
	timssdr_device* device;
    read_partid_serialno_t read_partid_serialno;

	list = timssdr_device_list();

	if (list->devicecount < 1) {
		printf("No TIMSSDR boards found.\n");
		return EXIT_FAILURE;
	}

    for(int i=0;i<list->devicecount;i++)
    {
		printf("Found TIMSSDR\n");
		printf("Index: %d\n", i);

		if (list->serial_numbers[i]) {
			printf("Serial number: %s\n", list->serial_numbers[i]);
		}

		device = NULL;
		result = timssdr_device_list_open(list, i, &device);
		if (result != TIMSSDR_SUCCESS) {
			fprintf(stderr,
				"timssdr_open() failed: %s (%d)\n",
				timssdr_error_name(result),
				result);
			if (result == TIMSSDR_ERROR_LIBUSB) {
				continue;
			}
			return EXIT_FAILURE;
		}


		result = timssdr_board_partid_serialno_read(device, &read_partid_serialno);
		if (result != TIMSSDR_SUCCESS) {
			fprintf(stderr,
				"timssdr_board_partid_serialno_read() failed: %s (%d)\n",
				timssdr_error_name(result),
				result);
			return EXIT_FAILURE;
		}
		printf("Part ID Number: 0x%08x 0x%08x\n",
		       read_partid_serialno.part_id[0],
		       read_partid_serialno.part_id[1]);

		result = timssdr_close(device);
		if (result != TIMSSDR_SUCCESS) {
			fprintf(stderr,
				"timssdr_close() failed: %s (%d)\n",
				timssdr_error_name(result),
				result);
		}
	}

	timssdr_device_list_free(list);
	timssdr_exit();

	return EXIT_SUCCESS;

}