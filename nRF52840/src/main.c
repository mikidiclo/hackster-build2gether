/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <ei_wrapper.h>
#include <string.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/device.h>

#include <zephyr/drivers/uart.h>
#include <drivers/sensor.h>

#include "input_data.h"

/* DMIC */
#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
#define READ_TIMEOUT     1000
#define BLOCK_SIZE(_sample_rate, _number_of_channels) (BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 2)
#define BLOCK_COUNT      4
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);
float audio[16000];
/* UART */
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

/* EI */
#define FRAME_ADD_INTERVAL_MS	100

/* DMIC transfer */
static int do_pdm_transfer(const struct device *dmic_dev, struct dmic_cfg *cfg, size_t block_count) {
	int ret;

	/* Start the microphone. */
	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		printk("START trigger failed: %d", ret);
		return ret;
	}
	
	for (int i = 0; i < 11; ++i) {
		void *buffer;
		uint32_t size;

		ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
		if (ret < 0) {
			printk("Read failed: %d", ret);
			break;
		}

		/* Discard first readout due to microphone needing to 
		 * stabilize before valid data can be read. */
		if(i!=0){
			int16_t tempInt;
			float tempFloat;
			for(int j=0; j<1600; j++){
				memcpy(&tempInt, buffer + 2*j, 2);
				tempFloat = (float)tempInt;
				audio[(i-1)*1600+j] = tempFloat;
			}
		}
		k_mem_slab_free(&mem_slab, &buffer);
	}

	/* Stop the microphone. */
	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		return ret;
	}

	/* Give the microphone data to Edge Impulse. */
	ret = ei_wrapper_add_data(&audio, ei_wrapper_get_window_size());
	if (ret) {
		printk("Cannot provide input data (err: %d)\n", ret);
		printk("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
	}
	ei_wrapper_start_prediction(0,0);

	return 0;
}

/* EI result callback */
static void result_ready_cb(int err)
{
	if (err) {
		printk("Result ready callback returned error (err: %d)\n", err);
		return;
	}

	const char *label;
	int label_count = 0;
	float value;

	printk("Classification results:\n");

	while (true) {
		err = ei_wrapper_get_next_classification_result(&label, &value, NULL); // results are ordered based on decending values -> the first value is the largest

		if (err) {
			if (err == -ENOENT) {
				err = 0;
			}
			break;
		}
		
		// Print out the prediction value
		printk("Value: %.2f\tLabel: %s\n", value, label);
		if (label_count == 0) {
			// Send the highest prediction value
			int ret = uart_tx(uart, label, sizeof(label), SYS_FOREVER_US);
			if (ret) {
				printk("UART sent data failed (err: %d)\n", ret);
				return;
			} 
		};
		label_count++;
	}

	bool cancelled;
	err = ei_wrapper_clear_data(&cancelled);
	if (err) {
		printk("Edge Impulse cannot clear data (err: %i)\n", err);
	}
}

/* Ultrasonic sensor */
static int measure(const struct device *dev)
{
    int ret;
    struct sensor_value distance;

    ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
    switch (ret) {
    case 0:
        ret = sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &distance);
        if (ret) {
            printk("sensor_channel_get failed ret %d", ret);
            return ret;
        }
        printk("%s: %d.%03dM", dev->name, distance.val1, (distance.val2 / 1000));
		// Send data through serial port
		char command[] = "off"
		if (distance.val1 < 3) {
			int ret = uart_tx(uart, command, sizeof(command), SYS_FOREVER_US);
			if (ret) {
				printk("UART sent data failed (err: %d)\n", ret);
				return;
			} 
		}
        break;
    case -EIO:
        printk("%s: Could not read device", dev->name);
        break;
    default:
        printk("Error when reading device: %s", dev->name);
        break;
    }
    return 0;
}


int main(void)
{
	int ret;
	/* Init HC-SR04 */
	const struct device *const us0 = DEVICE_DT_GET(DT_NODELABEL(us0));
	if (!device_is_ready(us0)) {
		printf("HC-SR04 failed to init (err: %s)\n", us0->name);
		return 0;
	}

	/* Check UART init*/
	if (!device_is_ready(uart)) {
		printk("UART failed to init (err: %s)\n", uart->name);
		return 0;
	};

	/* Init DMIC */
	const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));
	if (!device_is_ready(dmic_dev)) {
		printk("DMIC failed to init (err: %s)\n", dmic_dev->name);
		return 0;
	};
	/* Configure DMIC */
	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	cfg.channel.req_num_chan 	= 1;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate 	= MAX_SAMPLE_RATE;
	cfg.streams[0].block_size 	= BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	ret = dmic_configure(dmic_dev, &cfg);
	if (ret < 0) {
		printk("DMIC config error (err: %d)\n", ret);
		return 0;
	}

	/* Init EI */
	ret = ei_wrapper_init(result_ready_cb);
	if (ret) {
		printk("Edge Impulse wrapper failed to initialize (err: %d)\n", ret);
		return 0;
	};

	while (true) {
		ret = do_pdm_transfer(dmic_dev, &cfg, 2 * BLOCK_COUNT);
		if (ret < 0) {
			printk("Error\n");
			k_sleep(K_SECONDS(5));
			return 0;
		}
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
