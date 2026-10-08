#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "board_config.h"
#include "dc_motor_pwm.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "can2pwm";

#define MOTOR_COMMAND_CAN_ID 0x100
#define MOTOR_COMMAND_TIMEOUT_US 100000
#define MOTOR_COMMAND_QUEUE_LENGTH 1

typedef struct
{
	int16_t speed[DC_MOTOR_PWM_CHANNEL_COUNT];
	bool is_single_motor;
	uint8_t channel;
	int16_t requested_speed;
} motor_command_t;

static QueueHandle_t g_motor_command_queue;
static int16_t g_motor_speeds[DC_MOTOR_PWM_CHANNEL_COUNT];

static esp_err_t initialize_can_power_pins(void)
{
	const uint64_t pin_mask = (1ULL << CAN_GND_GPIO) | (1ULL << CAN_VCC_GPIO);
	gpio_config_t io_config = {
		.pin_bit_mask = pin_mask,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	esp_err_t err = gpio_config(&io_config);

	gpio_set_drive_capability(CAN_GND_GPIO, GPIO_DRIVE_CAP_3);
	gpio_set_drive_capability(CAN_VCC_GPIO, GPIO_DRIVE_CAP_3);
	if (err != ESP_OK)
	{
		return err;
	}

	err = gpio_set_level(CAN_GND_GPIO, 0);
	if (err != ESP_OK)
	{
		return err;
	}

	return gpio_set_level(CAN_VCC_GPIO, 1);
}

static bool decode_motor_command(const twai_frame_t *frame, motor_command_t *command)
{
	if (frame->header.ide || frame->header.rtr || frame->header.fdf ||
		frame->header.id != MOTOR_COMMAND_CAN_ID)
	{
		return false;
	}

	if (frame->header.dlc == DC_MOTOR_PWM_CHANNEL_COUNT)
	{
		for (size_t i = 0; i < DC_MOTOR_PWM_CHANNEL_COUNT; ++i)
		{
			uint8_t raw_speed = frame->buffer[i];
			int16_t speed = raw_speed < 0x80 ? raw_speed : (int16_t)raw_speed - 256;
			if (speed < -100 || speed > 100)
			{
				return false;
			}
			command->speed[i] = speed;
		}
		command->is_single_motor = false;
		return true;
	}

	return false;
}

static void update_motor_setpoint_snapshot(const motor_command_t *command)
{
	for (size_t i = 0; i < DC_MOTOR_PWM_CHANNEL_COUNT; ++i)
	{
		g_motor_speeds[i] = command->speed[i];
	}
}

static bool twai_rx_done_callback(twai_node_handle_t node,
								  const twai_rx_done_event_data_t *event_data,
								  void *user_ctx)
{
	(void)event_data;
	(void)user_ctx;

	uint8_t rx_data[TWAI_FRAME_MAX_LEN];
	twai_frame_t frame = {
		.buffer = rx_data,
		.buffer_len = sizeof(rx_data),
	};
	if (twai_node_receive_from_isr(node, &frame) != ESP_OK)
	{
		return false;
	}

	motor_command_t command;
	if (!decode_motor_command(&frame, &command))
	{
		return false;
	}
	update_motor_setpoint_snapshot(&command);

	BaseType_t higher_priority_task_woken = pdFALSE;
	xQueueOverwriteFromISR(g_motor_command_queue, &command, &higher_priority_task_woken);
	return higher_priority_task_woken == pdTRUE;
}

static esp_err_t initialize_twai(twai_node_handle_t *node)
{
	twai_onchip_node_config_t node_config = {
		.io_cfg = {
			.tx = CAN_TX_GPIO,
			.rx = CAN_RX_GPIO,
			.quanta_clk_out = -1,
			.bus_off_indicator = -1,
		},
		.bit_timing = {
			.bitrate = CAN_BITRATE,
		},
		.tx_queue_depth = 1,
		.fail_retry_cnt = -1,
		.flags = {
			.no_receive_rtr = 1,
		},
	};

	esp_err_t err = twai_new_node_onchip(&node_config, node);
	if (err != ESP_OK)
	{
		return err;
	}

	const twai_event_callbacks_t callbacks = {
		.on_rx_done = twai_rx_done_callback,
	};
	err = twai_node_register_event_callbacks(*node, &callbacks, NULL);
	if (err != ESP_OK)
	{
		twai_node_delete(*node);
		*node = NULL;
		return err;
	}

	err = twai_node_enable(*node);
	if (err != ESP_OK)
	{
		twai_node_delete(*node);
		*node = NULL;
	}
	return err;
}

void app_main(void)
{
	dc_motor_pwm_init();

	esp_err_t err = initialize_can_power_pins();
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "CAN GPIO initialization failed: %s", esp_err_to_name(err));
		return;
	}

	g_motor_command_queue = xQueueCreate(MOTOR_COMMAND_QUEUE_LENGTH, sizeof(motor_command_t));
	if (g_motor_command_queue == NULL)
	{
		ESP_LOGE(TAG, "Failed to create motor command queue");
		return;
	}

	twai_node_handle_t node = NULL;
	err = initialize_twai(&node);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "TWAI node initialization failed: %s", esp_err_to_name(err));
		vQueueDelete(g_motor_command_queue);
		g_motor_command_queue = NULL;
		return;
	}

	ESP_LOGI(TAG, "TWAI receiver ready at %d bit/s", CAN_BITRATE);

	int64_t last_command_us = esp_timer_get_time();
	bool timeout_stop_applied = false;

	while (true)
	{
		motor_command_t command;
		if (xQueueReceive(g_motor_command_queue, &command, pdMS_TO_TICKS(100)) == pdTRUE)
		{
			if (!command.is_single_motor)
			{
				// ESP_LOGI(TAG, "Decoded all-motor command");
			}

			for (uint8_t channel = 0; channel < DC_MOTOR_PWM_CHANNEL_COUNT; ++channel)
			{
				dc_motor_pwm_set_speed(channel, command.speed[channel]);
				if (!command.is_single_motor)
				{
					// ESP_LOGI(TAG, "Motor %u speed command: %d%%", channel, command.speed[channel]);
				}
			}
			last_command_us = esp_timer_get_time();
			timeout_stop_applied = false;
		}

		if (!timeout_stop_applied &&
			esp_timer_get_time() - last_command_us >= MOTOR_COMMAND_TIMEOUT_US)
		{
			dc_motor_pwm_stop_all();
			timeout_stop_applied = true;
			ESP_LOGW(TAG, "No valid motor command received; motors stopped");
		}
	}
}