/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2024 GammaKinematics - UART to ESB Bridge
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif

LOG_MODULE_REGISTER(esb_uart_bridge, CONFIG_ESB_PTX_APP_LOG_LEVEL);

/* HID packet structure for UART→ESB transport */
struct hid_packet_header {
    uint8_t type;      /* HID report type (1=keyboard, 2=consumer, 3=mouse) */
    uint8_t length;    /* Payload length */
} __packed;

#define HID_TYPE_KEYBOARD   1
#define HID_TYPE_CONSUMER   2  
#define HID_TYPE_MOUSE      3
#define MAX_HID_PAYLOAD     28  /* ESB max (32) - header (4) */

/* UART device from device tree */
static const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_bt_c2h_uart));

/* UART ring buffer for incoming data */
#define UART_BUF_SIZE 512
static uint8_t uart_buffer[UART_BUF_SIZE];
static struct ring_buf uart_ring_buf;

/* ESB payload for forwarding */
static struct esb_payload tx_payload;
static bool esb_ready = true;

/* ESB event handler */
void event_handler(struct esb_evt const *event)
{
    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
        LOG_DBG("ESB TX success");
        esb_ready = true;
        break;
    case ESB_EVENT_TX_FAILED:
        LOG_WRN("ESB TX failed");
        esb_ready = true;  /* Try again anyway */
        break;
    case ESB_EVENT_RX_RECEIVED:
        /* Not expecting RX in PTX mode */
        break;
    }
}

/* UART interrupt handler */
static void uart_isr(const struct device *dev, void *user_data)
{
    uint8_t byte;
    
    if (!uart_irq_update(dev)) {
        return;
    }
    
    if (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &byte, 1) == 1) {
            ring_buf_put(&uart_ring_buf, &byte, 1);
        }
    }
}

/* Process received HID packet from UART */
static void process_hid_packet(const uint8_t *data, size_t len)
{
    if (len < sizeof(struct hid_packet_header)) {
        LOG_WRN("Packet too short: %zu", len);
        return;
    }
    
    struct hid_packet_header *header = (struct hid_packet_header *)data;
    
    if (header->length > MAX_HID_PAYLOAD) {
        LOG_WRN("Payload too large: %d", header->length);
        return;
    }
    
    if (len != sizeof(struct hid_packet_header) + header->length) {
        LOG_WRN("Length mismatch: got %zu, expected %zu", 
                len, sizeof(struct hid_packet_header) + header->length);
        return;
    }
    
    /* Package into ESB payload */
    tx_payload.length = len;
    tx_payload.pipe = 0;
    tx_payload.noack = false;
    memcpy(tx_payload.data, data, len);
    
    /* Send via ESB if ready */
    if (esb_ready) {
        esb_ready = false;
        int err = esb_write_payload(&tx_payload);
        if (err) {
            LOG_ERR("ESB write failed: %d", err);
            esb_ready = true;
        } else {
            LOG_DBG("Forwarded HID packet: type=%d, len=%d", 
                    header->type, header->length);
        }
    } else {
        LOG_WRN("ESB busy, dropping packet");
    }
}

/* UART initialization */
static int uart_init(void)
{
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    }
    
    /* Initialize ring buffer */
    ring_buf_init(&uart_ring_buf, sizeof(uart_buffer), uart_buffer);
    
    /* Configure UART interrupts */
    uart_irq_callback_set(uart_dev, uart_isr);
    uart_irq_rx_enable(uart_dev);
    
    LOG_INF("UART initialized for PRIM communication");
    return 0;
}

/* Keep existing clock initialization functions */
#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

#elif defined(CONFIG_CLOCK_CONTROL_NRF2)

int clocks_start(void)
{
	int err;
	int res;
	const struct device *radio_clk_dev =
		DEVICE_DT_GET_OR_NULL(DT_CLOCKS_CTLR(DT_NODELABEL(radio)));
	struct onoff_client radio_cli;

	nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_1, true);

	sys_notify_init_spinwait(&radio_cli.notify);

	err = nrf_clock_control_request(radio_clk_dev, NULL, &radio_cli);

	do {
		err = sys_notify_fetch_result(&radio_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err == -EAGAIN);

	nrf_lrcconf_clock_always_run_force_set(NRF_LRCCONF000, 0, true);
	nrf_lrcconf_task_trigger(NRF_LRCCONF000, NRF_LRCCONF_TASK_CLKSTART_0);

	LOG_DBG("HF clock started");
	return 0;
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */

/* Keep existing ESB initialization */
int esb_initialize(void)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.retransmit_delay = 600;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.event_handler = event_handler;
	config.mode = ESB_MODE_PTX;
	config.selective_auto_ack = true;
	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);

	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	return 0;
}

/* New main function with UART bridge */
int main(void)
{
    int err;
    
    LOG_INF("BLESB UART→ESB Bridge starting");
    
    /* Initialize clocks */
    err = clocks_start();
    if (err) {
        LOG_ERR("Clock start failed: %d", err);
        return 0;
    }
    
    /* Initialize ESB */
    err = esb_initialize();
    if (err) {
        LOG_ERR("ESB initialization failed: %d", err);
        return 0;
    }
    
    /* Initialize UART */
    err = uart_init();
    if (err) {
        LOG_ERR("UART initialization failed: %d", err);
        return 0;
    }
    
    LOG_INF("Bridge initialization complete");
    
    /* Main packet processing loop */
    uint8_t packet_buffer[64];
    size_t packet_len = 0;
    
    while (1) {
        /* Check for complete packets in ring buffer */
        if (ring_buf_size_get(&uart_ring_buf) >= sizeof(struct hid_packet_header)) {
            struct hid_packet_header header;
            
            /* Peek at header */
            if (ring_buf_peek(&uart_ring_buf, (uint8_t *)&header, sizeof(header)) == sizeof(header)) {
                size_t total_len = sizeof(header) + header.length;
                
                /* Check if complete packet is available */
                if (ring_buf_size_get(&uart_ring_buf) >= total_len && total_len <= sizeof(packet_buffer)) {
                    /* Get complete packet */
                    packet_len = ring_buf_get(&uart_ring_buf, packet_buffer, total_len);
                    
                    if (packet_len == total_len) {
                        process_hid_packet(packet_buffer, packet_len);
                    }
                }
            }
        }
        
        /* Small delay to prevent busy waiting */
        k_sleep(K_USEC(100));
    }
    
    return 0;
}