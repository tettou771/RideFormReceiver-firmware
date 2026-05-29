/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "system/system.h"
#include "build_defines.h"
#include "parse_args.h"
#include "connection/cmd_queue.h"
#include "connection/modem.h"
#include "connection/tdma.h"

#include <stdlib.h>  /* strtof */
#include <math.h>    /* roundf */

#define USB DT_NODELABEL(usbd)
#if DT_NODE_HAS_STATUS(USB, okay)

#include <zephyr/drivers/gpio.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log_ctrl.h>
#include "connection/esb.h"

#include <ctype.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void console_thread(void);
// RFT: 2048 (was 1024) — `modem at` nests console_thread -> modem_console_at
// (line[256]) -> at_send_line (buf[256]) plus a 160B rejoin buffer, which
// overran the 1024B stack and tripped the MPU guard (HW_STACK_PROTECTION),
// rebooting the board on every raw-AT command.
K_THREAD_DEFINE(console_thread_id, 2048, console_thread, NULL, NULL, NULL, 6, 0, 0);

#define DFU_EXISTS CONFIG_BUILD_OUTPUT_UF2 || CONFIG_BOARD_HAS_NRF5_BOOTLOADER
#define ADAFRUIT_BOOTLOADER CONFIG_BUILD_OUTPUT_UF2
#define NRF5_BOOTLOADER CONFIG_BOARD_HAS_NRF5_BOOTLOADER

#if NRF5_BOOTLOADER
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
#endif

static const char *meows[] = {
	"Mew",
	"Meww",
	"Meow",
	"Meow meow",
	"Mrrrp",
	"Mrrf",
	"Mreow",
	"Mrrrow",
	"Mrrr",
	"Purr",
	"mew",
	"meww",
	"meow",
	"meow meow",
	"mrrrp",
	"mrrf",
	"mreow",
	"mrrrow",
	"mrrr",
	"purr",
};

static const char *meow_punctuations[] = {
	".",
	"?",
	"!",
	"-",
	"~",
	""
};

static const char *meow_suffixes[] = {
	" :3",
	" :3c",
	" ;3",
	" ;3c",
	" x3",
	" x3c",
	" X3",
	" X3c",
	" >:3",
	" >:3c",
	" >;3",
	" >;3c",
	""
};

static void skip_dfu(void)
{
#if DFU_EXISTS // Using Adafruit bootloader
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
	ram_range_retain(dbl_reset_mem, sizeof(dbl_reset_mem), true);
#endif
}

static void print_info(void)
{
	printk(CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT "\n");
	printk(FW_STRING);

	printk("\nBoard: " CONFIG_BOARD "\n");
	printk("SOC: " CONFIG_SOC "\n");
	printk("Target: " CONFIG_BOARD_TARGET "\n");

	printk("\nDevice address: %012llX\n", *(uint64_t *)NRF_FICR->DEVICEADDR & 0xFFFFFFFFFFFF);
}

static void print_uptime(void)
{
	int64_t uptime = k_ticks_to_us_floor64(k_uptime_ticks());

	uint32_t days = uptime / 86400000000;
	uptime %= 86400000000;
	uint8_t hours = uptime / 3600000000;
	uptime %= 3600000000;
	uint8_t minutes = uptime / 60000000;
	uptime %= 60000000;
	uint8_t seconds = uptime / 1000000;
	uptime %= 1000000;
	uint16_t milliseconds = uptime / 1000;
	uint16_t microseconds = uptime %= 1000;

	printk("Uptime: %u.%02u:%02u:%02u.%03u,%03u\n", days, hours, minutes, seconds, milliseconds, microseconds);
}

static void print_meow(void)
{
	int64_t ticks = k_uptime_ticks();

	ticks %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes); // silly number generator
	uint8_t meow = ticks / (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	ticks %= (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	uint8_t punctuation = ticks / ARRAY_SIZE(meow_suffixes);
	uint8_t suffix = ticks % ARRAY_SIZE(meow_suffixes);

	printk("%s%s%s\n", meows[meow], meow_punctuations[punctuation], meow_suffixes[suffix]);
}

static void print_list(void)
{
	printk("Stored devices:\n");
	for (uint8_t i = 0; i < stored_trackers; i++)
		printk("%012llX\n", stored_tracker_addr[i]);
}

static inline void strtolower(char *str) {
	for(int i = 0; str[i]; i++) {
		str[i] = tolower(str[i]);
	}
}

/* RFT: enqueue a mag-cal command for one tracker (or all). Returns count of
 * trackers the command was queued for, or -1 on bad input. */
static int rft_enqueue_for(const char *id_str, const rft_cmd_t *cmd)
{
	if (id_str == NULL) return -1;
	if (strcmp(id_str, "all") == 0) {
		int n = 0;
		for (uint8_t i = 0; i < stored_trackers; i++)
			if (rft_cmd_push(i, cmd)) n++;
		return n;
	}
	int id = (int)parse_i32(id_str, 10);
	if (id < 0 || id >= stored_trackers) return -1;
	return rft_cmd_push((uint8_t)id, cmd) ? 1 : 0;
}

static void print_help(void)
{
	printk("\nhelp                         Display this help text\n");

	printk("\ninfo                         Get device information\n");
	printk("uptime                       Get device uptime\n");
	printk("list                         Get paired devices\n");
	printk("reboot                       Soft reset the device\n");
	printk("\nadd <address>                Manually add a device\n");
	printk("remove                       Remove last device\n");
	printk("pair                         Enter pairing mode\n");
	printk("exit                         Exit pairing mode\n");
	printk("clear                        Clear stored devices\n");
	printk("\nset_mag_bias <id|all> <x> <y> <z>  Set mag bias (Gauss) on tracker\n");
	printk("clear_mag_bias <id|all>      Clear mag bias on tracker (NVS too)\n");
	printk("mag_recal <id|all>           Reset mag cal RAM state (NVS bias kept)\n");
	printk("mag_stream <0|1>             Toggle raw-mag streaming on ALL paired trackers\n");
	printk("rate <id|all> <Hz>           Cap tracker TX rate. \"all\" also sets the TDMA cycle rate (RAM only, 0=default)\n");
	printk("broadcast_index              Resend SET_SLOT_INDEX to every paired tracker (recovery after tracker NVS wipe)\n");
	printk("\nmodem status                 Show modem state and MQTT settings\n");
	printk("modem start | stop           Bring up / tear down LTE link\n");
	printk("modem at <command>           Raw AT passthrough (debug)\n");
	printk("modem apn <name>             Set APN (NVS)\n");
	printk("modem broker <host> <port>   Set MQTT broker (NVS)\n");
	printk("modem client <id>            Set MQTT client id (NVS)\n");
	printk("modem topic <topic>          Set MQTT publish topic (NVS)\n");
	printk("modem user <user>            Set MQTT username (empty clears)\n");
	printk("modem pass <pass>            Set MQTT password (empty clears)\n");
#if DFU_EXISTS
	printk("\ndfu                          Enter DFU bootloader\n");
#endif
	printk("\nmeow                         Meow!\n");
}

static void console_thread(void)
{
	console_getline_init();
	while (log_data_pending())
		k_usleep(1);
	k_msleep(100);
	printk("*** " CONFIG_USB_DEVICE_MANUFACTURER " " CONFIG_USB_DEVICE_PRODUCT " ***\n");
	printk(FW_STRING);
	print_help();

	const char command_help[] = "help";

	const char command_info[] = "info";
	const char command_uptime[] = "uptime";
	const char command_list[] = "list";
	const char command_reboot[] = "reboot";
	const char command_add[] = "add";
	const char command_remove[] = "remove";
	const char command_pair[] = "pair";
	const char command_exit[] = "exit";
	const char command_clear[] = "clear";
#if DFU_EXISTS
	const char command_dfu[] = "dfu";
#endif
	const char command_meow[] = "meow";

	while (1) {
		char *line = console_getline();

		char* argv[6] = {NULL}; // command and 5 args (e.g. set_mag_bias <id> <x> <y> <z>)
		size_t argc = parse_args(line, argv, ARRAY_SIZE(argv));
		if(argc == 0)
			continue;
		if(argc > 0)
			strtolower(argv[0]); // lower case the command
		if(argc > 1)
			strtolower(argv[1]); // lower case the first argument
		// only care that the first words are matchable

		if (strcmp(argv[0], command_help) == 0)
		{
			print_help();
		}
		else if (strcmp(argv[0], command_info) == 0)
		{
			print_info();
		}
		else if (strcmp(line, command_uptime) == 0)
		{
			print_uptime();
		}
		else if (strcmp(line, command_add) == 0)
		{
			if (argc != 2)
			{
				printk("Invalid number of arguments\n");
				continue;
			}
			uint64_t addr = parse_u64(argv[1], 16);
			uint8_t buf[13];
			snprintk(buf, 13, "%012llx", addr);
			if (addr != 0 && strcmp(buf, argv[1]) == 0)
				esb_add_pair(addr, true);
			else
				printk("Invalid address\n");
		}
		else if (strcmp(line, command_remove) == 0)
		{
			esb_pop_pair();
		}
		else if (strcmp(line, command_list) == 0)
		{
			print_list();
		}
		else if (strcmp(line, command_reboot) == 0)
		{
			skip_dfu();
			sys_reboot(SYS_REBOOT_COLD);
		}
		else if (strcmp(line, command_pair) == 0)
		{
			esb_reset_pair();
		}
		else if (strcmp(line, command_exit) == 0)
		{
			esb_finish_pair();
		}
		else if (strcmp(line, command_clear) == 0)
		{
			esb_clear();
		}
#if DFU_EXISTS
		else if (strcmp(line, command_dfu) == 0)
		{
#if ADAFRUIT_BOOTLOADER
			NRF_POWER->GPREGRET = 0x57;
			sys_reboot(SYS_REBOOT_COLD);
#endif
#if NRF5_BOOTLOADER
			gpio_pin_configure(gpio_dev, 19, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
#endif
		}
#endif
		else if (strcmp(line, command_meow) == 0)
		{
			print_meow();
		}
		else if (strcmp(argv[0], "set_mag_bias") == 0)
		{
			if (argc != 5) { printk("Usage: set_mag_bias <id|all> <x> <y> <z>\n"); continue; }
			float x = strtof(argv[2], NULL);
			float y = strtof(argv[3], NULL);
			float z = strtof(argv[4], NULL);
			/* Q11 fixed: 1.0G = 2048, range ±16G */
			int16_t qx = (int16_t)roundf(x * 2048.0f);
			int16_t qy = (int16_t)roundf(y * 2048.0f);
			int16_t qz = (int16_t)roundf(z * 2048.0f);
			rft_cmd_t cmd = { .type = RFT_CMD_SET_MAG_BIAS };
			memcpy(&cmd.data[0], &qx, 2);
			memcpy(&cmd.data[2], &qy, 2);
			memcpy(&cmd.data[4], &qz, 2);
			int n = rft_enqueue_for(argv[1], &cmd);
			if (n < 0) printk("Bad target id\n");
			else printk("Queued set_mag_bias for %d tracker(s)\n", n);
		}
		else if (strcmp(argv[0], "clear_mag_bias") == 0)
		{
			if (argc != 2) { printk("Usage: clear_mag_bias <id|all>\n"); continue; }
			rft_cmd_t cmd = { .type = RFT_CMD_CLEAR_MAG_BIAS };
			int n = rft_enqueue_for(argv[1], &cmd);
			if (n < 0) printk("Bad target id\n");
			else printk("Queued clear_mag_bias for %d tracker(s)\n", n);
		}
		else if (strcmp(argv[0], "mag_recal") == 0)
		{
			if (argc != 2) { printk("Usage: mag_recal <id|all>\n"); continue; }
			rft_cmd_t cmd = { .type = RFT_CMD_MAG_RECAL };
			int n = rft_enqueue_for(argv[1], &cmd);
			if (n < 0) printk("Bad target id\n");
			else printk("Queued mag_recal for %d tracker(s)\n", n);
		}
		else if (strcmp(argv[0], "mag_stream") == 0)
		{
			/* Global flag — every paired tracker streams in lock-step,
			 * no targeting. The flag rides every sync packet so all
			 * trackers see the new state within ~20 ms. */
			if (argc != 2) { printk("Usage: mag_stream <0|1>\n"); continue; }
			int onoff = (int)parse_i32(argv[1], 10);
			if (onoff != 0 && onoff != 1) { printk("Bad on/off (use 0 or 1)\n"); continue; }
			rft_set_global_flag(RFT_FLAG_STREAM_RAW_MAG, onoff != 0);
			printk("mag_stream=%d (broadcast to all)\n", onoff);
		}
		else if (strcmp(argv[0], "rate") == 0)
		{
			/* RFT: cap tracker TX rate. When "all" is the target this is
			 * also the TDMA cycle rate — the receiver itself starts a new
			 * cycle period and broadcasts it via TIMING ACKs. Per-tracker
			 * rate is kept as a hint for the tracker's fusion loop but
			 * the actual TX timing comes from the TDMA schedule. RAM only,
			 * power cycle reverts. */
			if (argc != 3) { printk("Usage: rate <id|all> <Hz 0-255>\n"); continue; }
			int hz = (int)parse_i32(argv[2], 10);
			if (hz < 0 || hz > 255) { printk("Hz out of range (0-255)\n"); continue; }
			rft_cmd_t cmd = { .type = RFT_CMD_SET_MAX_RATE_HZ };
			cmd.data[0] = (uint8_t)hz;
			int n = rft_enqueue_for(argv[1], &cmd);
			if (n < 0) { printk("Bad target id\n"); continue; }
			if (strcmp(argv[1], "all") == 0) {
				tdma_set_rate_hz((uint8_t)hz);
				printk("TDMA cycle now %uHz; queued SET_MAX_RATE_HZ to %d tracker(s)\n",
				       tdma_get_rate_hz(), n);
			} else {
				printk("Queued rate=%dHz for %d tracker(s) (TDMA cycle unchanged at %uHz)\n",
				       hz, n, tdma_get_rate_hz());
			}
		}
		else if (strcmp(argv[0], "broadcast_index") == 0)
		{
			/* RFT TDMA: push SET_SLOT_INDEX to every paired tracker so each
			 * one learns its position in our paired_addr[] table. Use this
			 * when a tracker's NVS was wiped, after firmware update, or
			 * when re-ordering the table — avoids a physical re-pair dance.
			 * Indices are the position in stored_trackers (0..N-1). */
			int n = 0;
			for (uint8_t i = 0; i < stored_trackers && i < TDMA_NUM_SLOTS; i++) {
				rft_cmd_t cmd = { .type = RFT_CMD_SET_SLOT_INDEX };
				cmd.data[0] = i;
				if (rft_cmd_push(i, &cmd)) n++;
			}
			printk("Queued SET_SLOT_INDEX for %d/%d paired tracker(s)\n",
			       n, stored_trackers);
		}
		else if (strcmp(argv[0], "modem") == 0)
		{
			/* RFT: BG770A modem control (wio_bg770a board only — stubs
			 * elsewhere). Subcommands mirror QMTOPEN/QMTCONN params. */
			if (argc < 2) {
				printk("Usage: modem status|start|stop|at|apn|broker|client|topic|user|pass|pace|ptmo ...\n");
				continue;
			}
			strtolower(argv[1]);
			if (strcmp(argv[1], "status") == 0) {
				modem_console_status();
			} else if (strcmp(argv[1], "start") == 0) {
				modem_request_start();
				printk("modem start requested\n");
			} else if (strcmp(argv[1], "stop") == 0) {
				modem_request_stop();
				printk("modem stopped\n");
			} else if (strcmp(argv[1], "at") == 0) {
				if (argc < 3) { printk("Usage: modem at <cmd>\n"); continue; }
				/* Re-join argv[2..] in case the user typed spaces. */
				char buf[160] = {0};
				for (int i = 2; i < (int)argc; i++) {
					strncat(buf, argv[i], sizeof(buf) - strlen(buf) - 1);
					if (i + 1 < (int)argc) strncat(buf, " ", sizeof(buf) - strlen(buf) - 1);
				}
				modem_console_at(buf);
			} else if (strcmp(argv[1], "apn") == 0) {
				modem_console_set_apn(argc >= 3 ? argv[2] : "");
				printk("apn set\n");
			} else if (strcmp(argv[1], "broker") == 0) {
				if (argc < 4) { printk("Usage: modem broker <host> <port>\n"); continue; }
				int port = (int)parse_i32(argv[3], 10);
				modem_console_set_broker(argv[2], (uint16_t)port);
				printk("broker set\n");
			} else if (strcmp(argv[1], "client") == 0) {
				modem_console_set_client(argc >= 3 ? argv[2] : "");
				printk("client set\n");
			} else if (strcmp(argv[1], "topic") == 0) {
				modem_console_set_topic(argc >= 3 ? argv[2] : "");
				printk("topic set\n");
			} else if (strcmp(argv[1], "user") == 0) {
				modem_console_set_user(argc >= 3 ? argv[2] : "");
				printk("user set\n");
			} else if (strcmp(argv[1], "pass") == 0) {
				modem_console_set_pass(argc >= 3 ? argv[2] : "");
				printk("pass set\n");
			} else if (strcmp(argv[1], "pace") == 0) {
				if (argc >= 3) modem_set_pace((int)parse_i32(argv[2], 10));
				printk("pace = %d ms\n", modem_get_pace());
			} else if (strcmp(argv[1], "ptmo") == 0) {
				if (argc >= 3) modem_set_ptmo((int)parse_i32(argv[2], 10));
				printk("ptmo = %d ms\n", modem_get_ptmo());
			} else {
				printk("Unknown modem subcommand\n");
			}
		}
		else
		{
			printk("Unknown command\n");
		}
	}
}

#endif