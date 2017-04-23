#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/types.h>

#include "rda_common.h"
#include "rda5991e.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

#define BT_CTRL "/dev/rdacombo"
#define BT_TTY "/dev/ttyS1"

int uart_speed(int s)
{
	switch (s)
	{
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 115200: return B115200;
	case 230400: return B230400;
	case 460800: return B460800;
	case 500000: return B500000;
	case 576000: return B576000;
	case 921600: return B921600;
	case 1000000: return B1000000;
	case 1152000: return B1152000;
	case 1500000: return B1500000;
	case 2000000: return B2000000;
	case 3000000: return B3000000;
	case 4000000: return B4000000;
	default: return B57600;
	}
}

int bt_power_on()
{
	int fd = -1;
	int ret = 0;
	fd = open(BT_CTRL, O_RDWR);
	if (fd < 0)
		return fd;
	ret = ioctl(fd, RDA_BT_POWER_OFF_IOCTL);
	if (ret == -1)
		goto out;
	ret = ioctl(fd, RDA_BT_POWER_ON_IOCTL);
	if (ret == -1)
		goto out;
	ret = ioctl(fd, RDA_BT_EN_CLK);
	if (ret == -1)
		goto out;
	usleep(30000);
	ret = ioctl(fd, RDA_BT_DC_DIG_RESET_IOCTL);
	if (ret == -1)
		goto out;
	usleep(10000);
	ret = ioctl(fd, RDA_BT_RF_INIT_IOCTL);
	if (ret == -1)
		goto out;
	ret = ioctl(fd, RDA_BT_RF_SWITCH_IOCTL);
out:
	if (fd >= 0)
		close(fd);
	return ret;
}

int bt_power_off()
{
	int fd = -1;
	int ret = 0;
	fd = open(BT_CTRL, O_RDWR);
	if (fd < 0)
		return fd;
	ret = ioctl(fd, RDA_BT_POWER_OFF_IOCTL);
	if (fd >= 0)
		close(fd);
	return ret;
}

int bt_get_version(unsigned int *version)
{
	int fd = -1;
	int ret = 0;
	fd = open(BT_CTRL, O_RDWR);
	if (fd < 0)
		return fd;
	ret = ioctl(fd, RDA_WLAN_COMBO_VERSION, version);
	if (fd >= 0)
		close(fd);
	return ret;
}

int bt_rda5991e_setup_flow_ctl(int fd)
{
	unsigned int i, num_send;
	for (i = 0; i < ARRAY_SIZE(rda_flow_ctl); i++)
	{
		num_send = write(fd, rda_flow_ctl[i], sizeof(rda_flow_ctl[i]));
		if (num_send != sizeof(rda_flow_ctl[i]))
		{
			return -1;
		}
		usleep(5000);
	}
	usleep(50000);
	return 0;
}

int bt_setup_uart(int fd, unsigned int version, int baud_rate)
{
	struct termios ti;
	tcflush(fd, TCIOFLUSH);
	if (tcgetattr(fd, &ti) < 0)
	{
		printf("Error getting serial settings");
		return -1;
	}
	cfmakeraw(&ti);
	ti.c_cflag |= CLOCAL;
	ti.c_cflag |= CRTSCTS;
	ti.c_iflag &= ~(IXON | IXOFF | IXANY | 0x80000000);
	if (tcsetattr(fd, TCSANOW, &ti) < 0)
	{
		printf("Error setting serial settings");
		return -1;
	}
	ti.c_cc[VTIME] = 30;
	ti.c_cc[VMIN] = 0;
	cfsetospeed(&ti, uart_speed(baud_rate));
	cfsetispeed(&ti, uart_speed(baud_rate));
	if (tcsetattr(fd, TCSANOW, &ti) < 0)
	{
		printf("Error setting baud rate");
		return -1;
	}
	tcflush(fd, TCIOFLUSH);

	switch (version)
	{
		case WLAN_VERSION_91_E:
			bt_rda5991e_setup_flow_ctl(fd);
			break;

		default:
			printf("Unknown version\n");
			return -1;
	}
	return 0;
}

void bt_write_memory(int fd, const __u32 addr, const __u32 *data, __u8 len, __u8 memory_type)
{
	__u16 num_to_send;
	__u16 i, j;
	__u8 data_to_send[256] = { 0 };
	__u32 address_convert;

	data_to_send[0] = 0x01;
	data_to_send[1] = 0x02;
	data_to_send[2] = 0xfd;
	data_to_send[3] = (__u8)(len * 4 + 6);
	//data_to_send[4] = (memory_type+0x80);  // add the event display(0x80); no event(0x00)
	data_to_send[4] = (memory_type + 0x00);  // add the event display(0x80); no event(0x00)
	data_to_send[5] = len;
	if (memory_type == 0x01)
	{
		address_convert = addr * 4 + 0x200;

		data_to_send[6] = (__u8)address_convert;
		data_to_send[7] = (__u8)(address_convert >> 8);
		data_to_send[8] = (__u8)(address_convert >> 16);
		data_to_send[9] = (__u8)(address_convert >> 24);
	}
	else
	{
		data_to_send[6] = (__u8)addr;
		data_to_send[7] = (__u8)(addr >> 8);
		data_to_send[8] = (__u8)(addr >> 16);
		data_to_send[9] = (__u8)(addr >> 24);
	}
	for (i=0; i < len; i++, data++)
	{
		j=10+i*4;
		data_to_send[j] = (__u8)(*data);
		data_to_send[j + 1] = (__u8)((*data) >> 8);
		data_to_send[j + 2] = (__u8)((*data) >> 16);
		data_to_send[j + 3] = (__u8)((*data) >> 24);
	}
	num_to_send = 4 + data_to_send[3];
	write(fd, &(data_to_send[0]), num_to_send);
}

void bt_uart_write_array(int fd, const __u32 buf[][2], __u16 len, __u8 type)
{
	__u32 i;
	for (i=0; i < len; i++)
	{
		bt_write_memory(fd, buf[i][0], &buf[i][1], 1, type);
		usleep(12000); //12ms?
	}
}

void bt_write_address(int s_fd, int d_fd)
{
	__u8 bt_addr[10] = { 0x01, 0x1a, 0xfc, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	__u8 bt_addr_lt[6] = { 0x00 };
	int i;

	ioctl(d_fd, RDA_BT_GET_ADDRESS_IOCTL, bt_addr_lt);
	for (i = 0; i < 6; i++)
	{
		bt_addr[4 + i] = bt_addr_lt[5 - i];
	}
	write(s_fd, &(bt_addr[0]), sizeof(bt_addr));
	usleep(10000);
}

void bt_write_pskey(int fd,unsigned char id, const unsigned char *data,unsigned char len)
{
	unsigned short num_to_send;
	unsigned char data_to_send[20];
	data_to_send[0] = 0x01;
	data_to_send[1] = 0x05;
	data_to_send[2] = 0xfd;
	data_to_send[3] = len + 1;
	data_to_send[4] = id;
	num_to_send = 5;
	write(fd, data_to_send, num_to_send);
	usleep(1000);
	num_to_send = len;
	write(fd, data, num_to_send);
	usleep(3000);
}

void bt_patch_write(int fd)
{
	bt_uart_write_array(fd, RDA5991e_PSKEY_MISC, ARRAY_SIZE(RDA5991e_PSKEY_MISC), 0);
	bt_write_pskey(fd, 0x35, rdabt_pskey_hostwake, sizeof(rdabt_pskey_hostwake));
	usleep(500);
	bt_write_pskey(fd, 0x21, rdabt_pskey_sleep, sizeof(rdabt_pskey_sleep));
	usleep(500);
	bt_write_pskey(fd, 0x15, rdabt_pskey_sys_config, sizeof(rdabt_pskey_sys_config));
	usleep(500);
	bt_write_pskey(fd,0x24, rdabt_pskey_rf_setting, sizeof(rdabt_pskey_rf_setting));
	usleep(5000);
	bt_uart_write_array(fd, RDA5991e_PATCH, ARRAY_SIZE(RDA5991e_PATCH), 0);
	bt_uart_write_array(fd, RDA5991e_NO_TXRX_PATCH, ARRAY_SIZE(RDA5991e_NO_TXRX_PATCH), 0);
	usleep(5000);
}

int bt_custom_baudrate_map(int baud_rate)
{
	switch (baud_rate)
	{
		case 1500000: return 1625000;
		case 3000000: return 3250000;
		default: return baud_rate;
	}
}

int bt_rda_change_baudrate(int fd, unsigned int version, int baud_rate)
{
	int baud = bt_custom_baudrate_map(baud_rate);
	__u8 uart_setting[13] = { 0 };
	memcpy(&uart_setting, uart_setting_change_baud_rate, ARRAY_SIZE(uart_setting_change_baud_rate));
	switch (version)
	{
		case WLAN_VERSION_91_E:
			uart_setting[5] = baud & 0xff;
			uart_setting[6] = (baud >> 8) & 0xff;
			uart_setting[7] = (baud >> 16) & 0xff;
			uart_setting[8] = (baud >> 24) & 0xff;
			bt_write_pskey(fd, 0x31, uart_setting_change_baud_rate, sizeof(uart_setting_change_baud_rate));
			break;

		default:
			printf("Not supported");
			return -1;
	}
	return 0;
}

int bt_init(int fd, unsigned int version, int baud_rate)
{
	int ret = 0;
	int ctl_fd = open(BT_CTRL, O_RDWR);
	switch (version)
	{
		case WLAN_VERSION_91_E:
			bt_uart_write_array(fd, RDA5991e_PSK_rf, sizeof(RDA5991e_PSK_rf) / sizeof(RDA5991e_PSK_rf[0]), 0);
			usleep(5000);
			ioctl(ctl_fd, RDA_BT_DC_CAL_IOCTL_FIX_5991_LNA_GAIN);
			bt_write_address(fd, ctl_fd);
			bt_patch_write(fd);
			break;
	}

	ret = bt_rda_change_baudrate(fd, version, baud_rate);

	if (ctl_fd > 0)
	{
		close(ctl_fd);
	}
	return ret;
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd = -1;

	ret = bt_power_on();
	if (ret)
		return -1;

	sleep(5);
	unsigned int version = 0;
	ret = bt_get_version(&version);
	if (ret)
		return -1;

	printf("Found version %d\n", version);

	fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY);
	if (fd < 0)
	{
		printf("Error opening serial port");
		return -1;
	}

	ret = bt_setup_uart(fd, version, 115200);
	if (ret)
		goto out;

	ret = bt_init(fd, version, 921600);
	if (ret)
		goto out;

	ret = bt_setup_uart(fd, version, 921600);
	if (ret)
		goto out;

	close(fd);
	return 0;

out:
	close(fd);
	bt_power_off();
	return ret;
}
