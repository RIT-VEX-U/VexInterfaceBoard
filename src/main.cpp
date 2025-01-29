#include "pico/stdlib.h"
#include "pico.h"
#include <iostream>
#include <cstring>
#include "otos/OtosDevice.h"

OtosDevice otos;

/** COBS encode data to buffer
	@param data Pointer to input data to encode
	@param length Number of bytes to encode
	@param buffer Pointer to encoded output buffer
	@return Encoded buffer length in bytes
	@note Does not output delimiter byte
*/
size_t cobsEncode(const void *data, size_t length, uint8_t *buffer)
{
	assert(data && buffer);

	uint8_t *encode = buffer; // Encoded byte pointer
	uint8_t *codep = encode++; // Output code pointer
	uint8_t code = 1; // Code value

	for (const uint8_t *byte = (const uint8_t *)data; length--; ++byte)
	{
		if (*byte) // Byte not zero, write it
			*encode++ = *byte, ++code;

		if (!*byte || code == 0xff) // Input is zero or block completed, restart
		{
			*codep = code, code = 1, codep = encode;
			if (!*byte || length)
				++encode;
		}
	}
	*codep = code; // Write final code value

	return (size_t)(encode - buffer);
}

/** COBS decode data from buffer
	@param buffer Pointer to encoded input bytes
	@param length Number of bytes to decode
	@param data Pointer to decoded output data
	@return Number of bytes successfully decoded
	@note Stops decoding if delimiter byte is found
*/
size_t cobsDecode(const uint8_t *buffer, size_t length, void *data)
{
	assert(buffer && data);

	const uint8_t *byte = buffer; // Encoded input byte pointer
	uint8_t *decode = (uint8_t *)data; // Decoded output byte pointer

	for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block)
	{
		if (block) // Decode block byte
			*decode++ = *byte++;
		else
		{
			block = *byte++;             // Fetch the next block length
			if (block && (code != 0xff)) // Encoded zero, write it unless it's delimiter.
				*decode++ = 0;
			code = block;
			if (!code) // Delimiter code found
				break;
		}
	}

	return (size_t)(decode - (uint8_t *)data);
}

int main() {
    stdio_usb_init();
    sleep_ms(5000);

    printf("basic readings\n");

    while (otos.begin() == false) {
        printf("otos not connected, check wiring and i2c address\n");
        sleep_ms(1000);
    }

    printf("otos connected\n");

    sleep_ms(2000);

    otos.set_linear_unit(kInches);
    otos.set_linear_scalar(1.094391244870041);
    otos.set_angular_unit(kDegrees);
    otos.set_angular_scalar(0.9979735592449776);
    otos.reset_tracking();

    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));

    // set rs485 transciever to tx
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    gpio_put(2, 0);

    

    
    

    uart_init(uart0, 115200);
    sleep_ms(1000);

    uint8_t initial_config_cobs[26];
    printf("waiting...\n");
    while (!uart_is_readable(uart0)) {
        sleep_ms(5);
    }
    uart_read_blocking(uart0, initial_config_cobs, sizeof(initial_config_cobs));
    printf("received\n");
    uint8_t initial_config[25];
    cobsDecode(initial_config_cobs, sizeof(initial_config_cobs), initial_config);

    gpio_put(2, 1);

    float initial_x;
    float initial_y;
    float initial_rot;

    float offset_x;
    float offset_y;
    float offset_rot;

    bool calc_vel_acc_on_brain;

    memcpy(&initial_x, &initial_config[0], sizeof(float));
    memcpy(&initial_y, &initial_config[4], sizeof(float));
    memcpy(&initial_rot, &initial_config[8], sizeof(float));
    memcpy(&offset_x, &initial_config[12], sizeof(float));
    memcpy(&offset_y, &initial_config[16], sizeof(float));
    memcpy(&offset_rot, &initial_config[20], sizeof(float));
    memcpy(&calc_vel_acc_on_brain, &initial_config[24], sizeof(bool));
    printf("initial %f %f %f\n", initial_x, initial_y, initial_rot);
    printf("offset  %f %f %f\n", offset_x, offset_y, offset_rot);

    
    otos_pose2d_t offset_pose{offset_x, offset_y, offset_rot};
    otos.set_offset(offset_pose);
    
    otos.reset_tracking();
    otos_pose2d_t initial_pose{initial_x, initial_y, initial_rot};
    otos.set_position(initial_pose);
    otos.calibrate_imu(255, true);


    printf("calibrated\n");


    while (true) {
        otos_pose2d_t pose;
        otos_pose2d_t vel;
        otos_pose2d_t acc;
        otos.get_pos_vel_acc(pose, vel, acc);
        float velf = sqrt(vel.x * vel.x + vel.y * vel.y);
        float avelf = vel.h;
        float accf = sqrt(acc.x * acc.x + acc.y * acc.y);
        float aaccf = acc.h;

        uint8_t raw[sizeof(pose) + 16];
        uint8_t encoded[sizeof(pose) + 16 + 1];

        memcpy(&raw[0], &pose, sizeof(pose));
        memcpy(&raw[12], &velf, sizeof(velf));
        memcpy(&raw[16], &avelf, sizeof(avelf));
        memcpy(&raw[20], &accf, sizeof(accf));
        memcpy(&raw[24], &aaccf, sizeof(aaccf));

        
        cobsEncode(raw, sizeof(raw), encoded);

        uart_write_blocking(uart0, encoded, sizeof(encoded));
        uart_putc(uart0, 0);


        // printf("%d\n", sizeof(raw));
        // printf("raw     %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7], raw[8], raw[9], raw[10], raw[11]);
        // printf("encoded %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", encoded[0], encoded[1], encoded[2], encoded[3], encoded[4], encoded[5], encoded[6], encoded[7], encoded[8], encoded[9], encoded[10], encoded[11], encoded[12]);

        // uart_write_blocking(uart1, raw, sizeof(pose));

        // uart_putc(uart0, 0xFF);
        // printf("sent ff\n");

        // sleep_ms(500);

        // uart_putc(uart0, 0x00);
        // printf("sent 00\n");




        sleep_ms(10);
    }
    
    return 0;
}
