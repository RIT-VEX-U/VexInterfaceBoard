#include "pico/stdlib.h"
#include "pico.h"
#include <iostream>
#include <cstring>
#include <bit>
#include "otos/OtosDevice.h"

OtosDevice otos;

enum UARTMODE { TX = 1, RX = 0 };

void set_uart_mode(uint gpio, UARTMODE mode) {
    gpio_put(gpio, mode);
}

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
    // printf("decoding\n");

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

// reads until 0x00
static size_t read_cobs_packet(uart_inst_t *uart, uint8_t *buffer, size_t max_length) {
    size_t count = 0;
    while (count < max_length) {
        uint8_t byte = uart_getc(uart);
        if (byte == 0x00) break;
        buffer[count++] = byte;
    }
    return count;
}

int main() {
    // BEGIN INITIAL SETUP
    stdio_usb_init();

    printf("basic readings\n");

    // while (otos.begin() == false) {
    //     printf("otos not connected, check wiring and i2c address\n");
    //     sleep_ms(1);
    // }

    // printf("otos connected\n");


    // otos.set_linear_unit(kInches);
    // otos.set_angular_unit(kDegrees);

    // otos.calibrate_imu();

    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));

    // set rs485 transciever to rx
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    set_uart_mode(2, RX);

    uart_init(uart0, 115200);
    uint8_t data[1] = {0b00010101};

    while (true) {
        // uart_write_blocking(uart0, data, 1);
        sleep_ms(10);
    }
    // while (!uart_is_readable(uart0)) {
    //     printf("asdfaf\n");
    //     sleep_ms(100);
    // }

    uint8_t ctl[1];
    uint8_t ctl_cobs[3];

    uint8_t config[6];
    uint8_t config_cobs[8];

    uint8_t calib_samples[1];
    uint8_t calib_samples_cobs[3];

    uint8_t linear_scalar[4];
    uint8_t linear_scalar_cobs[6];

    uint8_t angular_scalar[4];
    uint8_t angular_scalar_cobs[6];

    uint8_t success[1];
    uint8_t success_cobs[3];
    *success = 0xFF;
    cobsEncode(success, sizeof(success), success_cobs);
    success_cobs[2] = 0x00;

    uint8_t failure[1];
    uint8_t failure_cobs[2];
    *failure = 0xF0;
    cobsEncode(failure, sizeof(failure), failure_cobs);

    uint8_t forty_eight_bit[6];
    uint8_t forty_eight_bit_cobs[8];

    uint8_t ninety_six_bit[12];
    uint8_t ninety_six_bit_cobs[14];

    uint8_t one_forty_four[18];
    uint8_t one_forty_four_cobs[20];

    otos_pose2d_t config_pose;

    // while (true) {
    //     if (uart_is_readable(uart0)) {
    //         uint8_t jawn[1];
    //         uart_read_blocking(uart0, jawn, 1);
    //         printf("%X \n", jawn[0]);
    //     }
    //     sleep_ms(1);
    // }
    

    while (true) {
        // await packet from controller
        // when packet received, determine whether to set config, and what combination of data to send (this is gonna be scary and evil but idc)
        // 0b00000001 is pose (48 bit packet)
        // 0b00000010 is vel (48 bit packet)
        // 0b00000100 is acc (48 bit packet)
        // 0b00000011 is pos/vel (96 bit packet)
        // 0b00000101 is pos/acc (96 bit packet)
        // 0b00000110 is vel/acc (96 bit packet)
        // 0b00000111 is pos/vel/acc (144 bit packet)
        // 0b00001000 is set config mode (wait for another 48 bit packet with the pos to set) respond 0xFF when finished
        // 0b00010000 is calibrate imu, (wait for another 8 bit packet with one byte for 0-255 samples) respond 0xFF when finished
        // 0b00100000 is reset tracking respond 0xFF when finished
        // 0b01000000 is set linear scalar, (wait for another 16 bit packet float) respond 0xFF when finished
        // 0b10000000 is set angular scalar, (wait for another 16 bit packet float) respond 0xFF when finished
        // 0b10001000 is set offset (wait for another 48 bit packet with the offset to set) respond 0xFF when finished
        // anything else is fake as hell respond 0xF0
        // after sending response, switch back to rx mode
        // while (!uart_is_readable(uart0)) {
        //     printf("bad\n");
        //     sleep_ms(500);
        // }
        // printf("good\n");
        
        uart_read_blocking(uart0, ctl_cobs, sizeof(ctl_cobs));
        cobsDecode(ctl_cobs, sizeof(ctl_cobs), ctl);

        // for (int i = 0; i < sizeof(ctl_cobs))

        printf("BYTE!! %X \n", *ctl);
        

        // if it's a config packet
        if (*ctl == 0b00001000) {
            printf("config\n");
            uart_read_blocking(uart0, config_cobs, sizeof(config_cobs));
            cobsDecode(config_cobs, sizeof(config_cobs), config);

            otos.regs_to_pose(config, config_pose, INT16_TO_METER, INT16_TO_RAD);
            otos.set_position(config_pose);

            printf("setting position %f, %f, %f\n", config_pose.x, config_pose.y, config_pose.h);

            set_uart_mode(2, TX);
            sleep_ms(100);
            uart_write_blocking(uart0, success_cobs, sizeof(success_cobs));
            printf("sent: %X %X %X\n", success_cobs[0], success_cobs[1], success_cobs[2]);
        } // if it's a calibrate packet
        else if (*ctl == 0b00010000) {
            printf("calibrate\n");
            uart_read_blocking(uart0, calib_samples_cobs, sizeof(calib_samples_cobs));
            cobsDecode(calib_samples_cobs, sizeof(calib_samples_cobs), calib_samples);

            otos.calibrate_imu(*calib_samples, true);

            set_uart_mode(2, TX);
            sleep_ms(10);
            uart_write_blocking(uart0, success_cobs, sizeof(success_cobs));
        } // if it's a reset packet
        else if (*ctl == 0b00100000) {
            printf("reset\n");
            otos.reset_tracking();

            set_uart_mode(2, TX);
            sleep_ms(10);
            uart_write_blocking(uart0, success_cobs, sizeof(success_cobs));
        } // if it's a set linear scalar packet
        else if (*ctl == 0b01000000) {
            printf("linearscalar\n");
            uart_read_blocking(uart0, linear_scalar_cobs, sizeof(linear_scalar_cobs));
            cobsDecode(linear_scalar_cobs, sizeof(linear_scalar_cobs), linear_scalar);

            float linear_scalarf;
            memcpy(&linear_scalarf, &linear_scalar[0], sizeof(linear_scalarf));

            otos.set_linear_scalar(linear_scalarf);

            set_uart_mode(2, TX);
            sleep_ms(10);
            uart_write_blocking(uart0, success_cobs, sizeof(success_cobs));
        } // if it's a set angular scalar packet
        else if (*ctl == 0b10000000) {
            printf("angularscalar\n");
            uart_read_blocking(uart0, angular_scalar_cobs, sizeof(angular_scalar_cobs));
            cobsDecode(angular_scalar_cobs, sizeof(angular_scalar_cobs), angular_scalar);

            float angular_scalarf;
            memcpy(&angular_scalarf, &angular_scalar[0], sizeof(angular_scalarf));

            otos.set_angular_scalar(angular_scalarf);

            set_uart_mode(2, TX);
            sleep_ms(10);
            uart_write_blocking(uart0, success_cobs, sizeof(success_cobs));
        } // if it's a set offset packet
        else if (*ctl == 0b10001000) {
            printf("offset\n");
            uart_read_blocking(uart0, config_cobs, sizeof(config_cobs));
            cobsDecode(config_cobs, sizeof(config_cobs), config);

            otos.regs_to_pose(config, config_pose, INT16_TO_METER, INT16_TO_RAD);
            otos.set_offset(config_pose);

            printf("setting offset %f, %f, %f\n", config_pose.x, config_pose.y, config_pose.h);

            set_uart_mode(2, TX);
            sleep_ms(10);
            uart_write_blocking(uart0, success_cobs, sizeof(success_cobs));
            printf("sent: %X %X %X\n", success_cobs[0], success_cobs[1], success_cobs[2]);
        } // if it's a data request packet
        else if (*ctl & 0b00000111) {
            printf("data\n");
            if (*ctl == 0b00000001 || *ctl == 0b00000010 || *ctl == 0b00000100) {
                int16_t ints[3];
                // only one of pos vel acc
                if (*ctl == 0b00000001) {
                    printf("pos\n");
                    otos.get_pos_raw(ints[0], ints[1], ints[2]);
                } else if (*ctl == 0b00000010) {
                    printf("vel\n");
                    otos.get_vel_raw(ints[0], ints[1], ints[2]);
                } else if (*ctl == 0b00000100) {
                    printf("acc\n");
                    otos.get_acc_raw(ints[0], ints[1], ints[2]);
                }
                set_uart_mode(2, TX);
                sleep_ms(10);

                memcpy(&forty_eight_bit[0], &ints[0], sizeof(ints[0]));
                memcpy(&forty_eight_bit[2], &ints[1], sizeof(ints[1]));
                memcpy(&forty_eight_bit[4], &ints[2], sizeof(ints[2]));
                cobsEncode(forty_eight_bit, sizeof(forty_eight_bit), forty_eight_bit_cobs);
                forty_eight_bit_cobs[7] = 0x00;

                uart_write_blocking(uart0, forty_eight_bit_cobs, sizeof(forty_eight_bit_cobs));
                printf("sent: %X %X %X %X %X %X %X %X\n", forty_eight_bit_cobs[0], forty_eight_bit_cobs[1], forty_eight_bit_cobs[2], forty_eight_bit_cobs[3], forty_eight_bit_cobs[4], forty_eight_bit_cobs[5], forty_eight_bit_cobs[6], forty_eight_bit_cobs[7]);
            } else if (*ctl == 0b00000011 || *ctl == 0b00000101 || *ctl == 0b00000110) {
                int16_t ints[6];
                // which of the options (this is abysmal truly scary and evil)
                if (*ctl == 0b00000011) {
                    printf("posvel\n");
                    otos.get_pos_raw(ints[0], ints[1], ints[2]);
                    otos.get_vel_raw(ints[3], ints[4], ints[5]);
                } else if (*ctl == 0b00000101) {
                    printf("posacc\n");
                    otos.get_pos_raw(ints[0], ints[1], ints[2]);
                    otos.get_acc_raw(ints[3], ints[4], ints[5]);
                } else if (*ctl == 0b00000110) {
                    printf("velacc\n");
                    otos.get_vel_raw(ints[0], ints[1], ints[2]);
                    otos.get_acc_raw(ints[3], ints[4], ints[5]);
                }
                set_uart_mode(2, TX);
                sleep_ms(10);

                memcpy(&ninety_six_bit[0], &ints[0], sizeof(ints[0]));
                memcpy(&ninety_six_bit[2], &ints[1], sizeof(ints[1]));
                memcpy(&ninety_six_bit[4], &ints[2], sizeof(ints[2]));
                memcpy(&ninety_six_bit[6], &ints[3], sizeof(ints[3]));
                memcpy(&ninety_six_bit[8], &ints[4], sizeof(ints[4]));
                memcpy(&ninety_six_bit[10], &ints[5], sizeof(ints[5]));
                cobsEncode(ninety_six_bit, sizeof(ninety_six_bit), ninety_six_bit_cobs);
                ninety_six_bit_cobs[13] = 0x00;

                uart_write_blocking(uart0, ninety_six_bit_cobs, sizeof(ninety_six_bit_cobs));
                uart_putc(uart0, 0);

            } else if (*ctl == 0b00000111) {
                printf("all\n");
                int16_t ints[9];
                // all of pos vel acc
                otos.get_pos_raw(ints[0], ints[1], ints[2]);
                otos.get_vel_raw(ints[3], ints[4], ints[5]);
                otos.get_acc_raw(ints[6], ints[7], ints[8]);

                set_uart_mode(2, TX);
                sleep_ms(10);

                
                memcpy(&one_forty_four[0], &ints[0], sizeof(ints[0]));
                memcpy(&one_forty_four[2], &ints[1], sizeof(ints[0]));
                memcpy(&one_forty_four[4], &ints[2], sizeof(ints[0]));
                memcpy(&one_forty_four[6], &ints[3], sizeof(ints[0]));
                memcpy(&one_forty_four[8], &ints[4], sizeof(ints[0]));
                memcpy(&one_forty_four[10], &ints[5], sizeof(ints[0]));
                memcpy(&one_forty_four[12], &ints[6], sizeof(ints[0]));
                memcpy(&one_forty_four[14], &ints[7], sizeof(ints[0]));
                memcpy(&one_forty_four[16], &ints[8], sizeof(ints[0]));
                cobsEncode(one_forty_four, sizeof(one_forty_four), one_forty_four_cobs);
                one_forty_four_cobs[19] = 0x00;

                uart_write_blocking(uart0, one_forty_four_cobs, sizeof(one_forty_four_cobs));
                uart_putc(uart0, 0);


            } else {
                printf("ugh\n");
                // if this happens I'm quitting rit vecks you robotics
                set_uart_mode(2, TX);
                sleep_ms(10);
                uart_write_blocking(uart0, failure_cobs, sizeof(failure_cobs));
                uart_putc(uart0, 0);
            }


        } // if it's a fake ass packet
        else {
            printf("idiot\n");
            set_uart_mode(2, TX);
            sleep_ms(10);
            uart_write_blocking(uart0, failure_cobs, sizeof(failure_cobs));
            uart_putc(uart0, 0);
        }
        sleep_ms(5);
        printf("done processing\n");

        set_uart_mode(2, RX);
        
    }
    
    return 0;
}
