/**
 * \file
 *
 * \brief Application to control an Infineon ISO1H816G evaluation board via SPI
 * using commands from a serial terminal.
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "atmel_start.h"
#include "atmel_start_pins.h"
#include <hal_gpio.h>
#include <hal_delay.h>
#include <hal_spi_m_sync.h>
#include <hal_usart_sync.h> // Include for serial communication
#include <string.h>          // Include for string functions
#include <stdio.h>           // Include for sprintf

// --- Pin Definitions for Infineon Board Control ---
#define CS_PIN   PA07
#define DIS_PIN  PA02

// Peripheral instances from atmel_start
extern struct spi_m_sync_descriptor SPI_0;
extern struct usart_sync_descriptor USART_0; // USART for serial terminal

/**
 * @brief Sends commands to both daisy-chained ISO1H816G chips.
 * @param command_k2 The 8-bit command for the chip controlling the K2 connector.
 * @param command_k1 The 8-bit command for the chip controlling the K1 connector.
 */
void send_iso_command(uint8_t command_k2, uint8_t command_k1)
{
    uint8_t commands[2] = {command_k2, command_k1};
    struct spi_xfer xfer;
    xfer.txbuf = commands;
    xfer.rxbuf = NULL;
    xfer.size  = 2;

    gpio_set_pin_level(CS_PIN, false);
    delay_us(1);
    spi_m_sync_transfer(&SPI_0, &xfer);
    delay_us(1);
    gpio_set_pin_level(CS_PIN, true);
}

/**
 * @brief Parses an 8-character string of '0's and '1's into a byte.
 * @param buffer The character buffer containing the binary string.
 * @return The parsed 8-bit integer value. Returns 0 on error.
 */
uint8_t parse_binary_string(const char *buffer)
{
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        if (buffer[i] == '1') {
            result |= (1 << (7 - i));
        } else if (buffer[i] != '0') {
            return 0; // Invalid character
        }
    }
    return result;
}

/**
 * @brief Reads a line of exactly n characters from the serial terminal.
 * @param buffer A pointer to the character buffer to store the input.
 * @param len The number of characters to read.
 */
void read_n_bytes(char* buffer, int len) {
    int count = 0;
    while (count < len) {
        // Read one character at a time
        if (io_read(&USART_0.io, (uint8_t*)&buffer[count], 1) == 1) {
            count++;
        }
    }
}

int main(void)
{
	char k1_buffer[9] = {0}; // 8 chars + null terminator
	char k2_buffer[9] = {0};
	uint8_t k1_state = 0x00;
	uint8_t k2_state = 0x00;

	atmel_start_init();

	// --- Initialize Control Pins for Infineon Board ---
	gpio_set_pin_direction(CS_PIN, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(CS_PIN, true);
	gpio_set_pin_direction(DIS_PIN, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(DIS_PIN, true);

	// --- Initialize Peripherals ---
	spi_m_sync_enable(&SPI_0);
	usart_sync_enable(&USART_0);

	// Initial state: Turn all channels off
	send_iso_command(0x00, 0x00);
	
	// Send a ready message to the Python script
	io_write(&USART_0.io, (uint8_t*)"SAMD21 Ready\r\n", 15);

	while (true) {
		// Wait for and read the 8 bytes for K1
		read_n_bytes(k1_buffer, 8);
		k1_state = parse_binary_string(k1_buffer);

		// Wait for and read the 8 bytes for K2
		read_n_bytes(k2_buffer, 8);
		k2_state = parse_binary_string(k2_buffer);

		// Send the new commands to the Infineon board
		send_iso_command(k2_state, k1_state);

		// Send a confirmation message back to the Python script
		char confirmation_msg[50];
		sprintf(confirmation_msg, "OK: K1=0x%02X, K2=0x%02X\r\n", k1_state, k2_state);
		io_write(&USART_0.io, (uint8_t*)confirmation_msg, strlen(confirmation_msg));
		
		// Toggle the onboard LED to show a command was processed
		gpio_toggle_pin_level(LED0);
	}
}
