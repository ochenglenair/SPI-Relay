/**
 * \file
 *
 * \brief Application to control an Infineon ISO1H816G evaluation board via SPI.
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

// --- Pin Definitions for Infineon Board Control ---
// **IMPORTANT**: These pin assignments must match your MCC configuration.
//
// SPI Peripheral Pins (handled by MCC SERCOM configuration):
// MOSI: PA04 (Maps to K3 Pin 6 - SI)
// MISO: PA06 (Maps to K3 Pin 13 - SO)
// SCK:  PA05 (Maps to K3 Pin 4 - SCLK)
//
// GPIO Control Pins (handled by this code):
#define CS_PIN   PA07 // Chip Select (Maps to K3 Pin 3 - /CS) <-- UPDATED PIN
#define DIS_PIN  PA02 // Disable Pin (Maps to K3 Pin 2 - /DIS)

// SPI peripheral instance from atmel_start
extern struct spi_m_sync_descriptor SPI_0;

/**
 * @brief Sends commands to both daisy-chained ISO1H816G chips.
 * @param command_k2 The 8-bit command for the chip controlling the K2 connector.
 * @param command_k1 The 8-bit command for the chip controlling the K1 connector.
 */
void send_iso_command(uint8_t command_k2, uint8_t command_k1)
{
    // Create a 2-byte buffer for the SPI transfer.
    // Due to the daisy-chain, the command for the last chip (K2) must be sent first.
    uint8_t commands[2] = {command_k2, command_k1};

    struct spi_xfer xfer;
    xfer.txbuf = commands;
    xfer.rxbuf = NULL;
    xfer.size  = 2; // Transfer two bytes

    // 1. Assert Chip Select (pull it LOW to select the chips)
    gpio_set_pin_level(CS_PIN, false);
    delay_us(1); // Small delay after CS goes low

    // 2. Perform the 16-bit (2-byte) SPI transfer
    spi_m_sync_transfer(&SPI_0, &xfer);

    // 3. De-assert Chip Select (pull it HIGH to de-select the chips)
    // This latches the data into both chips' output registers simultaneously.
    delay_us(1); // Small delay before CS goes high
    gpio_set_pin_level(CS_PIN, true);
}


int main(void)
{
	uint8_t k1_output_state = 0x00; // State for K1 outputs
	uint8_t k2_output_state = 0x00; // State for K2 outputs
	bool toggle = false;

	atmel_start_init();

	// --- Initialize Control Pins for Infineon Board ---
	gpio_set_pin_direction(CS_PIN, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(CS_PIN, true); // Chip select is active low, so initialize it to HIGH (inactive)

	gpio_set_pin_direction(DIS_PIN, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(DIS_PIN, true); // Disable pin is active low, initialize to HIGH (outputs enabled)

	// --- Initialize SPI peripheral ---
	spi_m_sync_enable(&SPI_0);

	// Configure the switch pin with an internal pull-up resistor.
	gpio_set_pin_pull_mode(SW0, GPIO_PULL_UP);

	// Initial state: Turn all channels off on both chips and turn the onboard LED off.
	// Note: LED0 on the Xplained Pro is active-low, so true means OFF.
	gpio_set_pin_level(LED0, true);
	send_iso_command(0x00, 0x00);

	while (true) {
		// Wait for the button to be pressed (pin goes LOW)
		while (gpio_get_pin_level(SW0)) {
			// Do nothing, just wait for the press.
		}

		// Toggle the state for the output channels
		toggle = !toggle;
		// Set the same state for both K1 and K2 outputs
		k1_output_state = toggle ? 0xFF : 0x00; // 0xFF = all ON, 0x00 = all OFF
		k2_output_state = toggle ? 0xFF : 0x00;

		// Toggle the onboard LED for visual feedback at the same time.
		gpio_toggle_pin_level(LED0);

		// Send the new commands to the Infineon board
		send_iso_command(k2_output_state, k1_output_state);

		// Debounce delay for press
		delay_ms(50);

		// Wait for the button to be released (pin goes back to HIGH)
		while (!gpio_get_pin_level(SW0)) {
			// Do nothing, just wait for the release.
		}

		// Debounce delay for release
		delay_ms(50);
	}
}
