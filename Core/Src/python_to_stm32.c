#include "python_to_stm32.h" // This now includes main.h automatically!
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"


void RingBuffer_Write(RingBuffer *rb, uint8_t byte) {
    uint16_t next = (rb->head + 1) % RING_BUF_SIZE;
    if (next != rb->tail) {  // Avoid overwriting unread data
        rb->buffer[rb->head] = byte;
        rb->head = next;
    } else {
    	printf("Error! Overwriting ringbuffer.");
    }
}

int RingBuffer_Read(RingBuffer *rb, uint8_t *byte) {
    if (rb->head == rb->tail) {
        return 0;  // No data
    }
    *byte = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUF_SIZE;
    return 1;
}

void handshake(char *local_buf) {
	//print all things in buf one by one
    for (int i = 1; local_buf[i] != '\0'; i++) {
        printf("%c", local_buf[i]);
    }
    printf("\n");
}

void pin_high(char pin_number){
	switch(pin_number){
			case '3':
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				break;
			case '4':
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
				break;
			case '5':
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				break;
			case '6':
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
				break;
			case '7':
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				break;
		}
}

void pin_low(int pin_number){
	switch(pin_number){
		case '3':
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
			break;
		case '4':
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			break;
		case '5':
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			break;
		case '6':
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
			break;
		case '7':
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			break;
	}
}

// --- Parse all commands in BIG MEGA switch of DOOM! ---
void parse_commands(RingBuffer *rb, ADS_Stats *ads) {
    // Static variables remember their data between function calls
    static char local_buf[8];
    static uint8_t idx = 0;
    uint8_t c;

    // Drain the ring buffer one byte at a time
    while (RingBuffer_Read(rb, &c)) {
        // 1. When Python sends endcharector '\n'
        if (c == '\n') {
            local_buf[idx] = '\0'; // Cap off the string

            // The first letter tells us what command to run
            char command_letter = local_buf[0];

            // switch of DOOM!
            switch (command_letter) {

                case 'H': // Handshake
                    //printf("Handshake confirmed!\n");
                	handshake(local_buf); // new line
                    break;

                case 'D': // PIN HIGH
                	pin_high(local_buf[1]);
                	printf("pin on\n");
                	break;

                case 'd': // PIN low
                	pin_low(local_buf[1]);
                	printf("pin off\n"); //test
                	break;

                case 'T': // Tempature adc mode
                	initTEMPx_new(0);
                	printf("T-command\r\n");
                	break;

                case 't': // Tempature adc mode with channel selection
                    initTEMPx_new(local_buf[1] - '0'); // Convert char '0', '1', etc. to int 0, 1, etc.
                	printf("t-command\r\n");
                	break;

                case 'R': // read data from adc and give it back
					whileSPI();;
                	break;

                case 'U':
                	unlockSPI();
                	break;

                case 'V':
                	initVOLT_new();
                	printf("V-command\r\n");
                	break;

                case 'c':
                	ads->minchan = (local_buf[1] - '0');
                	printf("setminchan %i\r\n", local_buf[1]);
                	break;

                case 'C':
                	ads->maxchan = (local_buf[1] - '0');
                	printf("setmaxchan %i\r\n", local_buf[1]);
                	break;


                case 'f':
                	switch (local_buf[1]) {
                	case '1':
                		ads->flag1 = 0;
                		printf("no flag1!\r\n");
						break;
                	case '0':
                	default:
                		ads->flag0 = 0;
                		break;
                	}
                	break;

                case 'F':
					switch (local_buf[1]) {
						case '1':
							ads->flag1 = 1;
							printf("set flag1!\r\n");
							break;
						case '0':
						default:
							ads->flag0 = 1;
							break;
                	}
                	break;

                default:
                    printf("Error: Unknown command '%c'\n", command_letter);
                    break;
            }

            // Reset the string builder FOR THE NEXT COMMAND
            idx = 0;
        }
        //If the character is not a newline, save it to the buffer
        else if (c != '\r') {
            // Prevent overflowing the 8-byte buffer
            if (idx < sizeof(local_buf) - 1) {
                local_buf[idx] = (char)c;
                idx++;
            }
        }
    }
}
