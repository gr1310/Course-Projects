#include <stdio.h>
#include "MKL25Z4.h"
#define DHT22_PIN 3
#define UART_BAUD_RATE 57600 //UART baud rate

//to give delay of microseconds
void delay_us(uint32_t us)
{
    // Configure the systick timer for 1us interrupts
    SysTick->LOAD = SystemCoreClock / 1000000 * us;		//setting frequency of clock
    SysTick->VAL = 0;	//clearing the current value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_ENABLE_Msk;				//enable the SysTick timer and use the system clock as the source for the timer

    // Wait for the systick timer to expire
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);		//to check whether clock reached zero or not

    // Disable the systick timer
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	
}

void LED_init(void){
	SIM->SCGC5 |= 0x400; /* enable clock to Port B */
	SIM->SCGC5 |= 0x1000; /* enable clock to Port D */
	PORTB->PCR[18] = 0x100; /* make PTB18 pin as GPIO */
	PTB->PDDR |= 0x40000; /* make PTB18 as output pin */
	PORTB->PCR[19] = 0x100; /* make PTB19 pin as GPIO */
	PTB->PDDR |= 0x80000; /* make PTB19 as output pin */
	PORTD->PCR[1] = 0x100; /* make PTD1 pin as GPIO */
	PTD->PDDR |= 0x02; /* make PTD1 as output pin */
	PTB->PSOR |= 0xC0000;	//turning off LEDs
	PTD->PSOR |= 0x2;
}

//to turn on red LED
void LED_set_red(void){
	LED_init();
	PTB->PSOR |= 0xC0000;
	PTD->PSOR |= 0x2;
	PTB->PCOR |= 0x40000;

}

//to turn on Green LED
void LED_set_green(void){
	LED_init();
	PTB->PSOR |= 0xC0000;
	PTD->PSOR |= 0x2;
	PTB->PCOR |= 0x80000;

}

//to turn on Yellow LED
void LED_set_yellow(void){
	LED_init();
	PTB->PSOR |= 0xC0000;
	PTD->PSOR |= 0x2;
	PTB->PCOR |= 0xC0000;

}


//initialising UART for serial communication
void UART_init(void)
{
    SIM->SCGC4 |= 0x0400 ; /* enable clock for UART0 */
		SIM->SOPT2 |= 0x04000000 ; // selecting clock source MCGFLLCLK clock or MCGPLLCLK/2 clock
		UART0->C2 = 0 ; //Transmitter Receier Disabled
		UART0->BDH = 0x00 ; //Set Baud Rate
		UART0->BDL = 0x18 ; //Set Baud Rate
		UART0->C4 = 0x0F ; // Set oversampling to 16
		UART0->C1 = 0x00 ;
		UART0->C2 = 0x08 ; // Enabling Transmitter, disabling receiver
		SIM->SCGC5 |= 0x0200 ; // Enabling clock to port A 
		PORTA->PCR[ 2 ] = 0x0200 ; //Alternative 2 (chip-specific)
}

//for transmitting data 
void UART_send_byte(uint8_t data)
{
    // Wait for the UART0 transmitter buffer to be empty
    while (!(UART0->S1 & UART_S1_TDRE_MASK));

    // Send the byte to the UART0 transmitter buffer
    UART0->D = data;
}

//for character wise transmission of string
void UART_send_string(char *string)
{
    // Send each character in the string to the UART0 transmitter buffer
    while (*string) {
        UART_send_byte(*string++);
    }
}

//to start DHT22
void DHT22_start_signal() {
    // Set the DHT22 pin as output and pull it low
    GPIOB->PDDR |= (1 << DHT22_PIN);
    GPIOB->PCOR |= (1 << DHT22_PIN);
    
    // Wait for at least 1ms to send start signal
    delay_us(1100);
    
    // Release the DHT22 pin
    GPIOB->PSOR |= (1 << DHT22_PIN);
    
    // Wait for at least 20us for DHT22 to respond
    delay_us(40);
		//LED_set();
}

//to read byte data from DHT22
int DHT22_read_byte() {	
    int byte = 0;
    int mask = 0x80;
    
    // Read each bit in the byte
    for (int i = 0; i < 8; i++) {
        // Wait for the DHT22 to send a signal
        while (!(GPIOB->PDIR & (1 << DHT22_PIN)));	// if pin is low => indicates start signal
        
        // Wait for at least 80us for the bit transmission to be finished
        delay_us(80);
        
        // If the signal is still high after 80us, the bit is a 1
        if (GPIOB->PDIR & (1 << DHT22_PIN)) {
            byte |= mask;
        }
        
        mask >>= 1;
        
        // Wait for the signal to go low
        while (GPIOB->PDIR & (1 << DHT22_PIN));
    }
    
    return byte;
}

int DHT22_read_temperature() {
    // Start signal
    DHT22_start_signal();
    
    // Wait for DHT22 to respond
    while (GPIOB->PDIR & (1 << DHT22_PIN));
    
    // Wait for DHT22 to finish sending data
    while (!(GPIOB->PDIR & (1 << DHT22_PIN)));
    
    // Read the first 2 bytes of data (temperature value)
    int temperature_high = DHT22_read_byte();

    int temperature_low = DHT22_read_byte();
    
    // Combine the two bytes into a 16-bit value
    int temperature = (temperature_high << 8) | temperature_low;
    
    // Convert the temperature value to Celsius
    return (temperature & 0x7FFF) / 1000;
}


int main (void) {
    // Enable clock for PORTB
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    
    // Set the DHT22 pin as input
    GPIOB->PDDR &= ~(1 << DHT22_PIN);
	
    // Enable the pull-up resistor
		PORTB->PCR[DHT22_PIN] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK;
		
    // Read temperature value
    int temperature = DHT22_read_temperature();
	
    // Send the temperature value through UART
    UART_init();
    char temp_string[10];
    sprintf(temp_string, "%d", temperature);
    UART_send_string("Temperature: ");
    UART_send_string(temp_string);
    UART_send_string(" C\n");
		UART_send_string("\r");
		
		//checking whether given temperature is less than or greater than the given temperature
		if (temperature<30){
			LED_set_green();
			UART_send_string("Temperature is below desired temperature\n");
			UART_send_string("\r");
		}
		else if(temperature==30){
			LED_set_yellow();
			UART_send_string("Temperature is desired temperature\n");
			UART_send_string("\r");
		}
		else{
			LED_set_red();
			UART_send_string("Temperature is above desired temperature\n");
			UART_send_string("\r");
		}
	
    
}

