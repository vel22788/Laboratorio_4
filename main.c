/*
 * main.c
 *
 * Created: 4/8/2024 6:56:33 AM
 *  Author: Angel Velásquez - 2788
 */ 


#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define BUTTON1_PIN   PC4
#define BUTTON2_PIN   PC5




// Variable global para almacenar el valor del contador
volatile uint8_t counter_d = 0;
volatile uint8_t counter = 0;



// Función para inicializar los pines
void init_pins() {
	// Configurar los pines PC4 y PC5 como entradas y habilitar resistencias de pull-up
	DDRC &= ~(1 << DDC4) & ~(1 << DDC5);
	PORTC |= (1 << PORTC4) | (1 << PORTC5);
	

	// Pines de salida
	DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4) | (1 << DDB5); // PB1-PB5 como salidas
	DDRC |= (1 << DDC0) | (1 << DDC1) | (1 << DDC2)  | (1 << DDC3) ; // PC0-PC3 como salidas
	DDRD |= (1 << DDD0) | (1 << DDD1) | (1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7); // PD0-PD7 como salidas

	// Habilitar interrupciones por cambio de estado en PC4 y PC5
	PCICR |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT12) | (1 << PCINT13);

	// Habilitar interrupciones globales
	sei();
}

//Función para inicializar el ADC
void init_ADC(void){
	ADMUX=0;
	ADCSRA=0;
	ADMUX |= (1<<REFS0);
	//Justificación izquierda
	ADMUX |= (1<<ADLAR);
	//0110 Seleccionar puerto (ADC6 - A6)
	ADMUX |= (1<<MUX1) | (1<<MUX2);
	//Habilitando la interrupción del ADC
	ADCSRA |= (1<<ADIE);
	//Preescaler 128 frecuencia del adc= 16000000/128 = 125kHz
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (ADPS0);
	
	//Habilitando el ADC
	ADCSRA |= (1<<ADEN);
}

// Función para aumentar el contador
void increment_counter() {
	counter++;
	if (counter == 256) {
		// Si el contador llega a su máximo, regresar a 0
		counter = 0;
	}
}

// Función para decrementar el contador
void decrement_counter() {
	if (counter > 0) {
		counter--;
		} else {
		// Si el contador está en cero, poner el valor máximo
		counter = 255;
		}
	}

// Función para mostrar el valor del contador en los LEDs
void display_counter() {
	// Apagar todos los LEDs
	PORTB &= ~((1 << PINB1) | (1 << PINB2) | (1 << PINB3) | (1 << PINB4) | (1 << PINB5));
	PORTC &= ~((1 << PINC0) | (1 << PINC1) | (1 << PINC2));
	
	// Mostrar el valor del contador en los LEDs
	PORTB |= ((counter & 0x1F) << 1); // LED1 a LED5 (00011111)
	PORTC |= ((counter >> 5) & 0x07); // LED6 a LED8 (Desplaza desde el 5 a la  )
}

//Función para comparar el valor del contador del ADC y de las LEDs
void comparar(void){
	if (counter_d>counter){
		PORTC |= (1 << PORTC3);
		}else{
		PORTC &= ~(1 << PORTC3);
	}
}


ISR(ADC_vect){
	
	counter_d=ADCH;
	
	//Apagar bandera Interrupción
	ADCSRA |= (1<<ADIF);
	
}

// Rutina de interrupción
ISR(PCINT1_vect) {
	_delay_ms(15);
	
	// Comprobar si el botón de incremento (PC4) está presionado
	if (!(PINC & (1 << BUTTON1_PIN))) {
		increment_counter();
	}

	// Comprobar si el botón de decremento (PC5) está presionado
	if (!(PINC & (1 << BUTTON2_PIN))) {
		decrement_counter();
	}
	PCIFR |= (1 << PCIF1);
}

// Función principal
int main() {
	// Inicializar los pines
	init_pins();
	init_ADC();
	
	uint8_t tabla[]= {0b00111111, 0b00000110, 0b01011011 , 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111, 0b01110111, 0b01111100, 0b00111001, 0b01011110, 0b01111001, 0b01110001};
	int cont=0;
	uint8_t valor_disp1=0;
	uint8_t valor_disp2=0;

	while (1) {
		
		display_counter();
		valor_disp1= counter_d & 0b00001111;
		valor_disp2= counter_d >> 4;
		
		
		if (cont==0){
			
			PORTD= tabla[valor_disp1];
			PORTD |= (1<<PIND7);
			cont=1;
		}else if (cont==1){
			PORTD= tabla[valor_disp2];
			PORTB |= (1<<PINB0);
			cont=0;
			}
		
		//Encender secuencia del ADC
		ADCSRA |= (1<<ADSC);
		_delay_ms(10);
		comparar();

	}
	
	return 0;
}

