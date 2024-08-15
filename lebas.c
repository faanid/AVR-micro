#include <io.h>
#include <delay.h>
#include <alcd.h>
#include <stdint.h>

#define F_CPU 8000000UL

#define MOTOR1_PIN1 PORTD4
#define MOTOR1_PIN2 PORTD5
#define MOTOR2_PIN1 PORTD6
#define MOTOR2_PIN2 PORTD7
#define PWM_PIN1 PORTB3
#define PWM_PIN2 PORTB4



void PWM_init() {
    DDRB |= (1 << PWM_PIN1) | (1 << PWM_PIN2);
    TCCR0 = (1 << COM01) | (1 << WGM00) | (1 << CS00); 
}

void Motor_control(motor,,direction ) {
    if (motor == 1) {
        if (direction == 1) {
            PORTD |= (1 << MOTOR1_PIN1);
            PORTD &= ~(1 << MOTOR1_PIN2);
        } else {
            PORTD |= (1 << MOTOR1_PIN2);
            PORTD &= ~(1 << MOTOR1_PIN1);
        }
    } if (motor == 2) {
        if (direction == 1) {
            PORTD |= (1 << MOTOR2_PIN1);
            PORTD &= ~(1 << MOTOR2_PIN2);
        } else {
            PORTD |= (1 << MOTOR2_PIN2);
            PORTD &= ~(1 << MOTOR2_PIN1);
        }
    }
}

void PWM_set_speed(uint8_t motor uint8_t speed) {
    if (motor == 1) {
        OCR0 = speed;
    } else if (motor == 2) {
       
    }
}
void init_ports() {
    DDRD = 0xFF;  
    DDRB = 0x00;  
    PORTB = 0xFF; 
}

void fill_water() {
    PORTD |= (1 << PORTD0);  
    lcd_clear();        
    lcd_puts("Filling Water...");
    delay_ms(2000);      
    PORTD &= ~(1 << PORTD0); 
}

void wash() {
    PORTD |= (1 << PORTD1); 
    Motor_control(1, 1); 
    PWM_set_speed(1, 128); 
    lcd_clear();         
    lcd_puts("Washing...");
    delay_ms(5000);     
    PORTD &= ~(1 << PORTD1);
    Motor_control(1, 0); 
}

void rinse() {
    PORTD |= (1 << PORTD2);  
    lcd_clear();          
    lcd_puts("Rinsing...");
    delay_ms(3000);     
    PORTD &= ~(1 << PORTD2); 
}

void dry() {
    PORTD |= (1 << PORTD3);  
    Motor_control(2, 1);  
    PWM_set_speed(2, 128); 
    lcd_clear();          
    lcd_puts("Drying...");
    delay_ms(4000);      
    PORTD &= ~(1 << PORTD3);
    Motor_control(2, 0);  
}

int main(void) {
    init_ports();  
    lcd_init(16);  
    lcd_clear();    
    PWM_init();     

    while (1) {
        if (!(PINB & (1 << PB0))) {
            fill_water();
            wash();
            rinse();
            dry();
            lcd_clear();
            lcd_puts("Done!");
            delay_ms(2000);
            lcd_clear();
        }
    }
    return 0; 
}
