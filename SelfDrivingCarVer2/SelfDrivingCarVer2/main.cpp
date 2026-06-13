#define F_CPU 8000000UL
#define BAUD 9600
#define MYUBRR (((F_CPU / (BAUD * 16UL))) - 1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

// ==========================================
// C?U HÌNH CHÂN ??NG C? L298N
// ==========================================
#define MOT_L_DDR      DDRB
#define MOT_L_PORT     PORTB
#define MOT_L_IN1      PB1
#define MOT_L_IN2      PB2
#define MOT_L_PWM_PIN  PB3   // OC0 (Timer0)

#define MOT_R_IN_DDR   DDRC
#define MOT_R_IN_PORT  PORTC
#define MOT_R_IN3      PC0
#define MOT_R_IN4      PC1

#define MOT_R_PWM_DDR  DDRD
#define MOT_R_PWM_PORT PORTD
#define MOT_R_PWM_PIN  PD7   // OC2 (Timer2)

#define BASE_SPEED     100
#define TURN_SPEED     120
#define SLOW_TURN      50

// ==========================================
// C?U HÌNH CHÂN C?M BI?N & SERVO
// ==========================================
#define US_TRIG     PD2
#define US_ECHO     PD3      // INT1
#define SERVO_PIN   PD4      // OC1B

#define SERVO_TOP   19999
#define SERVO_MIN   500
#define SERVO_MAX   2500
#define SERVO_MID   1500

// ==========================================
// THÔNG S? ??NG C?
// ==========================================
#define KP                1.6  // H? s? K khi t?c ?? > 70
#define KP_LOW_SPEED      1  // H? s? K m?nh h?n khi t?c ?? <= 70 (B?n có th? tinh ch?nh s? này)
#define LEFT_MOTOR_OFFSET 5
#define COMPENSATE_K      10

// ==========================================
// THÔNG S? SIÊU ÂM (?Ã ???C T?I GI?N)
// ==========================================
#define ECHO_TIMEOUT_MS     30   // Timeout n?u không có h?i ?áp
#define DIST_MIN_CM         2    // D??i 2cm là nhi?u ch?m nón
#define DIST_MAX_CM         200  // Sa bàn th?c t? ch? c?n nhìn t?i ?a 2m

#define OBSTACLE_CONFIRM    5    // Ch? c?n 2 l?n nhìn th?y liên ti?p (100ms) là lách ngay
#define OBSTACLE_DIST_CM    30   // Kho?ng cách kích ho?t lách v?t c?n

// ==========================================
// BI?N TOÀN C?C H? TH?NG
// ==========================================
volatile char     rx_buffer[30];
volatile uint8_t  rx_index  = 0;
volatile uint8_t  data_ready = 0;

volatile uint32_t timer0_millis = 0;

volatile uint8_t auto_mode = 0; // 1: AUTO, 0: MANUAL
volatile uint8_t avoid_enable = 0; // [M?I] 1: Cho phep lach vat can, 0: Tat lach

// --- Bi?n ng?t siêu âm ---
volatile uint16_t echo_start    = 0;
volatile uint16_t echo_duration = 0;
volatile uint8_t  echo_ready    = 0;
volatile uint8_t  is_waiting_echo = 0;
volatile uint32_t ping_sent_time  = 0;

// --- Qu?n lý kho?ng cách ---
uint8_t  obstacle_count = 0;     // ??m khung hình liên ti?p
uint16_t distance_cm = 999;      // Kho?ng cách hi?n t?i

// --- ?i?u khi?n lách ---
uint8_t  is_avoiding = 0;
uint32_t avoid_start_time = 0;
int      steer_snapshot = 0;
int      current_steer  = 0;

// --- Servo smooth ---
uint16_t servo_target  = SERVO_MID;
uint16_t servo_current = SERVO_MID;
uint32_t last_servo_update = 0;

// ==========================================
// 1. HÀM TH?I GIAN (millis)
// ==========================================
ISR(TIMER0_OVF_vect) {
	timer0_millis += 2;
}

uint32_t millis() {
	uint32_t m;
	cli();
	m = timer0_millis;
	sei();
	return m;
}

// ==========================================
// 2. SERVO SMOOTH
// ==========================================
void set_servo_target(uint16_t target) {
	servo_target = target;
}

void update_servo_smoothly(uint32_t current_time) {
	if (current_time - last_servo_update >= 5) {
		if (servo_current < servo_target) {
			servo_current += 10;
			if (servo_current > servo_target) servo_current = servo_target;
			} else if (servo_current > servo_target) {
			servo_current -= 10;
			if (servo_current < servo_target) servo_current = servo_target;
		}
		OCR1B = servo_current;
		last_servo_update = current_time;
	}
}

// ==========================================
// 3. NG?T SIÊU ÂM VÀ UART
// ==========================================
ISR(INT1_vect) {
	if (PIND & (1 << US_ECHO)) {
		if (is_waiting_echo) {
			echo_start = TCNT1;
		}
		} else {
		if (is_waiting_echo) {
			uint16_t echo_end = TCNT1;
			if (echo_end >= echo_start) {
				echo_duration = echo_end - echo_start;
				} else {
				echo_duration = (SERVO_TOP - echo_start) + echo_end + 1;
			}
			echo_ready      = 1;
			is_waiting_echo = 0;
		}
	}
}

ISR(USART_RXC_vect) {
	char data = UDR;
	if (data == '\n') {
		rx_buffer[rx_index] = '\0';
		data_ready = 1;
		rx_index   = 0;
		} else {
		if (rx_index < 29) rx_buffer[rx_index++] = data;
	}
}

// ==========================================
// 4. B? L?C C? B?N VÀ XÁC NH?N V?T C?N (T?C ?? CAO)
// ==========================================
uint16_t filter_distance(uint16_t raw_cm) {
	// Ch? lo?i b? các nhi?u vô lý, còn l?i tin t??ng tuy?t ??i ?? ??m b?o Real-time
	if (raw_cm < DIST_MIN_CM || raw_cm > DIST_MAX_CM) {
		return 999;
	}
	return raw_cm;
}

uint8_t check_obstacle_confirmed(uint16_t dist) {
	if (dist > 0 && dist <= OBSTACLE_DIST_CM) {
		if (obstacle_count < OBSTACLE_CONFIRM) obstacle_count++;
		} else {
		obstacle_count = 0; // Xóa b? ??m ngay l?p t?c n?u kho?ng tr?ng xu?t hi?n
	}
	return (obstacle_count >= OBSTACLE_CONFIRM);
}

// ==========================================
// 5. HÀM TRUY?N ??NG C? C?T LÕI (VI SAI)
// ==========================================
void Motor_Drive(int target_speed, int steering_angle) {
	if (target_speed == 0) {
		MOT_L_PORT    &= ~((1<<MOT_L_IN1) | (1<<MOT_L_IN2));
		MOT_R_IN_PORT &= ~((1<<MOT_R_IN3) | (1<<MOT_R_IN4));
		OCR0 = 0; OCR2 = 0;
		return;
	}

	// --- THU?T TOÁN K THÍCH ?NG THEO T?C ?? ---
	float active_kp = KP; // M?c ??nh dùng KP cho t?c ?? cao
	if (abs(target_speed) <= 70) {
		active_kp = KP_LOW_SPEED; // ??i sang h? s? K khác khi ?i ch?m
	}

	int base_left  = target_speed + LEFT_MOTOR_OFFSET;
	int base_right = target_speed;

	// Áp d?ng h? s? active_kp v?a ???c quy?t ??nh ? trên
	int left_speed  = base_left  + (int)(steering_angle * active_kp);
	int right_speed = base_right - (int)(steering_angle * active_kp);

	if (left_speed >= 0) {
		MOT_L_PORT |=  (1<<MOT_L_IN1);
		MOT_L_PORT &= ~(1<<MOT_L_IN2);
		if (left_speed > 255) left_speed = 255;
		} else {
		MOT_L_PORT &= ~(1<<MOT_L_IN1);
		MOT_L_PORT |=  (1<<MOT_L_IN2);
		left_speed = -left_speed;
		if (left_speed > 255) left_speed = 255;
	}
	OCR0 = (uint8_t)left_speed;

	if (right_speed >= 0) {
		MOT_R_IN_PORT |=  (1<<MOT_R_IN3);
		MOT_R_IN_PORT &= ~(1<<MOT_R_IN4);
		if (right_speed > 255) right_speed = 255;
		} else {
		MOT_R_IN_PORT &= ~(1<<MOT_R_IN3);
		MOT_R_IN_PORT |=  (1<<MOT_R_IN4);
		right_speed = -right_speed;
		if (right_speed > 255) right_speed = 255;
	}
	OCR2 = (uint8_t)right_speed;
}

// ==========================================
// HÀM ?I?U KHI?N ??NG C? TRÁI (MOTOR A)
// ==========================================
void set_motor_a(int speed) {
	// Gi?i h?n t?c ?? trong kho?ng -255 ??n 255
	if (speed > 255) speed = 255;
	if (speed < -255) speed = -255;
	
	// L?y giá tr? tuy?t ??i ?? b?m xung PWM
	uint8_t abs_speed = (uint8_t)(speed > 0 ? speed : -speed);
	OCR0 = abs_speed;
	
	// ?i?u khi?n chi?u quay
	if (speed > 0) {
		// Ch?y t?i
		MOT_L_PORT |=  (1 << MOT_L_IN1);
		MOT_L_PORT &= ~(1 << MOT_L_IN2);
		} else if (speed < 0) {
		// Ch?y lùi
		MOT_L_PORT &= ~(1 << MOT_L_IN1);
		MOT_L_PORT |=  (1 << MOT_L_IN2);
		} else {
		// D?ng t? do (Coast)
		MOT_L_PORT &= ~((1 << MOT_L_IN1) | (1 << MOT_L_IN2));
	}
}

// ==========================================
// HÀM ?I?U KHI?N ??NG C? PH?I (MOTOR B)
// ==========================================
void set_motor_b(int speed) {
	// Gi?i h?n t?c ?? trong kho?ng -255 ??n 255
	if (speed > 255) speed = 255;
	if (speed < -255) speed = -255;
	
	// L?y giá tr? tuy?t ??i ?? b?m xung PWM
	uint8_t abs_speed = (uint8_t)(speed > 0 ? speed : -speed);
	OCR2 = abs_speed;
	
	// ?i?u khi?n chi?u quay
	if (speed > 0) {
		// Ch?y t?i
		MOT_R_IN_PORT |=  (1 << MOT_R_IN3);
		MOT_R_IN_PORT &= ~(1 << MOT_R_IN4);
		} else if (speed < 0) {
		// Ch?y lùi
		MOT_R_IN_PORT &= ~(1 << MOT_R_IN3);
		MOT_R_IN_PORT |=  (1 << MOT_R_IN4);
		} else {
		// D?ng t? do (Coast)
		MOT_R_IN_PORT &= ~((1 << MOT_R_IN3) | (1 << MOT_R_IN4));
	}
}

// ==========================================
// 6. CÁC HÀM ?I?U H??NG C? TH?
// ==========================================
void move_stop() {
	set_motor_a(0);
	set_motor_b(0);
}

void forward() {
	set_motor_a(BASE_SPEED);
	set_motor_b(BASE_SPEED);
}

void backward() {
	set_motor_a(-BASE_SPEED);
	set_motor_b(-BASE_SPEED);
}

void turn_right() {
	set_motor_a(TURN_SPEED);
	set_motor_b(-SLOW_TURN);
}

void turn_left() {
	set_motor_a(-SLOW_TURN);
	set_motor_b(TURN_SPEED);
}

// ==========================================
// 7. MÁY TR?NG THÁI: LÁCH V?T C?N
// ==========================================
void process_obstacle_avoidance() {
	// [M?I] N?u ?ang ?i?u khi?n tay (MANUAL) HO?C c? lách v?t c?n b? T?T (avoid_enable == 0)
	// -> H?y b? lách v?t c?n ngay l?p t?c
	if (auto_mode == 0 || avoid_enable == 0) {
		if (is_avoiding != 0) {
			is_avoiding = 0;
			set_servo_target(SERVO_MID); // Tr? th?ng c?m bi?n
		}
		return; // Thoát hàm, không ki?m tra siêu âm n?a
	}
	
	if (is_avoiding == 0 && check_obstacle_confirmed(distance_cm)) {
		move_stop();
		set_servo_target(SERVO_MAX);
		is_avoiding      = 10;
		avoid_start_time = millis();
		steer_snapshot   = current_steer;
		obstacle_count   = 0;
		return;
	}

	if (is_avoiding == 0) return;

	uint32_t current_time = millis();
	uint32_t elapsed      = current_time - avoid_start_time;
	int dynamic_delay;

	switch (is_avoiding) {
		case 10:
		if (elapsed > 600) {
			set_servo_target(SERVO_MID);
			is_avoiding      = (distance_cm > 30) ? 11 : 99;
			avoid_start_time = current_time;
		}
		break;
		case 11:
		if (elapsed > 600) {
			is_avoiding      = 1;
			avoid_start_time = current_time;
		}
		break;
		case 99:
		move_stop();
		if (elapsed > 2000) is_avoiding = 0;
		break;
		case 1:
		backward();
		if (elapsed > 200) { is_avoiding = 2; avoid_start_time = current_time; }
		break;
		case 2:
		turn_left();
		dynamic_delay = 700 - (steer_snapshot * COMPENSATE_K);
		if (dynamic_delay < 150) dynamic_delay = 150;
		if (dynamic_delay > 800) dynamic_delay = 800;
		if (elapsed > (uint32_t)dynamic_delay) { is_avoiding = 3; avoid_start_time = current_time; }
		break;
		case 3:
		forward();
		if (elapsed > 600) { is_avoiding = 4; avoid_start_time = current_time; }
		break;
		case 4:
		turn_right();
		if (elapsed > 800) { is_avoiding = 5; avoid_start_time = current_time; }
		break;
		case 5:
		forward();
		if (elapsed > 800) { is_avoiding = 6; avoid_start_time = current_time; }
		break;
		case 6:
		turn_left();
		if (elapsed > 200) {
			move_stop();
			is_avoiding = 0;
		}
		break;
	}
}

// ==========================================
// 8. KH?I T?O PH?N C?NG
// ==========================================
void Init_Hardware() {
	MOT_L_DDR     |= (1<<MOT_L_IN1) | (1<<MOT_L_IN2) | (1<<MOT_L_PWM_PIN);
	MOT_R_IN_DDR  |= (1<<MOT_R_IN3) | (1<<MOT_R_IN4);
	MOT_R_PWM_DDR |= (1<<MOT_R_PWM_PIN);
	MOT_L_PORT    &= ~((1<<MOT_L_IN1) | (1<<MOT_L_IN2));
	MOT_R_IN_PORT &= ~((1<<MOT_R_IN3) | (1<<MOT_R_IN4));

	TCCR0 = (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS01) | (1<<CS00);
	OCR0  = 0;
	TIMSK |= (1<<TOIE0);

	TCCR2 = (1<<WGM20) | (1<<WGM21) | (1<<COM21) | (1<<CS22);
	OCR2  = 0;

	UBRRH = (unsigned char)(MYUBRR >> 8);
	UBRRL = (unsigned char)MYUBRR;
	UCSRB = (1<<RXEN) | (1<<RXCIE);
	UCSRC = (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);

	DDRD |=  (1 << US_TRIG);
	DDRD &= ~(1 << US_ECHO);
	MCUCR |=  (1 << ISC10);
	MCUCR &= ~(1 << ISC11);
	GICR  |=  (1 << INT1);

	DDRD  |= (1 << SERVO_PIN);
	TCCR1A = (1<<COM1B1) | (1<<WGM11);
	TCCR1B = (1<<WGM13)  | (1<<WGM12) | (1<<CS11);
	ICR1   = SERVO_TOP;
	OCR1B  = SERVO_MID;

	GIFR |= (1 << INTF1);
	sei();
}

// ==========================================
// CH??NG TRÌNH CHÍNH
// ==========================================
int main(void) {
	Init_Hardware();

	uint32_t last_ping_time = 0;

	while (1) {
		uint32_t current_time = millis();

		update_servo_smoothly(current_time);

		if (current_time - last_ping_time >= 50) {
			if (is_waiting_echo) {
				if (current_time - ping_sent_time > ECHO_TIMEOUT_MS) {
					is_waiting_echo = 0;
				}
			}

			if (!is_waiting_echo) {
				is_waiting_echo = 1;
				ping_sent_time  = current_time;

				PORTD |=  (1 << US_TRIG);
				_delay_us(10);
				PORTD &= ~(1 << US_TRIG);

				last_ping_time = current_time;
			}
		}

		if (echo_ready) {
			uint16_t raw_cm = echo_duration / 58;
			distance_cm = filter_distance(raw_cm);
			echo_ready  = 0;
		}

		process_obstacle_avoidance();

		if (data_ready == 1) {
			if (rx_buffer[0] == 'D' && rx_buffer[1] == ':') {
				char *ptr = (char*)&rx_buffer[2];
				int target_speed = atoi(ptr); // L?y V?n t?c

				while (*ptr != ':' && *ptr != '\0') ptr++;

				if (*ptr == ':') {
					ptr++;
					int angle = atoi(ptr); // L?y Góc lái
					current_steer = angle;

					// --- TÌM VÀ L?Y C? AUTO/MANUAL ---
					while (*ptr != ':' && *ptr != '\0') ptr++;
					if (*ptr == ':') {
						ptr++;
						auto_mode = atoi(ptr);
						
						// --- [M?I] TÌM VÀ L?Y C? AVOID_ENABLE (Cho phép lách) ---
						while (*ptr != ':' && *ptr != '\0') ptr++;
						if (*ptr == ':') {
							ptr++;
							avoid_enable = atoi(ptr);
						}
					}

					// Th?c thi ??ng c?
					if (is_avoiding == 0) {
						Motor_Drive(target_speed, angle);
						set_servo_target(SERVO_MID);
					}
				}
			}
			data_ready = 0;
		}
	}
}