////////////////////////////////////////////////////////////////////////////////////////////////
// PIC16F887 - NEC IR (Timer1) + Motor + PWM + HC-SR04 (Timer0) + LCD (CCS C)
// KEY_3: LCD 1.sat?r "asd", 2.sat?r "qwe"
////////////////////////////////////////////////////////////////////////////////////////////////

#define LCD_RS_PIN      PIN_B1
#define LCD_RW_PIN      PIN_B2
#define LCD_ENABLE_PIN  PIN_B3
#define LCD_DATA4       PIN_B4
#define LCD_DATA5       PIN_B5
#define LCD_DATA6       PIN_B6
#define LCD_DATA7       PIN_B7

#include <16F887.h>
#device ADC=10

#fuses XT, NOWDT, NOPUT, NOPROTECT, NOLVP, NOMCLR, NOBROWNOUT
#use delay(crystal=4000000)

#include <lcd.c>

// ---------------- Motor y?n pinleri ----------------
#define MOTOR_IN1     PIN_D0
#define MOTOR_IN2     PIN_D1
#define MOTOR2_IN3    PIN_D2
#define MOTOR2_IN4    PIN_D3   // RC6 UART TX ile ?ak???r (terminal a?acaksan de?i?tir)

// ---------------- PWM ----------------
// PIC16F887: CCP1=RC2, CCP2=RC1  
unsigned int16 pwm_degeri = 500;
#define PWM_MIN    500
#define PWM_MAX   1000
#define PWM_STEP   100

// ---------- NEC IR de?i?kenleri ----------
volatile int1 nec_ok = 0;
volatile unsigned int8 nec_state = 0, command = 0, inv_command = 0, i = 0;
volatile unsigned int16 address = 0;
volatile unsigned int32 nec_code = 0;

// Kumanda tu?lar? (senin de?erlerin)
#define KEY_up     0x18
#define KEY_down   0x4A
#define KEY_right  0x5A
#define KEY_left   0x10
#define KEY_ok     0x38
#define KEY_1      0xA2  // h?z +
#define KEY_2      0x62  // h?z -
#define KEY_3      0xE2  // LCD asd/qwe

// HC-SR04
#define TRIG_PIN PIN_A0
#define ECHO_PIN PIN_A1
volatile int16 distance_cm = -1;

// KEY_3 ekran modu
volatile int1 lcd_key3_mode = 0;

// ---------------- Timer0 overflow counter (HC-SR04 ?l??m? i?in) ----------------
volatile unsigned int16 t0_ovf = 0;

#INT_TIMER0
void t0_isr(void){
   t0_ovf++;
}

// PWM uygula
void pwm_apply(void){
   set_pwm1_duty(pwm_degeri);   // CCP1 -> RC2
   set_pwm2_duty(pwm_degeri);   // CCP2 -> RC1
}

// ---------- Motor fonksiyonlar? ----------
void motors_stop(void){
  output_low(MOTOR_IN1);  output_low(MOTOR_IN2);
  output_low(MOTOR2_IN3); output_low(MOTOR2_IN4);
}
void motors_forward(void){
  output_high(MOTOR_IN1); output_low(MOTOR_IN2);
  output_high(MOTOR2_IN3); output_low(MOTOR2_IN4);
}
void motors_reverse(void){
  output_low(MOTOR_IN1); output_high(MOTOR_IN2);
  output_low(MOTOR2_IN3); output_high(MOTOR2_IN4);
}
void motors_turn_left(void){
  output_high(MOTOR_IN1);  output_low(MOTOR_IN2);
  output_low(MOTOR2_IN3);  output_high(MOTOR2_IN4);
}
void motors_turn_right(void){
  output_low(MOTOR_IN1);   output_high(MOTOR_IN2);
  output_high(MOTOR2_IN3); output_low(MOTOR2_IN4);
}

// LCD g?sterimi
void lcd_show(void){
  if(lcd_key3_mode){
     lcd_gotoxy(1,1);
     printf(lcd_putc, "Ayhan  Yavasoglu"); // 16 karaktere tamamla
     lcd_gotoxy(1,2);
     printf(lcd_putc, "Berke Parlak");
     return;
  }

  unsigned int8 pwm_pct = (unsigned int8)((unsigned int32)pwm_degeri * 100 / 1023);

  lcd_gotoxy(1,1);
  printf(lcd_putc, "CMD:%02X PWM:%3u%%", command, pwm_pct);

  lcd_gotoxy(1,2);
  if(distance_cm < 0){
     printf(lcd_putc, "Dist: --- cm     ");
  } else {
     // CCS: %Lu unsigned long ister -> cast ver
     printf(lcd_putc, "Dist:%3Lu cm     ", (unsigned int32)distance_cm);
  }
}

// ---------------- HC-SR04 distance (Timer0 ile) ----------------
// Timer0: prescaler 1:4 => 1 tick = 4us (4MHz'te instruction cycle 1us)
// Echo_us = ticks * 4
// cm = echo_us / 58  
int16 hcsr04_get_cm_timer0(void)
{
   unsigned int16 guard;
   unsigned int16 ticks;
   unsigned int32 echo_us;

   // TRIG pulse (>=10us)
   output_low(TRIG_PIN);
   delay_us(2);
   output_high(TRIG_PIN);
   delay_us(10);
   output_low(TRIG_PIN);

   // ECHO rising bekle (timeout)
   guard = 0;
   while(!input(ECHO_PIN)){
      delay_us(1);
      if(++guard > 30000) return -1;
   }

   // ?l??m ba?lat
   disable_interrupts(INT_TIMER0);
   t0_ovf = 0;
   set_timer0(0);
   clear_interrupt(INT_TIMER0);
   enable_interrupts(INT_TIMER0);

   // ECHO falling bekle (timeout)
   guard = 0;
   while(input(ECHO_PIN)){
      delay_us(1);
      if(++guard > 30000){
         disable_interrupts(INT_TIMER0);
         return -1;
      }
   }

   // ?l??m? al
   disable_interrupts(INT_TIMER0);
   ticks = (t0_ovf * 256u) + get_timer0();

   // prescaler 1:4 => 4us/tick
   echo_us = (unsigned int32)ticks * 4u;

   return (int16)(echo_us / 58u);
}

// ---------- NEC decode ISR (Timer1) ----------
#INT_EXT
void ext_isr(void){
  unsigned int16 time;

  if(nec_state != 0){
    time = get_timer1();
    set_timer1(0);
  }

  switch(nec_state){

    case 0:
      set_timer1(0);
      nec_state = 1;
      i = 0;
      nec_code = 0;
      ext_int_edge(L_TO_H);
      return;

    case 1:
      if((time > 9500) || (time < 8500)){
        nec_state = 0;
      } else {
        nec_state = 2;
      }
      ext_int_edge(H_TO_L);
      return;

    case 2:
      if((time > 5000) || (time < 4000)){
        nec_state = 0;
        return;
      }
      nec_state = 3;
      ext_int_edge(L_TO_H);
      return;

    case 3:
      if((time > 700) || (time < 400)){
        nec_state = 0;
      } else {
        nec_state = 4;
      }
      ext_int_edge(H_TO_L);
      return;

    case 4:
      if((time > 1800) || (time < 400)){
        nec_state = 0;
        return;
      }

      if(time > 1000) bit_set(nec_code, (31 - i));
      else            bit_clear(nec_code, (31 - i));

      i++;

      if(i > 31){
        address     = (unsigned int16)(nec_code >> 16);
        command     = (unsigned int8)(nec_code >> 8);
        inv_command = (unsigned int8)(nec_code);

        if((command ^ inv_command) == 0xFF){
          nec_ok = 1;
        }
        disable_interrupts(INT_EXT);
      }

      nec_state = 3;
      ext_int_edge(L_TO_H);
      return;
  }
}

void main(){
   setup_adc_ports(NO_ANALOGS);
   setup_adc(ADC_OFF);

   // RD0 TRIG output, RD1 ECHO input
   set_tris_a(0x02);
   set_tris_d(0b00000010);
   set_tris_c(0x00);

   motors_stop();

   // PWM (Timer2)
   setup_timer_2(T2_DIV_BY_4, 255, 1);
   setup_ccp1(CCP_PWM);
   setup_ccp2(CCP_PWM);
   pwm_apply();

   // Timer1: IR i?in 1us/tick (4MHz -> Fosc/4 = 1MHz)
   setup_timer_1(T1_INTERNAL | T1_DIV_BY_1);

   // Timer0: HC-SR04 i?in (prescaler 1:4 => 4us/tick)
   setup_timer_0(RTCC_INTERNAL | RTCC_DIV_4);
   clear_interrupt(INT_TIMER0);
   enable_interrupts(INT_TIMER0);

   // LCD
   lcd_init();
   lcd_putc('\f');
   lcd_show();

   // IR interrupt
   enable_interrupts(GLOBAL);
   enable_interrupts(INT_EXT_H2L);

   while(TRUE){
      // Mesafe ?l?
      distance_cm = hcsr04_get_cm_timer0();

      // IR komut geldiyse uygula
      if(nec_ok){
         nec_ok = 0;

         if(command == KEY_up){
            lcd_key3_mode = 0;
            motors_forward();
         }
         else if(command == KEY_down){
            lcd_key3_mode = 0;
            motors_reverse();
         }
         else if(command == KEY_right){
            lcd_key3_mode = 0;
            motors_turn_right();
         }
         else if(command == KEY_left){
            lcd_key3_mode = 0;
            motors_turn_left();
         }
         else if(command == KEY_ok){
            lcd_key3_mode = 0;
            motors_stop();
         }
         else if(command == KEY_1){
            lcd_key3_mode = 0;
            if(pwm_degeri + PWM_STEP <= PWM_MAX) pwm_degeri += PWM_STEP;
            else pwm_degeri = PWM_MAX;
            pwm_apply();
         }
         else if(command == KEY_2){
            lcd_key3_mode = 0;
            if(pwm_degeri >= PWM_MIN + PWM_STEP) pwm_degeri -= PWM_STEP;
            else pwm_degeri = PWM_MIN;
            pwm_apply();
         }
         else if(command == KEY_3){
            lcd_key3_mode = 1;   // ?zel yaz? modu
         }
         else{
            lcd_key3_mode = 0;
         }

         nec_state = 0;
         enable_interrupts(INT_EXT_H2L);
      }

      lcd_show();
      delay_ms(60);
   }
}
