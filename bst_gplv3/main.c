/* firmware for some 3360 mouse

   Copyright (c) 2016 qsxcv

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/

// this code assumes

// 3360 NCS      <-> B0
// 3360 SCLK    <-> B1
// 3360 MOSI    <-> B2
// 3360 MISO    <-> B3
// 3360 NRESET set to high by pcb.
// wheel B      <-> C6 (wheel up)
// wheel A      <-> C7 (wheel down)
// left button    <-> D0
// right button   <-> D1
// wheel button   <-> D2
// L back side    <-> D3
// L forwards side  <-> D4
// dpi button   <-> D5

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "srom_3360_0x03.h"
#include "usb_mouse.h"

#define delay_us(t) __builtin_avr_delay_cycles((t) * (F_CPU/1000000))
#define delay_ms(t) __builtin_avr_delay_cycles((t) * (F_CPU/1000))

#define PORT_SPI PORTB
#define DDR_SPI DDRB

#define DD_SS 0 // aka NCS
#define DD_SCK  1
#define DD_MOSI 2
#define DD_MISO 3

#define SS_LOW  (PORT_SPI &= ~(1<<DD_SS))
#define SS_HIGH (PORT_SPI |= (1<<DD_SS))

#define DEBOUNCE_TIME 200 // debounce time in units of 125us. only affects release latency.
//#define HARDEB  // hardware debouncing, disable MENU to avoid pin conflicts. pins can be randomly assigned from port F and D
//#define MIX   // allow a mix of 3 pin hardware and 2 pin software debounced switches. must be defined when HARDEB is defined
//#define MENU  // disable this with HARDEB enabled to avoid pin conflicts, or change the conflicting pins.
//#define INVERTX // to use a sensor rotated 180 degrees, invert both axes.
//#define INVERTY // Y axis inversion, for games that dont support it.
//#define INVERTWHL // wheel turns the wrong way.
#define WHLBUX  // ben buxtons rotary encoder code
//#define WHLACC  //have to use WHLBUX and WHLACC together, not compatible with older wheel code.
//#define RGB   // the classic intellimouse code from youtube, dont use with HARDEB enabled unless you fix pin conflicts.
#define SIXTEENMHZ  // 16MHZ crystal, needs 8mhz for 3.3v mcu. F_CPU in makefile must be set to 16000000 or 8000000

/* Use the full-step state table (emits a code at 00 only) sums to 0xf8 or 248
const uint8_t ttable[7][4] = {
  {0x0, 0x2, 0x4,  0x0}, {0x3, 0x0, 0x1, 0x40},
  {0x3, 0x2, 0x0,  0x0}, {0x3, 0x2, 0x1,  0x0},
  {0x6, 0x0, 0x4,  0x0}, {0x6, 0x5, 0x0, 0x80},
  {0x6, 0x5, 0x4,  0x0},
};
*/

#ifndef WHLBUX
#define WHL_A_IS_HIGH (!!(PINC & (1<<7)))
#define WHL_B_IS_HIGH (!!(PINC & (1<<6)))
#endif

#ifdef WHLBUX
volatile int8_t state = 0;
// Use the half-step state table (emits a code at 00 and 11) sums to 0x1b0 or 432
const uint8_t ttable[6][4] = {
  {0x3 , 0x2, 0x1,  0x0}, {0x83, 0x0, 0x1,  0x0},
  {0x43, 0x2, 0x0,  0x0}, {0x3 , 0x5, 0x4,  0x0},
  {0x3 , 0x3, 0x4, 0x40}, {0x3 , 0x5, 0x3, 0x80}
};

int8_t oldpinstate = 0;
uint8_t rotary_process(int8_t pinstate)
{ 
  	state = ttable[state & 0xf][pinstate];
  	return (state & 0xc0);
} 
#endif
// use this instead of bitshifts or LSB/MSB macros.
union motion_data {
  int16_t all;
  struct {
    uint8_t lo, hi;
  };
};

static void pins_init(void)
{

  // buttons
#ifndef HARDEB
  PORTD |= 0b00111111; // L, R, M, FSB, RSB, DPI
#endif
#ifdef HARDEB
  PORTD |= 0b10000111; //
  PORTF |= 0b11110011; //
  DDRD &= 0b10000111; //pullup inputs
  DDRF &= 0b11110011; //pullup inputs
#endif

  PORTD |=(1<<6);

  // wheel (optical encoder, quadrature outputs A/B)
  DDRC &= ~((1 << 6) | (1 << 7)); // disable pullup inputs on C6, C7
  PORTC |= (1 << 6) | (1 << 7);

  // teensy LED
  DDRD |= (1 << 6);
  
#ifdef RGB
  // RGB LED
  DDRB |= (1 << 5) | (1 << 6);
  DDRD |= (1 << 7);
  PORTB |= (1 << 5); // Red (off)
  PORTB |= (1 << 6); // Green (off)
  PORTD |= (1 << 7); // Blue (off)
#endif
 
  // not necssary if NRESET is pulled high in PCB
  //DDRC |= (1<<7); PORTC |= (1<<7); // C7, NRESET high output

  EICRA = 0b01010101; // generate interrupt request on any edge of D0/D1/D2/D3
  EIMSK = 0; // but don't enable any actual interrupts
  EIFR = 0b00001111; // clear EIFR
}


// spi functions
static void spi_init(void)
{
  DDR_SPI |= (1 << DD_MOSI) | (1 << DD_SCK) | (1 << DD_SS); // outputs
  DDRB |= (1 << 0); PORTB |= (1 << 0); // set the hardware SS pin to low to enable SPI
  // MISO pullup input is already done in hardware
  // enable spi, master mode, mode 3, clock rate = fck/4 = 2MHz
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA);
}

static inline void spi_send(const uint8_t b)
{
  SPDR = b;
  while (!(SPSR & (1 << SPIF)));
}

static inline uint8_t spi_recv(void)
{
  spi_send(0x00);
  return SPDR;
}

static inline void spi_write(const uint8_t addr, const uint8_t data)
{
  spi_send(addr | 0x80);
  spi_send(data);
  delay_us(180); // maximum of t_SWW, t_SWR
}

static inline uint8_t spi_read(const uint8_t addr)
{
  spi_send(addr);
  delay_us(160); // t_SRAD
  uint8_t data = spi_recv();
  delay_us(20);
  return data;
}

static void pmw3360_init(const uint8_t dpi)
{
  const uint8_t *psrom = srom;

  SS_HIGH;
  delay_ms(3);

  // shutdown first
  SS_LOW;
  spi_write(0x3b, 0xb6);
  SS_HIGH;
  delay_ms(300);

  // drop and raise ncs to reset spi port
  SS_LOW;
  delay_us(40);
  SS_HIGH;
  delay_us(40);

  // power up reset
  SS_LOW;
  spi_write(0x3a, 0x5a);
  SS_HIGH;
  delay_ms(50);

  // read from 0x02 to 0x06
  SS_LOW;
  spi_read(0x02);
  spi_read(0x03);
  spi_read(0x04);
  spi_read(0x05);
  spi_read(0x06);

  // srom download
  spi_write(0x10, 0x00);
  spi_write(0x13, 0x1d);
  SS_HIGH;
  delay_ms(10);
  SS_LOW;
  spi_write(0x13, 0x18);

  spi_send(0x62 | 0x80);
  for (uint16_t i = 0; i < SROM_LENGTH; i++) {
    delay_us(16);
    spi_send(pgm_read_byte(psrom++));
  }
  delay_us(18);
  SS_HIGH;
  delay_us(200);

  // configuration/settings
  SS_LOW;
  spi_write(0x10, 0x20); // Rest mode & independant X/Y DPI disabled , (0x10, 0x20) for wireless rest enabled.
  spi_write(0x0d, 0x00); // Camera angle
  spi_write(0x11, 0x00); // Camera angle fine tuning
  spi_write(0x0f, dpi); // DPI
  // LOD Stuff
  spi_write(0x63, 0x03); // LOD: 0x00 disable lift detection, 0x02 = 2mm, 0x03 = 3mm
  spi_write(0x2b, 0x10); // Minimum SQUAL for zero motion data (default: 0x10)
  spi_write(0x2c, 0x0a); // Minimum Valid features (reduce SQUAL score) (default: 0x0a)
  SS_HIGH;
  delay_us(200);
}

// angle snapping
static void angle_init(const uint8_t angle) {
  SS_LOW;
  spi_write(0x42, angle); // Angle snapping: 0x00 = off, 0x80 = on
  SS_HIGH;
}

#ifdef HARDEB
uint8_t btn_dbncd = 0x00;
#endif

#ifdef WHLACC
//y=ax+c line
int8_t whl_time = 0;
int8_t whl_eighth = 0;
int8_t rev = 0;
int8_t stack = 0;
int8_t _rev = 18;
#endif

int main(void)
{
  union motion_data x, y;

  // set clock prescaler for 16MHz
  CLKPR = 0x80;

#ifdef SIXTEENMHZ
  CLKPR = 0x00; //0x00==16MHZ, 0x01==8MHZ
#endif

#ifndef SIXTEENMHZ
  CLKPR = 0x00; //0x00==16MHZ, 0x01==8MHZ
#endif

  pins_init();

  // previous state to compare against for debouncing
  uint8_t btn_prev = (~PIND) & 0b00111111; // read L, R, M, FSB, RSB, DPI
    // time (in 125us) button has been unpressed.
  // consider button to be released if this time exceeds DEBOUNCE_TIME.
   // binary OR of all button states since previous usb transmission
#ifdef MIX
  uint8_t btn_time[8] = {0, 0, 0, 0, 0, 0, 0, 0};
#endif
#ifndef HARDEB
  uint8_t btn_time[8] = {0, 0, 0, 0, 0, 0, 0, 0};
#endif
#ifdef RGB
  // RGB Timers
  uint8_t led_max_timer = 16; // Timer maximum
  uint8_t led_r_timer = 0; // Red LED Timer initial value
  uint8_t led_g_timer = 0; // Green LED Timer initial value
  uint8_t led_b_timer = 0; // Blue LED Timer initial value
  // RGB Brightness
  uint8_t led_bright_index = 4;
  float led_bright[] = {0, 0.25, 0.5, 0.75, 1};
  float led_rgb_brightness = led_bright[led_bright_index];
  // RGB Initial Values
  uint8_t led_r_value = 0; // Red LED brightness: 0-255
  uint8_t led_g_value = 0; // Green LED brightness: 0-255
  uint8_t led_b_value = 0; // Blue LED brightness: 0-255
  // RGB Colour Selection
  uint8_t led_colours_index = 1;
  uint8_t led_colours[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  uint8_t led_colour = led_colours[led_colours_index];
  // Profiles Intitial Values
  uint8_t p1_led_colour = 1; // Red
  uint8_t p2_led_colour = 2; // Pink
  uint8_t p3_led_colour = 4; // Purple
  uint8_t p1_led_brightness = 4; // 0 = 0, 1 = 0.25, 2 = 0.5, 3 = 0.75, 4 = 1
  uint8_t p2_led_brightness = 4;
  uint8_t p3_led_brightness = 4;
#endif
  // binary OR of all button states since previous usb transmission
  uint8_t btn_usb = 0x00;
  // previously transmitted button state
  uint8_t btn_usb_prev = 0x00;
  // if dpi button is pressed when plugging in, jump to bootloader
  // see https://www.pjrc.com/teensy/jump_to_bootloader.html
  delay_ms(50);
#ifndef HARDEB
  if (!(PIND & (1 << 1)))
    __asm__ volatile ("jmp 0x7e00");
#endif
#ifdef HARDEB
  if (!(PINF & (1 << 4)))
    __asm__ volatile ("jmp 0x7e00");
#endif

   // wheel stuff
#ifndef WHLBUX
  uint8_t whl_prev_same = 0; // what A was the last time A == B
  uint8_t whl_prev_diff = 0; // what A was the last time A != B
#endif
  int8_t whl = 0; // scrolls since previous usb transmission

  spi_init();

  // dpi settings
  // dpi argument is what's written to register 0x0f
  // actual dpi value = (dpi + 1) * 100
  uint8_t dpi_index = 1;
  uint8_t dpis[] = {3, 7, 15};

  // Angle snapping settings
  uint8_t angle_index = 0;
  uint8_t angles[] = {0x00, 0x80}; // Off, On
  
  // Profile switching when mouse is plugged in
  delay_ms(50);
#ifdef MENU
  if (!(PIND & (1 << 3))) {
#ifdef RGB
    // Initial profile
    profile = 2;
#endif
    // dpi settings
    dpi_index = 0;
  }

  else if (!(PIND & (1 << 4))) {
#ifdef RGB
    // Initial profile
    profile = 3;
#endif
    // dpi settings
    dpi_index = 2;
  }
#endif
#ifdef RGB
  // Initial profile
  uint8_t profile = 1; //generates compiler warning unless rgb is enabled
#endif

  // Init 3360
  pmw3360_init(dpis[dpi_index]);

  // Init angle snapping
  angle_init(angles[angle_index]);

  usb_init();
  while (!usb_configured());
  delay_ms(456); // arbitrary

  // set up timer0 to set OCF0A in TIFR0 every 125us
  TCCR0A = 0x02; // CTC
  TCCR0B = 0x02; // prescaler 1/8 = 1us period

#ifdef SIXTEENMHZ
  OCR0A = 249; // = 125 - 1
#endif
#ifndef SIXTEENMHZ
  OCR0A = 124; // = 125 - 1
#endif

  cli();
  while (1) {
    for (uint8_t i = 0; i < 8; i++) {
      // synchronization to usb frames and 125us intervals
      // polling interrupt flags gives 5 clock cycles or so of
      // jitter. possible to eliminate by going into sleep
      // mode and waking up using interrupts, but whatever.
      if (i == 0) {
        // sync to usb frames (1ms)
        UDINT &= ~(1 << SOFI);
        while (!(UDINT & (1 << SOFI)));
        // reset prescaler phase, not really necessary
        GTCCR |= (1 << PSRSYNC);
        TCNT0 = 0;
      } else {
        // sync to 125us intervals using timer0
        while (!(TIFR0 & (1 << OCF0A)));
      }
      TIFR0 |= (1 << OCF0A); // 0CF0A is cleared by writing 1
#ifdef RGB
      // RGB ------------------------------
      // Colours
      // Brightness values 0-255. When adding new colours here, add their number to led_colours[] and change value of (pX_led_colour < 3) to suit.
      // White
      if (led_colour == 1) {
        led_r_value = 180;
        led_g_value = 255;
        led_b_value = 255;
      }
      // Red
      if (led_colour == 2) {
        led_r_value = 255;
        led_g_value = 0;
        led_b_value = 0;
      }
      // Pink
      if (led_colour == 3) {
        led_r_value = 255;
        led_g_value = 0;
        led_b_value = 60;
      }
      // Magenta
      if (led_colour == 4) {
        led_r_value = 255;
        led_g_value = 0;
        led_b_value = 255;
      }
      // Violet
      if (led_colour == 5) {
        led_r_value = 100;
        led_g_value = 0;
        led_b_value = 255;
      }
      // Blue
      if (led_colour == 6) {
        led_r_value = 0;
        led_g_value = 0;
        led_b_value = 255;
      }
      // Sky Blue
      if (led_colour == 7) {
        led_r_value = 0;
        led_g_value = 90;
        led_b_value = 255;
      }
      // Cyan
      if (led_colour == 8) {
        led_r_value = 0;
        led_g_value = 255;
        led_b_value = 255;
      }
      // Green
      if (led_colour == 9) {
        led_r_value = 0;
        led_g_value = 255;
        led_b_value = 0;
      }
      // Toxic Green
      if (led_colour == 10) {
        led_r_value = 0;
        led_g_value = 255;
        led_b_value = 64;
      }
      // Yellow
      if (led_colour == 11) {
        led_r_value = 200;
        led_g_value = 255;
        led_b_value = 0;
      }
      // Orange
      if (led_colour == 12) {
        led_r_value = 255;
        led_g_value = 100;
        led_b_value = 0;
      }

      // Convert RGB values from x/255 to same percentage as max timer value
      uint8_t led_r_intensity = ((led_r_value / led_max_timer) + 0.5);
      uint8_t led_g_intensity = ((led_g_value / led_max_timer) + 0.5);
      uint8_t led_b_intensity = ((led_b_value / led_max_timer) + 0.5);

      // // RGB Brightness * individual LED brightness
      uint8_t led_r_on = ((led_rgb_brightness * led_r_intensity) + 0.5);
      uint8_t led_g_on = ((led_rgb_brightness * led_g_intensity) + 0.5);
      uint8_t led_b_on = ((led_rgb_brightness * led_b_intensity) + 0.5);

      // Calculates number to turn off LED
      uint8_t led_r_off = (led_r_on + 1);
      uint8_t led_g_off = (led_g_on + 1);
      uint8_t led_b_off = (led_b_on + 1);

      // Red LED PWM
      // Turn off LED if led_x_on is 0
      if (led_r_on == 0) {
        PORTB |= (1 << 5);
      }
      // If led_x_on is greater than zero, enable the timer
      if (led_r_on > 0) {
        // LED Timer
        led_r_timer++;
        // Reset Timer
        if (led_r_timer == led_max_timer) {
          led_r_timer = 0;
        }
        // Turn on LED at led_x_on value
        if (led_r_timer <= led_r_on) {
          PORTB &= ~(1 << 5);
        }
        // Turn off LED at led_x_off value
        if (led_r_timer >= led_r_off) {
          PORTB |= (1 << 5);
        }
      }

      // Green LED PWM
      // Turn off LED if led_x_on is 0
      if (led_g_on == 0) {
        PORTB |= (1 << 6);
      }
      // If led_x_on is greater than zero, enable the timer
      if (led_g_on > 0) {
        // LED Timer
        led_g_timer++;
        // Reset Timer
        if (led_g_timer == led_max_timer) {
          led_g_timer = 0;
        }
        // Turn on LED at led_x_on value
        if (led_g_timer <= led_g_on) {
          PORTB &= ~(1 << 6);
        }
        // Turn off LED at led_x_off value
        if (led_g_timer >= led_g_off) {
          PORTB |= (1 << 6);
        }
      }

      // Blue LED PWM
      // Turn off LED if led_x_on is 0
      if (led_b_on == 0) {
        PORTD |= (1 << 7);
      }
      // If led_x_on is greater than zero, enable the timer
      if (led_b_on > 0) {
        // LED Timer
        led_b_timer++;
        // Reset Timer
        if (led_b_timer == led_max_timer) {
          led_b_timer = 0;
        }
        // Turn on LED at led_x_on value
        if (led_b_timer <= led_b_on) {
          PORTD &= ~(1 << 7);
        }
        // Turn off LED at led_x_off value
        if (led_b_timer >= led_b_off) {
          PORTD |= (1 << 7);
        }
      }
#endif
      // sensor stuff
      union motion_data _x, _y;
      SS_LOW;
      spi_send(0x50);
      delay_us(35);
      spi_send(0x00); // motion, not used
      spi_send(0x00); // observation, not used
      _x.lo = spi_recv();
      _x.hi = spi_recv();
      _y.lo = spi_recv();
      _y.hi = spi_recv();
      SS_HIGH;

      // wheel stuff
      int8_t _whl = 0; // number of scrolls this 125us
#ifndef WHLBUX
      const uint8_t whl_a = WHL_A_IS_HIGH;
      const uint8_t whl_b = WHL_B_IS_HIGH;
      // calculate number of scrolls
      if (whl_a != whl_b)
        whl_prev_diff = whl_a;
      else if (whl_a != whl_prev_same) {
        _whl = 2 * (whl_a ^ whl_prev_diff) - 1;
        whl_prev_same = whl_a;
      }
#endif
#ifdef WHLBUX
	  int8_t pinstate = 0;
  	  if(!bit_is_clear(PINC, PC7)){
	  	pinstate |= (1<<1); 
      }
      if(!bit_is_clear(PINC, PC6)){
	  	pinstate |= (1<<0);
      }
      if((pinstate & 0b00000011) != (oldpinstate & 0b00000011)){ 
      	uint8_t result = rotary_process(pinstate);
		oldpinstate = pinstate & 0b00000011;
	  	if (result & (1<<6)){
		#ifndef WHLACC
			_whl = -1;
		#endif
		#ifdef WHLACC
          if (whl_time < 15) {
            rev = 15 - whl_time;
            rev = (rev >> 2) + 4;
            stack = stack - rev;
          }
          else {
            _rev = whl_time;
            stack--;
          }
          whl_time = 0;
		#endif
	  	}
	  	if (result & (1<<7)){
		#ifndef WHLACC
			_whl = 1;
		#endif
		#ifdef WHLACC
          if (whl_time < 15) {
            rev = 15 - whl_time;
            rev = (rev >> 2) + 4;
            stack = stack + rev;
          }
          else {
            _rev = whl_time;
            stack++;
          } 
          whl_time = 0;
		#endif
	  	}
	  }
		#ifdef WHLACC
      if (_rev < 1) {
        if (stack > 0) {
          stack--;
          _whl = 1;
        }
        if (stack < 0) {
          stack++;
          _whl = -1;
        }
      }

      if (whl_eighth > 7) {
        whl_eighth = 0;
        if (_rev > 0) {
          _rev--;
          if (stack > 0) {
            stack--;
            _whl = 1;
          }
          if (stack < 0) {
            stack++;
            _whl = -1;
          }
        }
        if (whl_time < 125) {
          whl_time++;
        }
      }
      whl_eighth++;
		#endif
#endif
	  
	  // button stuff
      //high = not pressed, low = pressed
      //PIND 0 EIFR 0: low, no edges -> is low
      //PIND 0 EIFR 1: low, edge -> is low
      //PIND 1 EIFR 0: high, no edges -> always high during last 125us
      //PIND 1 EIFR 1: high, edge -> low at some point in the last 125us


#ifndef HARDEB
      const uint8_t btn_unpressed = PIND & (~(EIFR) | 0b00110000);
      EIFR = 0b00001111; // clear EIFR (clear interrupts)
      uint8_t btn_dbncd = 0x00;
#endif

      // manual loop debouncing for every button

      // button debouncing logic
      //          >input<           |        >output<
      //------------------------------------------------------
      // previous    | current      | unclicked  | current
      // dbncd state | actual state | time       | dbncd state
      //-------------+--------------+------------+------------
      //    btn_prev |   ~unpressed | btn_time   |   btn_dbncd
      //-------------+--------------+------------+------------
      //           0 |            0 |         =0 |          =0
      //           0 |            1 |         =0 |          =1
      //           1 |            0 |         ++ | (time < DEBOUNCE_TIME)
      //           1 |            1 |         =0 |          =1
#ifdef HARDEB
      btn_dbncd &= ~0b00100000;
#endif

#ifndef HARDEB
#define DEBOUNCE(index) \
  if ((btn_prev & (1<<index)) && (btn_unpressed & (1<<index))) { \
    btn_time[index]++; \
    if (btn_time[index] < DEBOUNCE_TIME) \
      btn_dbncd |= (1<<index); \
  } else { \
    btn_time[index] = 0; \
    btn_dbncd |= (~btn_unpressed) & (1<<index); \
  }
      DEBOUNCE(0); // L
      DEBOUNCE(1); // R
      DEBOUNCE(2); // M
      DEBOUNCE(3); // RSB
      DEBOUNCE(4); // FSB
      DEBOUNCE(5); // DPI
#undef DEBOUNCE
#endif

#ifdef MIX
      const uint8_t btn_unpressed = PIND & (~(EIFR) | 0b00000100); //(set soft pins 1 here and 0 next line)
      EIFR = 0b00111011; // clear EIFR
      btn_dbncd &= ~0b00100100; // to do a mix of soft and hard debouncing, set the bit to 1 and below here comment out all
      // other DEBOUNCE() while Define HARDEB. comment out or #ifndef MIX(prefered) both btn_dbncd references per switch.
      // dpi bit left alone  in btn_dbncd so it doesnt self trigger.
#define DEBOUNCE(index) \
  if ((btn_prev & (1<<index)) && (btn_unpressed & (1<<index))) { \
    btn_time[index]++; \
    if (btn_time[index] < DEBOUNCE_TIME) \
      btn_dbncd |= (1<<index); \
  } else { \
    btn_time[index] = 0; \
    btn_dbncd |= (~btn_unpressed) & (1<<index); \
  }

      //DEBOUNCE(0); // L
      //DEBOUNCE(1); // R
      DEBOUNCE(2); // M
      //DEBOUNCE(3); // RSB
      //DEBOUNCE(4); // FSB
      DEBOUNCE(5); // DPI

#undef DEBOUNCE
#endif

#ifdef HARDEB
      uint8_t btn_f = PINF & 0b11110011;
      uint8_t btn_d = PIND & 0b10000111;

      if (!(btn_f & (1 << 0))) { //left hand primary +click
        btn_dbncd |= (1 << 0); //+left
      }

      if (!(btn_f & (1 << 1))) { //left hand primary -click
        btn_dbncd &= ~(1 << 0); //-left
      }

      if (!(btn_f & (1 << 4))) { //right hand secondary +click
        btn_dbncd |= (1 << 1); //+right
      }

      if (!(btn_d & (1 << 7))) { //right hand secondary -click
        btn_dbncd &= ~(1 << 1); //-right
      }

#ifndef MIX
      if (!(btn_d & (1 << 2))) {
        btn_dbncd |= (1 << 2); //+middle +click
        _whl = 0;
      }

      if (!(btn_f & (1 << 5))) { //-click
        btn_dbncd &= ~(1 << 2); //-middle
      }
#endif

      if (!(btn_f & (1 << 7))) { //right hand side +click
        btn_dbncd |= (1 << 4); //+forward
      }

      if (!(btn_f & (1 << 6))) { //right hand side -click
        btn_dbncd &= ~(1 << 4); //-forward
      }

      if (!(btn_d & (1 << 1))) { //left hand side +click
        btn_dbncd |= (1 << 3); //+back
      }

      if (!(btn_d & (1 << 0))) { //left hand side -click
        btn_dbncd &= ~(1 << 3); //-back
      }
#endif
#ifndef HARDEB
      if (btn_dbncd & (1 << 2)) {
        _whl = 0;
      }
#endif
#ifdef MIX
      if (btn_dbncd & (1 << 2)) {
        _whl = 0;
      }
#endif
#ifdef RGB
      // Profiles ------------------------
      if (profile == 1) {
        led_colours_index = p1_led_colour;
        led_colour = led_colours[led_colours_index];
        led_bright_index = p1_led_brightness;
        led_rgb_brightness = led_bright[led_bright_index];
      }

      if (profile == 2) {
        led_colours_index = p2_led_colour;
        led_colour = led_colours[led_colours_index];
        led_bright_index = p2_led_brightness;
        led_rgb_brightness = led_bright[led_bright_index];
      }

      if (profile == 3) {
        led_colours_index = p3_led_colour;
        led_colour = led_colours[led_colours_index];
        led_bright_index = p3_led_brightness;
        led_rgb_brightness = led_bright[led_bright_index];
      }
#endif
        // usb
        // first make sure it's configured
        sei();
        while (!usb_configured());
        cli();

        // this stuff is very intricate and confusing
        // i'm fairly certain all of it is correct.

        // mask dpi button state for usb
        const uint8_t btn_dbncd_mask = btn_dbncd & 0b00111111;
        // there's nothing to do if nothing's changed in this 125us cycle
        if ((btn_dbncd_mask != (btn_prev & 0b00111111)) || _x.all || _y.all || _whl) {
          UENUM = MOUSE_ENDPOINT;
          if (UESTA0X & (1 << NBUSYBK0)) { // untransmitted data still in bank
            UEINTX |= (1 << RXOUTI); // kill bank; RXOUTI == KILLBK
            while (UEINTX & (1 << RXOUTI));
          } else {
            // transmission's finished, or the data that should be in the
            // bank is exactly the same as what was previously transmitted
            // so that there was nothing worth transmitting before.
            btn_usb_prev = btn_usb;
            btn_usb = 0x00;
            x.all = 0;
            y.all = 0;
            whl = 0;
          }

        btn_usb |= btn_dbncd_mask & 0b00011111; // L, R, M, FSB, RSB
#ifndef INVERTX
        x.all += _x.all;
#endif
#ifdef INVERTX
        x.all -= _x.all;
#endif
#ifndef INVERTY
        y.all += _y.all;
#endif
#ifdef INVERTY
        y.all -= _y.all;
#endif
#ifndef INVERTWHL
        whl += _whl;
#endif
#ifdef INVERTWHL
        whl -= _whl;
#endif
          // only load bank with data if there's something worth transmitting
          if ((btn_usb != btn_usb_prev) || x.all || y.all || whl) {
            UEDATX = btn_usb;
            UEDATX = x.lo;
            UEDATX = x.hi;
            UEDATX = y.lo;
            UEDATX = y.hi;
            UEDATX = whl; // wheel scrolls
            UEINTX = 0x3a;
          }
        }
      
      // update btn_prev
      btn_prev = btn_dbncd;
    }
  }
}

