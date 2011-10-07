#include <util/delay.h>
#include <avr/sleep.h>

#include <arduino/pins.h>
#include <arduino/serial.h>
#include <arduino/sleep.h>
#include <arduino/adc.h>

#define SERIAL_EOF -256

#define EV_SERIAL (1 << 0)
#define EV_ADC (1 << 1)
static uint8_t events= 0;

static uint16_t adc_value;

static volatile struct {
  uint8_t buf[16];
  uint8_t start;
  uint8_t end;
} serial_input;

static volatile struct {
  uint8_t buf[81];
  uint8_t start;
  uint8_t end;
} serial_output;

serial_interrupt_rx()
{
  uint8_t end = serial_input.end;

  serial_input.buf[end] = serial_read();
  serial_input.end = (end + 1) % sizeof(serial_input.buf);

  events |= EV_SERIAL;
}

serial_interrupt_dre()
{
  uint8_t start = serial_output.start;

  if (start == serial_output.end)
    serial_interrupt_dre_disable();
  else {
    serial_write(serial_output.buf[start]);
    serial_output.start = (start + 1) % sizeof(serial_output.buf);
  }
}

static int
serial_getchar(void)
{
  uint8_t start = serial_input.start;
  int r;

  cli();
  if (start == serial_input.end)
  {
    events&= ~EV_SERIAL;
    sei();
    return SERIAL_EOF;
  }
  sei();

  r = (char)serial_input.buf[start];
  serial_input.start = (start + 1) % sizeof(serial_input.buf);

  return r;
}

static void
serial_print(const char *str)
{
  uint8_t end = serial_output.end;

  while ((serial_output.buf[end] = *str++))
    end = (end + 1) % sizeof(serial_output.buf);

  serial_output.end = end;
  serial_interrupt_dre_enable();
}

static void
sprint_uint16_b10(char *p, uint16_t n)
{
  if (n >= 10000)
    *p++ = '0' + (n / 10000);

  if (n >= 1000)
    *p++ = '0' + (n / 1000) % 10;

  if (n >= 100)
    *p++ = '0' + (n / 100) % 10;

  if (n >= 10)
    *p++ = '0' + (n / 10) % 10;

  *p++ = '0' + n % 10;
  *p++ = '\0';
}

static unsigned char pin13_state= 0;

static void
showstate(void)
{
  char buf[2];
  serial_print("State: ");
  buf[0]= '0' + pin13_state;
  buf[1]= '\0';
  serial_print(buf);
  serial_print(" v=");
  sprint_uint16_b10(buf, adc_value);
  serial_print(buf);
  serial_print("\r\n");
}

static void
handle_serial_input(void)
{
  char buf[10];
  int c;

  while ((c= serial_getchar()) != SERIAL_EOF)
  {
    switch(c)
    {
    case '?':
    case 'h':
      serial_print("?/h: This help\r\ns:   Status\r\n");
      break;
    case 's':
      showstate();
      break;
    default:
      serial_print("Unknown: '");
      buf[0]= c;
      buf[1]= '\0';
      serial_print(buf);
      serial_print("'\r\n");
    }
  }
}

adc_interrupt()
{
  events|= EV_ADC;
}

static void init(void)
{
  serial_baud_9600();
  serial_mode_8n1();
  serial_transmitter_enable();
  serial_receiver_enable();
  serial_interrupt_rx_enable();

  pin13_mode_output();

  adc_reference_internal_5v();
  adc_pin_select(5);
  adc_clock_d128();
  adc_trigger_freerunning();
  adc_trigger_enable();
  adc_interrupt_enable();
  adc_enable();
}

int __attribute__((noreturn)) main()
{
  init();

  sleep_mode_idle();
  sei();

  while (1) {
    /*
     * sleep if no new events need to be handled
     * while avoiding race conditions. see
     * http://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
     */
    cli();
    if (!events) {
      sleep_enable();
      sei();
      sleep_cpu();
      sleep_disable();
      continue;
    }
    sei();

    if (events && EV_ADC)
    {
      adc_value= adc_data();
    }

    if (events & EV_SERIAL)
      handle_serial_input();

    if ((pin13_state= !pin13_state))
      pin13_high();
    else
      pin13_low();
  }
}
