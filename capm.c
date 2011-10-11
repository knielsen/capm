#include <util/delay.h>
#include <avr/sleep.h>

#include <arduino/pins.h>
#include <arduino/serial.h>
#include <arduino/sleep.h>
#include <arduino/adc.h>
#include <arduino/timer1.h>
#include <arduino/timer2.h>

#define SERIAL_EOF -256

static uint8_t ev_serial= 0;
static uint8_t ev_adc= 0;
static uint8_t ev_timer= 0;

static uint16_t adc_value;

static volatile struct {
  uint8_t buf[16];
  uint8_t start;
  uint8_t end;
} serial_input;

static volatile struct {
  uint8_t buf[240];
  uint8_t start;
  uint8_t end;
} serial_output;

serial_interrupt_rx()
{
  uint8_t end = serial_input.end;

  serial_input.buf[end] = serial_read();
  serial_input.end = (end + 1) % sizeof(serial_input.buf);

  ev_serial = 1;
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
    ev_serial = 0;
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
serial_hexdump(uint8_t *p, uint8_t n)
{
  int i;
  char buf[2];
  buf[1] = 0;
  for (i = 0; i < 2*n; i++)
  {
    uint8_t d= (p[i/2] >> (4-4*(i % 2))) & 0xf;
    buf[0]= d >= 10 ? ('a' - 10) + d : '0' + d;
    serial_print(buf);
  }
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

static int8_t next_onewire(void);

static void
timer_delay_us(uint16_t usec)
{
  /* For now, assume the timer is already stopped. */
  timer2_count_set(0);
  /* d32 with 16 MHz -> 2 usec resolution. */
  timer2_compare_a_set((usec+1)/2);
  timer2_clock_d32();
}

static uint16_t timer_count;

timer2_interrupt_a()
{
  timer2_clock_off();
  ev_timer = 1;
}

static void
start_timer(void)
{
  timer_count= 0;
  timer_delay_us(500);
  serial_print("Start timer ...\r\n");
}

static void
handle_timer(void)
{
#if 0
  timer_count++;
  if (timer_count >= 2000)    /* 5000 * 200usec -> 1 sec */
    serial_print("Timer up!\r\n");
  else
    timer_delay_us(500);
#else
  next_onewire();
#endif
}


static void
timer_delay_ms(uint16_t msec)
{
  /* For now, assume the timer is already stopped. */
  timer1_count_set(0);
  /* d1024 with 16 MHz -> 64 usec resolution. */
  /* Thus max. wait 65535*64usec = 4.194 sec */
  timer1_compare_a_set(((uint32_t)msec*1000 + 63)/64);
  timer1_clock_d1024();
}

timer1_interrupt_a()
{
  timer1_clock_off();
  timer1_count_set(0);
  ev_timer = 1;
}


#define OW_PIN 9

static void
ow_release(void)
{
  pin_high(OW_PIN);
  /* Enable internal pullup. */
  pin_mode_input(OW_PIN);
}

static void
ow_low(void)
{
  pin_mode_output(OW_PIN);
  pin_low(OW_PIN);
}

static void
ow_high(void)
{
  pin_mode_output(OW_PIN);
  pin_high(OW_PIN);
}

static uint8_t
ow_read(void)
{
  return !!pin_is_high(OW_PIN);
}

struct onewire_cmd {
  int8_t (*cmd)(uint8_t);
  uint8_t count;
};
static const struct onewire_cmd *cmd_list;
static uint8_t cmd_index, cmd_count;

/* Run next step; return 0 if still running, 1 when done, -1 if error. */
static int8_t
next_onewire(void)
{
  int8_t err;
  err= (*cmd_list[cmd_index].cmd)(cmd_count);
  if (err)
    return -1;
  cmd_count++;
  if (cmd_count >= cmd_list[cmd_index].count)
  {
    cmd_count= 0;
    cmd_index++;
    if (!cmd_list[cmd_index].cmd)
      return 1;
  }
  return 0;
}

/* Start a list of commands; return 0 on ok, -1 on error. */
static int8_t
start_cmds(const struct onewire_cmd *cmds)
{
  cmd_list= cmds;
  cmd_index= 0;
  cmd_count= 0;
  return next_onewire();
}

static void
ow_delay_us(uint16_t usec)
{
  timer_delay_us(usec);
}

static void
ow_delay_ms(uint16_t usec)
{
  timer_delay_ms(usec);
}

static uint8_t ow_presence= 0xff;
#define OW_INIT {ow_init, 3}
static int8_t
ow_init(uint8_t i)
{
  if (i == 0)
  {
    /* Reset pulse: low for >= 480 usec. */
    ow_low();
    ow_delay_us(480);
  }
  else if (i == 1)
  {
    /* Presence detect 60 usec <= T <= 240 usec. */
    ow_release();
    ow_delay_us(60);
  }
  else
  {
    /* Total presence pulse 480 usec. */
    ow_presence= !ow_read();
    ow_delay_us(480-60);
  }
  return 0;
}

static int8_t
ow_write_bit(uint8_t bit)
{
  ow_release();
  /* Min. 1 usec recovery between slots. */
  _delay_us(1);
  if (bit)
  {
    /* Write 1: release bus within 1 usec <= T <= 15 usec. */
    /* Let's make that 2 usec just to be a bit on the safe side. */
    cli();
    ow_low();
    _delay_us(2);
    ow_release();
    sei();
    /* Total write pulse >= 60 usec. */
    ow_delay_us(60 - 2);
  }
  else
  {
    /* Write 0: pull low for 60 usec <= T <= 120 usec. */
    ow_low();
    ow_delay_us(60);
  }
  return 0;
}

#define OW_SKIP_ROM {ow_skip_rom, 8}
static int8_t
ow_skip_rom(uint8_t i)
{
  return ow_write_bit(0xcc & (1 << i));
}

#define OW_READ_SCRATCH {ow_read_scratch, 8}
static int8_t
ow_read_scratch(uint8_t i)
{
  return ow_write_bit(0xbe & (1 << i));
}

#define OW_READ_ROM {ow_read_rom, 8}
static int8_t
ow_read_rom(uint8_t i)
{
  return ow_write_bit(0x33 & (1 << i));
}

#define OW_CONVERT_T {ow_convert_t, 9}
static int8_t
ow_convert_t(uint8_t i)
{
  if (i < 8)
    return ow_write_bit(0x44 & (1 << i));
  else
  {
    /* Temperature conversion takes max 750 msec. */
    ow_release();
    ow_delay_ms(750);
    return 0;
  }
}

#define OW_READ_N(n) {ow_read_bit, (n)*8}
uint8_t ow_read_buf[9];
static int8_t
ow_read_bit(uint8_t i)
{
  uint8_t bit;
  /* >= 1usec recovery time. */
  ow_release();
  _delay_us(1);
  /* >= 1 usec pull low to start read slot. */
  /* The read slot is time critical to a few usec, so disable interrupts. */
  cli();
  ow_low();
  _delay_us(1);
  ow_release();
  /*
    We must read the bus within at most 15 usec from pulling the bus low.
    The later we read, the more margin. But let's keep a couple usec to account
    for delays outside of _delay_us().
  */
  _delay_us(12);
  bit= ow_read();
  sei();
  if ((i % 8) == 0)
      ow_read_buf[i / 8] = 0;
  if (bit)
    ow_read_buf[i / 8] |= (1 << (i % 8));
  /* Total read slot >= 60 usec. */
  ow_delay_us(60-1-12);
  return 0;
}

#define OW_END {ow_end, 1}
static int8_t
ow_end(uint8_t i)
{
  serial_print("Done!\r\n");
  return 0;
}

static const struct onewire_cmd
ow_cmds_read_temp_simple[] =
{
  OW_INIT,
  OW_SKIP_ROM,
  OW_CONVERT_T,
  OW_INIT,
  OW_SKIP_ROM,
  OW_READ_SCRATCH,
  OW_READ_N(9),
  OW_END,
  {0,0}
};

static void
start_temp_measure(void)
{
  serial_print("Starting temperature conversion...\r\n");
  start_cmds(ow_cmds_read_temp_simple);
}

static const struct onewire_cmd
ow_cmds_read_rom[] =
{
  OW_INIT,
  OW_READ_ROM,
  OW_READ_N(8),
  OW_END,
  {0,0}
};

static void
start_read_rom(void)
{
  serial_print("Reading rom...\r\n");
  start_cmds(ow_cmds_read_rom);
}

static uint16_t num_adc_reads= 0;
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
  serial_print(" tc=");
  sprint_uint16_b10(buf, timer_count);
  serial_print(buf);
  serial_print(" adc_reads=");
  sprint_uint16_b10(buf, num_adc_reads);
  serial_print(buf);
  serial_print(" ds=");
  sprint_uint16_b10(buf, ow_presence);
  serial_print(buf);
  serial_print(" ");
  serial_hexdump(ow_read_buf, 9);
  serial_print(" ow:");
  sprint_uint16_b10(buf, cmd_index);
  serial_print(buf);
  serial_print(" ");
  sprint_uint16_b10(buf, cmd_count);
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
      serial_print("?/h: This help\r\ns:   Status\r\nt:   Start timer\r\nT:   Start temp. measure\r\nr:   Read rom\r\n");
      break;
    case 's':
      showstate();
      break;
    case 't':
      start_timer();
      break;
    case 'T':
      start_temp_measure();
      break;
    case 'r':
      start_read_rom();
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
  ev_adc = 1;
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

  timer2_mode_normal();
  /* Timer resolution 2 usec -> max 512 usec delay */
  timer2_clock_off();
  timer2_count_set(0);
  timer2_compare_a_set(255);
  timer2_interrupt_a_enable();

  timer1_mode_normal();
  timer1_clock_off();
  timer1_count_set(0);
  timer1_compare_a_set(0xffff);
  timer1_interrupt_a_enable();
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
    if (!ev_serial && !ev_adc && !ev_timer) {
      sleep_enable();
      sei();
      sleep_cpu();
      sleep_disable();
      continue;
    }
    sei();

    if (ev_adc)
    {
      ev_adc = 0;
      adc_value= adc_data();
      if (num_adc_reads < 0xffff)
        num_adc_reads++;
    }

    if (ev_serial)
      handle_serial_input();

    if (ev_timer)
    {
      ev_timer = 0;
      handle_timer();
    }

    if ((pin13_state= !pin13_state))
      pin13_high();
    else
      pin13_low();
  }
}
