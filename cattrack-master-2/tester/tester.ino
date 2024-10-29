#define RADIO_FREQ        434.0
#define RADIO_MY_ADDRESS  0x61
#define RADIO_TX_POWER    20
#define RADIO_LONG_RANGE  1

#include "mcurses.h"
#include "printf.h"
#include "mcurses-printf.h"
#include "radio.h"
#include "io.h"
#include "pt/pt.h"

#define MAX_LINE_LENGTH   80
#define MAX_LINES         30
#define MAX_TX_MSG_LENGTH 50

#define MIN_TX_INTERVAL   1
#define MAX_TX_INTERVAL   60

#define MIN_FREQ   430.00
#define MAX_FREQ   470.00

#define MIN_TX_POWER    0
#define MAX_TX_POWER   23

struct pt ptReadline;
struct pt ptRx;
struct pt ptTx;

Radio radio;
bool tx_started = false;
uint32_t tx_interval = 5000;
char tx_message[200];
float radio_freq = RADIO_FREQ;
uint8_t radio_addr = RADIO_MY_ADDRESS;
uint8_t radio_tx_power = RADIO_TX_POWER;

#define PT_DELAY(pt,ms,tsVar) \
  tsVar = millis(); \
  PT_WAIT_UNTIL(pt, millis()-tsVar >= (ms));

/**********************************************/
void Arduino_putchar(uint8_t c)
{
  Serial.write(c);
}

/**********************************************/
char Arduino_getchar()
{
  while (!Serial.available());
  return Serial.read();
}

/**********************************************/
void show_message(const char* msg) {
  scroll();
  move(MAX_LINES+1,0);
  //mc_printf("0x%02X: ", addr);
  addstr((char*)msg);
}

/**********************************************/
void init_screen() {
  clear();
  move(1,0);
  for (int i=0; i<MAX_LINE_LENGTH; i++)
    Serial.print('-');
}

/**********************************************/
void show_help() {
  show_message("Available commands:");
  show_message("-------------------");
  show_message("  a<addr>  - set device address to <addr>");
  show_message("  p<power> - set transmission power to <power> dBm");
  show_message("  f<freq>  - set radio frequency to <freq> MHz");
  show_message("  i<val>   - set transmission period to <val> seconds");
  show_message("  m<msg>   - set transmitted message to <msg>");
  show_message("  t        - start transmitting");
  show_message("  b        - stop transmitting");
  show_message("  r        - reset screen");
  show_message("  ?        - show help message");
}

/**********************************************/
void process_command(const char* cmd) {
  char code = cmd[0];
  char buf[MAX_LINE_LENGTH];

  if (code == 0)
    return;
  else if (code == 'r')
    init_screen();
  else if (code == 'f') {
    float freq = atof(cmd+1);
    if (tx_started)
      sprintf(buf, "Cannot change frequency while transmitting");
    else if (MIN_FREQ <= freq && freq <= MAX_FREQ) {
      sprintf(buf, "Set frequency to %d.%02d MHz", (int)freq, ((int)(freq*100))%100);
      radio_freq = freq;
      radio.init(radio_addr,radio_freq,radio_tx_power,RADIO_LONG_RANGE);
    }
    else
      sprintf(buf, "Frequency must be between %d.%02d and %d.%02d",
        (int)MIN_FREQ, ((int)(MIN_FREQ*100))%100,
        (int)MAX_FREQ, ((int)(MAX_FREQ*100))%100);
  }
  //else if (code == 's') {
  //  uint8_t sf = atoi(cmd+1);
  //  sprintf(buf, "Set SF to %d", sf);
  //}
  else if (code == 'm') {
    strncpy(tx_message, cmd+1, MAX_TX_MSG_LENGTH);
    tx_message[MAX_TX_MSG_LENGTH-1] = 0;
    sprintf(buf, "Set msg to '%s'", cmd+1);
  }
  else if (code == 'a') {
    uint8_t addr = atoi(cmd+1);
    if (tx_started)
      sprintf(buf, "Cannot change address while transmitting");
    else {
      sprintf(buf, "Set my address to %d", addr);
      radio_addr = addr;
      radio.init(radio_addr,radio_freq,radio_tx_power,RADIO_LONG_RANGE);
    }
  }
  else if (code == 'p') {
    int power = atoi(cmd+1);
    if (tx_started)
      sprintf(buf, "Cannot change power while transmitting");
    else if (MIN_TX_POWER <= power && power <= MAX_TX_POWER) {
      sprintf(buf, "Set TX power to %d dBm", power);
      radio_tx_power = power;
      radio.init(radio_addr,radio_freq,radio_tx_power,RADIO_LONG_RANGE);
    }
    else
      sprintf(buf, "TX power must be between %d and %d", MIN_TX_POWER, MAX_TX_POWER);
  }
  else if (code == 'i') {
    uint8_t sec = atoi(cmd+1);
    if (MIN_TX_INTERVAL <= sec && sec <= MAX_TX_INTERVAL) {
      sprintf(buf, "Set tx interval to %d sec", sec);
      tx_interval = sec*1000;
    }
    else {
      sprintf(buf, "Interval must be between %d and %d", MIN_TX_INTERVAL, MAX_TX_INTERVAL);
    }
  }
  else if (code == 't') {
    sprintf(buf, "Transmission started");
    tx_started = true;
  }
  else if (code == 'b') {
    sprintf(buf, "Transmission stopped");
    tx_started = false;
  }
  else if (code == '?') {
    show_help();
  }
  else
    sprintf(buf, "Invalid command");

  // show message if available
  if (buf[0]) {
    buf[MAX_LINE_LENGTH-1] = 0;
    show_message(buf);
  }
}

/**********************************************/
PT_THREAD(taskReadline(struct pt* pt)) {
  static uint_fast8_t ch;
  static uint_fast8_t curlen;
  static uint_fast8_t curpos;
  static uint_fast8_t starty;
  static uint_fast8_t startx;
  static uint_fast8_t i;
  static uint_fast8_t maxlen;
  static char linebuf[MAX_LINE_LENGTH+1];
  static char* str;

  PT_BEGIN(pt);

  for (;;) {
    move(0,0);
    addstr("> ");

    curlen = 0;
    curpos = 0;
    str = linebuf;
    maxlen = MAX_LINE_LENGTH;
    clrtoeol();

    maxlen--;   // reserve one byte in order to store '\0' in last position
    getyx (starty, startx);   // get current cursor position

    for (;;)
    {
      PT_WAIT_UNTIL(pt,Serial.available());

      move (starty, startx + curpos);

      ch = getch();
      if (ch == KEY_CR) break;

      switch (ch)
      {
        case KEY_LEFT:
          if (curpos > 0)
            curpos--;
          break;
        case KEY_RIGHT:
          if (curpos < curlen)
            curpos++;
          break;
        case KEY_HOME:
          curpos = 0;
          break;
        case KEY_END:
          curpos = curlen;
          break;
        case KEY_BACKSPACE:
          if (curpos > 0)
          {
            curpos--;
            curlen--;
            move (starty, startx + curpos);

            for (i = curpos; i < curlen; i++)
            {
              str[i] = str[i + 1];
            }
            str[i] = '\0';
            delch();
          }
          break;

        case KEY_DC:
          if (curlen > 0)
          {
            curlen--;
            for (i = curpos; i < curlen; i++)
              str[i] = str[i + 1];
            str[i] = '\0';
            delch();
          }
          break;

        default:
          if (curlen < maxlen && (ch & 0x7F) >= 32 && (ch & 0x7F) < 127)      // printable ascii 7bit or printable 8bit ISO8859
          {
            for (i = curlen; i > curpos; i--)
              str[i] = str[i - 1];
            insch (ch);
            str[curpos] = ch;
            curpos++;
            curlen++;
          }
      }
    }
    str[curlen] = '\0';
    process_command(str);
  }

  PT_END(pt);
}

/**********************************************/
PT_THREAD(taskRx(struct pt* pt)) {

  char msg[200];
  char hash[200];
  int i;

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt,radio.available());
    Address src;
    uint8_t buf[80];
    uint8_t len = sizeof(buf)-1;
    radio.recv(&src,buf,&len);
    uint16_t rssi_len = (radio.last_rssi + 148)/2;
    buf[len] = '\0';
    for (i=0; i<len; i++)
      if (!(32 <= buf[i] && buf[i] <= 127))
        buf[i] = '.';

    sprintf(msg, "RX: src %d, rssi, %d, msg '%s'", src, radio.last_rssi, buf);
    show_message(msg);

    for (i=0; i<rssi_len; i++)
      hash[i] = '#';
    hash[i] = 0;
    sprintf(msg, "RSSI: %s", hash);
    show_message(msg);
  }

  PT_END(pt);
}

/**********************************************/
void init_radio() {
  Serial.print("Initializing radio...");
  if (!radio.init(RADIO_MY_ADDRESS,RADIO_FREQ,RADIO_TX_POWER,RADIO_LONG_RANGE)) {
    Serial.println("failed!");
    while (1)
      ;
  } Serial.println("done.");
}

/**********************************************/
void setup() {
  PT_INIT(&ptReadline);
  PT_INIT(&ptRx);
  PT_INIT(&ptTx);

  pinMode(LED_BUILTIN,OUTPUT);
  LED_OFF();

  Serial.begin(115200);
  while (!Serial)
    ;
  setFunction_putchar(Arduino_putchar); // tell the library which output channel shall be used
  setFunction_getchar(Arduino_getchar); // tell the library which input channel shall be used

  init_radio();

  initscr();                            // initialize MCURSES
  setscrreg(2,MAX_LINES+2);

  init_screen();
  LED_ON();
}

/**********************************************/
PT_THREAD(taskTx(struct pt* pt)) {

  static uint32_t now;
  static uint32_t seq;
  char buf[200];

  PT_BEGIN(pt);

  seq = 0;
  for (;;) {
    PT_WAIT_UNTIL(pt, tx_started);
    now = millis();
    show_message("Transmitting...");
    uint16_t len = sprintf(buf, "%d: %s", seq, tx_message);
    seq++;
    radio.send(0xFF, (uint8_t*)buf, len);
    PT_WAIT_UNTIL(pt, radio.send_done());
    show_message("tx done.");
    PT_WAIT_UNTIL(pt, millis()-now >= tx_interval);
  }

  PT_END(pt);
}

/**********************************************/
void loop() {
  taskReadline(&ptReadline);
  taskRx(&ptRx);
  taskTx(&ptTx);
}
