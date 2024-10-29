#define RADIO_FREQ        434.0
#define RADIO_MY_ADDRESS  0x61
#define RADIO_TX_POWER    10
#define RADIO_LONG_RANGE  1

#include "mcurses.h"
#include "printf.h"
#include "mcurses-printf.h"
#include "radio.h"
#include "io.h"
#include "pt/pt.h"

#define MAX_LINE_LENGTH   60
#define MAX_LINES         20

struct pt ptReadline;
struct pt ptRecv;

Radio radio;

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
void new_message(Address addr, uint8_t *msg) {
  scroll();
  move(MAX_LINES+1,0);
  mc_printf("0x%02X: ", addr);
  addstr((char*)msg);
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
  static uint8_t linebuf[MAX_LINE_LENGTH+1];
  static uint8_t* str;

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
    new_message(RADIO_MY_ADDRESS, str);
    radio.send(0xFF,str,curlen);
    PT_WAIT_UNTIL(pt,radio.send_done());
  }

  PT_END(pt);
}

/**********************************************/
PT_THREAD(taskRecv(struct pt* pt)) {

  PT_BEGIN(pt);

  for (;;) {
    PT_WAIT_UNTIL(pt,radio.available());
    Address src;
    uint8_t buf[80];
    uint8_t len = sizeof(buf)-1;
    radio.recv(&src,buf,&len);
    buf[len] = '\0';
    new_message(src,buf);
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
  PT_INIT(&ptRecv);

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
  clear();
  move(1,0);
  Serial.print("--------------------------------------");

  LED_ON();
}

/**********************************************/
void loop() {
  taskReadline(&ptReadline);
  taskRecv(&ptRecv);
}
