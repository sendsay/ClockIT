#define DIN_PIN   11                                                                   //GPIO 13 / D7
#define CS_PIN    10                                                                    //GPIO 15 / D8
#define CLK_PIN   12
#define MAX_DIGITS 16
#define NUM_MAX 4
#define TONEPIN 8

#define DEBUG           //DEBUG MODE

byte del = 0;                             // ???
byte dig[MAX_DIGITS] = {0};               // ???
byte digold[MAX_DIGITS] = {0};            // ???
byte digtrans[MAX_DIGITS] = {0};          // ???
int rotate = 90;        //поворот матрицы
int dx = 0;             //координаты
int dy = 0;             //координаты
int hour=22, minute=40, second=42, month=4, day=6, dayOfWeek=6, year=2018;
int secFr, lastSecond, lastMinute;                    // Работа с временем
bool ShowFlag = false;                                // Показывать данные с датчиков
// byte digPos[6] = {1, 8, 18, 25, 15, 16};
byte digPos[6] = {1, 8, 19, 26, 15, 16};
byte digPosSet[4] = {1, 9, 17, 25};

float t5 = 0;                           // ??
int t4 = 0;                             // ??
int t3 = 85;                            // ??
int t2 = 0;                             // ??
int t1 = 85;                            // ??

boolean flash = false;                              //flash dots

enum modes {CLOCK, TEMP, HUM, DAYOFWEEK, DAYMON, END_MODES, TIMER, TIMERALARM, SETTIME};
int mode = 0;      //show mode
byte h1 = 0;
byte h2 = 0;
byte h3 = 0;

int digPosDay[3] = {7, 14, 20};

int dayofweek[7][3] = {{77, 111, 110}, {84, 117, 101}, {87, 101, 110}, {84, 104, 117}, {70, 114, 105}, {83, 97, 116}, {83, 117, 110}};
// int dayofweekRu[7][3] = {{207, 237, 228}, {194, 242, 240}, {209, 240, 228}, {215, 210, 226}, {207, 242, 237}, {209, 225, 242}, {194, 241, 234}};
byte bLang = 0;
int arrDay = 0;

boolean timerActive = false;
int timerTime = 1;
unsigned long timingHold;
int timerStartTime = 1;
boolean timeRun = false;
boolean timerDots = false;
boolean timerAlarmFlag = false;

enum settimemodes {SETHOUR, SETMINUTE, SETSECOND, SETYEAR, SETMONTH, SETDAY, SETHOURSIGNAL,
                    SETHOURSIGNALON, SETHOURSIGNALOFF, SETLANG};
int setMode = 0;
uint8_t setHour = 15;
uint8_t setMinute = 55;
uint8_t setSecond = 0;
int setYear = 20;
uint8_t setMon = 9;
uint8_t setDay = 6;
uint8_t maxDays = 0;
boolean setSecondFlag = false;

struct          //time signal structure
{
    boolean hourSignal = true;
    byte timeSignalStart = 6;
    byte timeSignalEnd = 21;
} timeSignal;

struct          //params struct for save
{
    int iBrightMode = 0;


} params;
