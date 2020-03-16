#include <Arduino.h>
#include <main.h>
#include <max7219.h>
#include <fonts.h>
#include <DS3231.h>
#include <Ticker.h>
#include <GyverEncoder.h>



// #include <Adafruit_Sensor.h>
// #include <Adafruit_BMP280.h>
// #include <SPI.h>
// #include <Wire.h>

void showDigit(char ch, int col, const uint8_t *data);
void showAnimClock();
void setCol(int col, byte v);
int showChar(char ch, const uint8_t *data);
void showSimpleTemp();
void flashSecond();
void flashDot();
void buttonClick();
void isr();
void resetMode();
void showSimpleHum();
void showDayMon();
void showYear();
void showDayOfWeek();
void showTimerTime();
void timerTick();
void timerHoldButton();
void timerFlashDot();
void alarmTimeEnd();
void changeModeBack();
void setTime();
void bip(int del);

DS3231 clock;
RTCDateTime dt;
Ticker flashTimer(flashSecond, 1000, 0, MILLIS);
Ticker flashDots(flashDot, 500);
Ticker rstMode(resetMode, 3000, 1, MILLIS);
Encoder enc(3, 2, 5, TYPE2);
Ticker timerAction(timerTick, 60000, 0, MILLIS); //60 sec
Ticker timerHold(timerHoldButton, 1500, 1, MILLIS);
Ticker timerFlashDots(timerFlashDot, 333);
Ticker timerAlarm(alarmTimeEnd, 10000, 0, MILLIS);     // 10 sec
Ticker timerChangeMode(changeModeBack, 600000, 0, MILLIS); // 10 min

// Adafruit_BMP280 bmp; // use I2C interface
// Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
// Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
// sensors_event_t temp_event, pressure_event;

/*
..######..########.########.##.....##.########.
.##....##.##..........##....##.....##.##.....##
.##.......##..........##....##.....##.##.....##
..######..######......##....##.....##.########.
.......##.##..........##....##.....##.##.......
.##....##.##..........##....##.....##.##.......
..######..########....##.....#######..##.......
*/
void setup()
{
    Serial.begin(9600);

    Serial.println("");
    Serial.println("==> START!");
    Serial.println("");

    clock.begin(); // Clock start

    initMAX7219();                // Initialize panel
    sendCmdAll(CMD_SHUTDOWN, 1);  // Сброс панели
    sendCmdAll(CMD_INTENSITY, 0); // Установка яркости

    enc.setTickMode(AUTO);

    pinMode(TONEPIN, OUTPUT);


    // clock.armAlarm1(false);
    // clock.clearAlarm1();
    // // clock.setAlarm1(0, 0, 0, 1, DS3231_MATCH_H_M_S, true);

    // // clock.armAlarm2(false);
    // // clock.clearAlarm2();
    // // clock.setAlarm2(0, 0, 17, DS3231_MATCH_H_M_S, true);

    flashTimer.start();
    flashDots.start();

    attachInterrupt(0, isr, CHANGE); // encoder interrrupt

    // bmp.begin(0x76);

    // if (!bmp.begin(0x76)) {
    //     Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    //     while (1) delay(10);
    // }

    // /* Default settings from datasheet. */
    // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    //                 Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
    //                 Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
    //                 Adafruit_BMP280::FILTER_X16,      /* Filtering. */
    //                 Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    // bmp_temp->printSensorDetails();

} // setup

/*
.##........#######...#######..########.
.##.......##.....##.##.....##.##.....##
.##.......##.....##.##.....##.##.....##
.##.......##.....##.##.....##.########.
.##.......##.....##.##.....##.##.......
.##.......##.....##.##.....##.##.......
.########..#######...#######..##.......
*/
void loop()
{
    flashTimer.update();     //update timer every second
    flashDots.update();      //update dots flasher
    enc.tick();              //check encoder
    rstMode.update();        //update reset mode timer
    timerAction.update();    //update timer action
    timerHold.update();      //update hold button
    timerFlashDots.update(); //update timer flash dots
    timerAlarm.update();     //update alarm timer
    timerChangeMode.update();

    switch (mode)
    {
    case CLOCK:
        showAnimClock(); //show time
        // showTimeNew();
        break;
    case TEMP:
        t1 = 25;
        t2 = 7;
        showSimpleTemp();
        break;
    case HUM:
        h1 = 3;
        h2 = 7;
        h3 = 5;
        showSimpleHum();
        break;
    case DAYMON:
        showDayMon();
        break;
    case DAYOFWEEK:
        showDayOfWeek();
        break;
    case TIMER:
        rstMode.stop();
        showTimerTime();
        break;
    case SETTIME:
        setTime();
        break;

    default:
        break;
    }

    //rotate encoder left
    if (mode != SETTIME)
    {
        if (enc.isLeft())
        {
            if (mode == TIMER)
            {
                if (enc.isFastL())
                {
                    timerTime = timerTime - 3;
                }
                else
                    timerTime--;

                if (timerTime < 1)
                {
                    timerTime = 90;
                }
            }
            else
            {
                mode--;
                if (mode < CLOCK)
                {
                    mode = END_MODES - 1;
                }
            }
        }
    }

    //rotate encodr right
    if (mode != SETTIME)
    {
        if (enc.isRight() or enc.isFastR())
        {
            if (mode == TIMER)
            {
                if (enc.isFastR())
                {
                    timerTime = timerTime + 3;
                }
                else
                    timerTime++;

                if (timerTime > 90)
                {
                    timerTime = 1;
                }
            }
            else
            {
                mode++;
                if (mode >= END_MODES)
                {
                    mode = CLOCK;
                }
            }
        }
    }

    // any turn encoder
    if (enc.isTurn())
    {
        if ((mode != CLOCK) or (mode != TIMER) or (mode != SETTIME))
        {
            rstMode.start();
            // Serial.println("rst start!");
        }

        if (mode == TIMER)
        {
            timerChangeMode.start();
        }

        if (mode == CLOCK)
        {
           bip(10);
        }
        else
        {
            bip(1);
        }


        if (mode == SETTIME)
        {
            flash = true;
            rstMode.stop();
        }

        // Serial.print("mode : ");
        // Serial.println(mode);
    }

    //double click encoder button
    if (enc.isDouble())
    {
        if (mode == TIMER)
        {
            mode = CLOCK;
            timerChangeMode.stop();
            bip(10);
        }
        else
        {
            mode = TIMER;
            timerChangeMode.start();
            rstMode.stop();
        }
        // Serial.println("timer mode");
    }

    //single click encoder button
    if (enc.isSingle())
    {
        if (mode == TIMER)
        {
            switch (timerAction.state())
            {
            case STOPPED:
                if (timerAlarmFlag)
                {
                    //for future ))
                }
                else
                {
                    timerStartTime = timerTime;
                    timerAction.start();
                    timeRun = true;
                    timerFlashDots.start();
                    timerChangeMode.stop();
                    bip(10);

                    Serial.println("timer START!");
                }
                break;
            // case PAUSED:
            //     timerAction.resume();
            //     Serial.println("timer RESUME");
            //     break;
            // case RUNNING:
            //     timerAction.pause();
            //     Serial.println("timer PAUSED");
            default:
                break;
            }
        }

        if (mode == SETTIME)
        {
            // setMode++;          //next mode

            // if (setMode > SETHOURSIGNAL)
            // {
            //     setMode = SETHOUR;
            //     mode = CLOCK;
            // }




            switch (setMode)
            {
            case SETHOUR:
                setMinute = dt.minute;
                setMode = SETMINUTE;
                break;
            case SETMINUTE:
                setMode = SETSECOND;
                break;
            case SETSECOND:
                if (setSecondFlag)
                {
                    clock.setDateTime(dt.year, dt.month, dt.day, setHour, setMinute, dt.second + 3); //add 3 sec for setup time
                }
                else
                {
                    clock.setDateTime(dt.year, dt.month, dt.day, setHour, setMinute, dt.second); //add 3 sec for setup time
                }

                setYear = dt.year;
                setMode = SETYEAR;
                break;
            case SETYEAR:
                setMon = dt.month;
                setMode = SETMONTH;
                break;
            case SETMONTH:
                setDay = dt.day;
                maxDays = clock.daysInMon(setYear, setMon);
                setMode = SETDAY;
                break;
            case SETDAY:
                clock.setDateTime(setYear, setMon, setDay, setHour, setMinute, dt.second);
                setMode = SETHOURSIGNAL;
                break;
            case SETHOURSIGNAL:
                if (timeSignal.hourSignal)
                {
                    setMode = SETHOURSIGNALON;

                } else {
                    setMode = SETHOUR;
                    mode = CLOCK;
                }
            break;

            default:
                break;
            }
        }

        if (timerAlarmFlag)
        {
            alarmTimeEnd();
        }

        if (mode == CLOCK)
        {

            params.iBrightMode++;

            if (params.iBrightMode == 3)
            {
                params.iBrightMode = 0;
            }

            switch (params.iBrightMode)
            {
            case 0:
                sendCmdAll(CMD_INTENSITY, 0);
                break;
            case 1:
                sendCmdAll(CMD_INTENSITY, 5);
                break;
            case 2:
                sendCmdAll(CMD_INTENSITY, 15);
                break;

            default:
                break;
            }
        }



//   bmp_temp->getEvent(&temp_event);
//   bmp_pressure->getEvent(&pressure_event);

//   Serial.print(F("Temperature = "));
//   Serial.print(bmp.readTemperature());
//   Serial.println(" *C");

//   Serial.print(F("Pressure = "));
//   Serial.print(bmp.readPressure());
//   Serial.println(" hPa");


    }

    // hold encoder button
    if (enc.isHold())
    {
        if ((mode == TIMER) or (mode == CLOCK))
        {
            if (timerHold.state() == STOPPED)
            {
                timerHold.start();
            }
        }
    }
    else
    {
        timerHold.stop();
    }

    //flag every second
    if(dt.second != lastSecond) {
        lastSecond = dt.second;
        secFr = 0;
    } else {
        secFr++;
    }

    //check hour signal
    if (timeSignal.hourSignal and (dt.minute == 0) and (dt.second == 0) and (secFr == 0) and
       (dt.hour >= timeSignal.timeSignalStart) and (dt.hour <= timeSignal.timeSignalEnd))
    {
        bip(300);
    }

} //loop

/*
.########.##.....##.##....##..######..########.####..#######..##....##..######.
.##.......##.....##.###...##.##....##....##.....##..##.....##.###...##.##....##
.##.......##.....##.####..##.##..........##.....##..##.....##.####..##.##......
.######...##.....##.##.##.##.##..........##.....##..##.....##.##.##.##..######.
.##.......##.....##.##..####.##..........##.....##..##.....##.##..####.......##
.##.......##.....##.##...###.##....##....##.....##..##.....##.##...###.##....##
.##........#######..##....##..######.....##....####..#######..##....##..######.
*/
//=== set time mode ==================================================================================
void setTime()
{
    clr();
    showDigit(58 - 32, digPosSet[1], fontUA_RU_PL_DE);  //:

    switch (setMode)
    {
    case SETHOUR:
        showDigit(104 - 32, digPosSet[0], fontUA_RU_PL_DE);     //h
        // showDigit(61 - 32, digPosSet[1], fontUA_RU_PL_DE);
        showDigit(flash ? (setHour / 10) : 24, digPosSet[2], dig5x8rn);
        showDigit(flash ? (setHour % 10) : 24, digPosSet[3], dig5x8rn);

        if (enc.isRight())
        {
            setHour++;
            if (setHour > 23)
            {
                setHour = 0;
            }
        }
        if (enc.isLeft())
        {
            setHour--;
            if (setHour < 0)
            {
                setHour = 23;
            }
        }
        break;

    case SETMINUTE:
        showDigit(109 - 32, digPosSet[0], fontUA_RU_PL_DE);     //m
        // showDigit(61 - 32, digPosSet[1], fontUA_RU_PL_DE);
        showDigit(flash ? (setMinute / 10) : 24, digPosSet[2], dig5x8rn);
        showDigit(flash ? (setMinute % 10) : 24, digPosSet[3], dig5x8rn);

        if (enc.isRight())
        {
            setMinute++;
            if (setMinute > 59)
            {
                setMinute = 0;
            }
        }
        if (enc.isLeft())
        {
            setMinute--;
            if (setMinute < 0)
            {
                setMinute = 59;
            }
        }
        break;

    case SETSECOND:
        setSecond = dt.second;
        showDigit(115 - 32, digPosSet[0], fontUA_RU_PL_DE);     //s
        // showDigit(61 - 32, digPosSet[1], fontUA_RU_PL_DE);
        showDigit(flash ? (setSecond / 10) : 24, digPosSet[2], dig5x8rn);
        showDigit(flash ? (setSecond % 10) : 24, digPosSet[3], dig5x8rn);

        if (enc.isRight() or enc.isLeft())
        {
            clock.setDateTime(dt.year, dt.month, dt.day, dt.hour, dt.minute, 0); //add 3 sec for setup time
            setSecondFlag = true;
        }

        break;

    case SETYEAR:
        showDigit(121 - 32, digPosSet[0], fontUA_RU_PL_DE);     //y
        // showDigit(61 - 32, digPosSet[1], fontUA_RU_PL_DE);
        showDigit(flash ? (setYear %1000 % 100 / 10) : 24, digPosSet[2], dig5x8rn);
        showDigit(flash ? (setYear %1000 % 100 % 10) : 24, digPosSet[3], dig5x8rn);

        if (enc.isRight())
        {
            setYear++;
            if (setYear > 2099)
            {
                setYear = 2020;
            }
        }
        if (enc.isLeft())
        {
            setYear--;
            if (setYear < 2020)
            {
                setYear = 2099;
            }
        }
        break;

    case SETMONTH:
        showDigit(109 - 32, digPosSet[0], fontUA_RU_PL_DE);     //m
        // showDigit(61 - 32, digPosSet[1], fontUA_RU_PL_DE);
        showDigit(flash ? (setMon / 10) : 24, digPosSet[2], dig5x8rn);
        showDigit(flash ? (setMon % 10) : 24, digPosSet[3], dig5x8rn);

        if (enc.isRight())
        {
            setMon++;
            if (setMon > 12)
            {
                setMon = 1;
            }
        }
        if (enc.isLeft())
        {
            setMon--;
            if (setMon < 1)
            {
                setMon = 12;
            }
        }
        break;

    case SETDAY:
        showDigit(100 - 32, digPosSet[0], fontUA_RU_PL_DE);     //d
        // showDigit(61 - 32, digPosSet[1], fontUA_RU_PL_DE);
        showDigit(flash ? (setDay / 10) : 24, digPosSet[2], dig5x8rn);
        showDigit(flash ? (setDay % 10) : 24, digPosSet[3], dig5x8rn);

        if (enc.isRight())
        {
            setDay++;
            if (setDay > maxDays)
            {
                setDay = 1;
            }
        }
        if (enc.isLeft())
        {
            setDay--;
            if (setDay < 1)
            {
                setDay = maxDays;
            }
        }
        break;

    case SETHOURSIGNAL:
        showDigit(72 - 32, 0, fontUA_RU_PL_DE);     //H
        showDigit(115 - 32, 6, fontUA_RU_PL_DE);     //s
        showDigit(58 - 32, 15, fontUA_RU_PL_DE);  //:
        showDigit(111 - 32, 18, fontUA_RU_PL_DE); //o


        if (timeSignal.hourSignal)
        {
            showDigit(110 - 32, 24, fontUA_RU_PL_DE); //n
        } else
        {
            showDigit(102 - 32, 24, fontUA_RU_PL_DE); //f
            showDigit(102 - 32, 28, fontUA_RU_PL_DE); //f
        }

        if (enc.isRight() or enc.isLeft())
        {
            timeSignal.hourSignal = not timeSignal.hourSignal;
        }
        break;

    case SETHOURSIGNALON:
        clr();
        showDigit(83 - 32, 0, fontUA_RU_PL_DE);   //S
        showDigit(116 - 32, 5, fontUA_RU_PL_DE);    //t
        showDigit(114 - 32, 10, fontUA_RU_PL_DE);   //r
        showDigit(58 - 32, 15, fontUA_RU_PL_DE);  //:






        break;

    default:
        break;
    }
    refreshAll();
}

//=== change mode back to clock mode ==================================================================================
void changeModeBack()
{
    mode = CLOCK;
    bip(30);
    Serial.println("back mode");
    timerChangeMode.stop();
}

//=== timer end alarm ==================================================================================
void alarmTimeEnd()
{
    timerAlarmFlag = false;
    timerTime = timerStartTime;
    timerAlarm.stop();
    timerChangeMode.start();
    digitalWrite(TONEPIN, LOW);
    Serial.println("alarm end");
}

//=== timer flash dots ==================================================================================
void timerFlashDot()
{
    timerDots = !timerDots;
}

//=== encoder button hold ==================================================================================
void timerHoldButton()
{
    if (mode == TIMER)
    {
        timerAction.stop();
        timerTime = timerStartTime;
        timerFlashDots.stop();
        timerChangeMode.start();
        bip(30);
        Serial.println("timer STOP");
    }

    if (mode == CLOCK)
    {
        setHour = dt.hour;
        mode = SETTIME;
        rstMode.stop();
    }

    timerHold.stop();
}

//=== Flashing dots ==================================================================================
void flashDot()
{
    flash = !flash; //change flash state
}

//=== Flash every second ==================================================================================
void flashSecond()
{
    dt = clock.getDateTime(); //time update
}

//=== Show time ==================================================================================
void showAnimClock()
{
    if ((millis() % 25) == 0)
    {

        int digHt = 16;
        int num = 4;
        int i;
        if (del == 0)
        {
            del = digHt;
            for (i = 0; i < num; i++)
                digold[i] = dig[i];
            dig[0] = dt.hour / 10;
            dig[1] = dt.hour % 10;
            dig[2] = dt.minute / 10;
            dig[3] = dt.minute % 10;
            for (i = 0; i < num; i++)
                digtrans[i] = (dig[i] == digold[i]) ? 0 : digHt;
        }
        else
            del--;
        clr();
        for (i = 0; i < num; i++)
        {
            if (digtrans[i] == 0)
            {
                dy = 0;
                showDigit(dig[i], digPos[i], dig5x8rn);
            }
            else
            {
                dy = digHt - digtrans[i];
                showDigit(digold[i], digPos[i], dig5x8rn);
                dy = -digtrans[i];
                showDigit(dig[i], digPos[i], dig5x8rn);
                digtrans[i]--;
            }
        }
        dy = 0;

        if (timerFlashDots.state() == RUNNING)
        {
            int flash = millis() % 1000;

            setCol(digPos[4], flash < 500 ? 0x24 : 0x42);
            setCol(digPos[5], flash > 500 ? 0x42 : 0x24);

            // if(flash >= 0 && flash < 500) {
            //     // setCol(digPos[4], true ? 0x24 : 0x20);
            //     // setCol(digPos[5], true ? 0x42 : 0x40);
            // }
            // if(flash >= 501 && flash < 1000) {
            //     // setCol(digPos[4], false ? 0x42 : 0x40);
            //     setCol(digPos[5], false ? 0x24 : 0x20);
            // }
        }
        else
        {
            setCol(digPos[4], flash ? 0x00 : 0x66);
            setCol(digPos[5], flash ? 0x00 : 0x66);
        }
        refreshAll();
    }
}


//=== Output only digits ==================================================================================
void showDigit(char ch, int col, const uint8_t *data)
{
    if ((dy < -8) | (dy > 8))
        return;
    int len = pgm_read_byte(data);
    int w = pgm_read_byte(data + 1 + ch * len);
    col += dx;
    for (int i = 0; i < w; i++)
    {
        if (col + i >= 0 && col + i < 8 * NUM_MAX)
        {
            byte v = pgm_read_byte(data + 1 + ch * len + 1 + i);
            if (!dy)
                scr[col + i] = v;
            else
                scr[col + i] |= dy > 0 ? v >> dy : v << -dy;
        }
    }
}

//=== Show simbol in row ===================================================================================
void setCol(int col, byte v)
{
    if ((dy < -8) | (dy > 8))
        return;
    col += dx;
    if (col >= 0 && col < 8 * NUM_MAX)
    {
        if (!dy)
            scr[col] = v;
        else
            scr[col] |= dy > 0 ? v >> dy : v << -dy;
    }
}

//=== Encoder interrupt ===================================================================================
void isr()
{
    enc.tick();
}

//=== Вывод на экран температуры в доме =======================================
void showSimpleTemp()
{
    dx = dy = 0;
    clr();
    showDigit((t1 < 0 ? 14 : 13), 0, dig5x8rn); // друкуємо D+ альбо D-

    if (t1 <= -10.0 || t1 >= 10)
        showDigit((t1 < 0 ? (t1 * -1) / 10 : t1 / 10), 4, dig5x8rn);

    showDigit((t1 < 0 ? (t1 * -1) % 10 : t1 % 10), 10, dig5x8rn);
    showDigit(12, 16, dig5x8rn);
    showDigit(t2, 18, dig5x8rn);
    showDigit(10, 24, dig5x8rn);
    showDigit(11, 27, dig5x8rn);
    refreshAll();
}

//=== Reset mode after turn encoder =======================================
void resetMode()
{
    Serial.println("rest mode");
    mode = CLOCK;
}

//=== Show humidity ========================================
void showSimpleHum()
{
    dx = dy = 0;
    clr();
    clr();
    clr();
    showDigit(17, 0, dig5x8rn); // друкуємо знак вологості
    if (h1 >= 0)
        showDigit(h1, 6, dig5x8rn);
    showDigit(h2, 12, dig5x8rn);
    showDigit(12, 18, dig5x8rn);
    showDigit(h3, 20, dig5x8rn);
    showDigit(18, 26, dig5x8rn);
    refreshAll();
}

//=== Show day month ==================
void showDayMon()
{
    int d1, d2, d3, d4 = 0;
    int y3, y4 = 0;
    clr();
    d1 = dt.day / 10;
    d2 = dt.day % 10;
    d3 = dt.month / 10;
    d4 = dt.month % 10;
    y3 = dt.year % 1000 % 100 / 10;
    y4 = dt.year % 1000 % 100 % 10;

    showDigit(d1, 0, dig4x8);
    showDigit(d2, 5, dig4x8);
    showDigit(d3, 11, dig4x8);
    showDigit(d4, 16, dig4x8);
    showDigit(y3, 22, dig4x8);
    showDigit(y4, 27, dig4x8);
    refreshAll();
}

//=== show day of week ==================
void showDayOfWeek()
{
    int day = dt.dayOfWeek - 1;


    clr();
    for (int i = 0; i < 3; i++)
    {
        arrDay = dayofweek[day][i];
        arrDay -= 32;
        showDigit(arrDay, digPosDay[i], fontUA_RU_PL_DE);

        // Serial.println( arrDay);

    }
    refreshAll();
}

//=== show time timer ==================
void showTimerTime()
{
    int tt1, tt2 = 0;
    clr();
    tt1 = timerTime / 10;
    tt2 = timerTime % 10;
    showDigit(23, 7, dig5x8rn);

    if (timerFlashDots.state() == RUNNING)
    {
        showDigit(timerDots ? 26 : 0, 13, fontUA_RU_PL_DE);
    }
    else
    {
        showDigit(26, 13, fontUA_RU_PL_DE);
    }
    showDigit(tt1, 16, dig5x8rn);
    showDigit(tt2, 23, dig5x8rn);

    if (timerAlarmFlag)
    {
        if (flash)
        {
            showDigit(23, 7, dig5x8rn);
            showDigit(tt1, 16, dig5x8rn);
            showDigit(tt2, 23, dig5x8rn);
            digitalWrite(TONEPIN, HIGH);
        }
        else
        {
            digitalWrite(TONEPIN, LOW);
            showDigit(24, 16, dig5x8rn);
            showDigit(24, 23, dig5x8rn);
            showDigit(22, 7, dig5x8rn);
        }
    }
    refreshAll();
}

//=== Timer tick ==================
void timerTick()
{
    timerTime--;
    if (timerTime == 0)
    {
        if (mode == not TIMER)
        {
            mode = TIMER;
        }

        timerAction.stop();
        timerFlashDots.stop();
        timerAlarm.start();

        timerAlarmFlag = true;

        Serial.println("timer END!");
    }
    Serial.println("timer TICK!");
}

void bip(int del)
{
    digitalWrite(TONEPIN, HIGH);
    delay(del);
    digitalWrite(TONEPIN, LOW);
    Serial.println("BIP");

// sa
// digitalWrite(TONEPIN, HIGH);
// if (millis() - currentTime > del) // Если время контроллера millis, больше переменной на 1000, то запускаем условие if
// {
//     currentTime = millis();        // Приравниваем переменную текущего времени к времени контроллера, чтобы через 1000мс опять сработал наш цикл.
//     digitalWrite(TONEPIN, LOW);
// }



}

/*
.########.##....##.########.
.##.......###...##.##.....##
.##.......####..##.##.....##
.######...##.##.##.##.....##
.##.......##..####.##.....##
.##.......##...###.##.....##
.########.##....##.########.
*/
