#!/usr/bin/env python3
# coding=utf-8
"""
@author: lizhen Created on 2020.11.30 09:37:25
         shengyixu on 2022.8.20
"""
import sys
import datetime

def gpswd2ydoy(gweek, gday):
    """
    @author: lizhen Created on 2019.8.15
    Purpose: Convert GPS week and week day to year and doy
    """
    wday = gweek * 7 + gday
    delta = datetime.timedelta(days=wday)
    time = datetime.datetime(1980, 1, 6) + delta
    year = time.year
    doy = time.strftime("%j")
    return year, doy


def gpsws2ymdhms(gweek, gsec):
    """
    @author: lizhen Created on 2019.8.15
    Purpose: Convert GPS week and week second to year, month, day, hour, minute and second
    """
    wday = gweek * 7
    delta = datetime.timedelta(days=wday, seconds=gsec)
    time = datetime.datetime(1980, 1, 6) + delta
    year = time.year
    month = time.month
    day = time.day
    hour = time.hour
    minute = time.minute
    second = time.second
    return year, month, day, hour, minute, second


def ymdhms2gpsws(year, month, day, hour, minute, second):
    """
    @author: lizhen Created on 2021.6.02
    Purpose: Convert year, month, day, hour, minute and second to GPS week and week second
    """
    time = datetime.datetime(year=year, month=month, day=day, hour=hour, minute=minute, second=second)
    stime = datetime.datetime(1980, 1, 6)
    delta = time - stime
    gweek = int(delta.days / 7)
    gday = delta.days % 7
    gsec = gday * 24 * 60 * 60 + delta.seconds
    return gweek, gsec


def ymd2yeardoy(year, month, day):
    """
    @author: lizhen Created on 2021.6.07
    Purpose: Convert year, month, day to year and doy
    """
    time = datetime.datetime(year=year, month=month, day=day)
    stime = datetime.datetime(year=year, month=1, day=1)
    dt = time - stime
    return year, dt.days + 1


def yeardoy2ymd(year, doy):
    """
    @author: lizhen Created on 2021.6.07
    Purpose: Convert year and doy to year, month, day
    """
    stime = datetime.datetime(year=year, month=1, day=1)
    dt = datetime.timedelta(days=doy - 1)
    time = stime + dt
    return time.year, time.month, time.day


def gps(s):
    stime = datetime.datetime(1980, 1, 6)
    delta1 = datetime.timedelta(seconds = s)
    delta2 = datetime.timedelta(seconds=315964782)

    time = stime + delta1 -delta2
    print(time.year,time.month,time.day,time.hour,time.minute,time.second)


def gps2():
    stime = datetime.datetime(1980, 1, 6)
    etime = datetime.datetime(2021, 9, 9)
    dt = etime - stime
    print(dt.seconds + dt.days * 60*60*24)


def ConvTime(list, _convxy):
    """
    @author: shengyixu Created on 2022.8.20
    Purpose: Calculate the convergence time after a given convergence threshold(_convxy)
    """
    for m in range(0, len(list)-10):
        k = 0
        if abs(list[m]) <= _convxy:
            for j in range(1,10):
                if abs(list[m+j]) <= _convxy:
                    k = k+1
        if k == 9:
            return m 


def sow2hms(gsec):
    """
    @author: shengyixu Created on 2022.8.20
    Purpose: Convert GPS week second to hour, minnute and second
    Purpose: Convert GPS week second to hour, minnute and second
    """
    sod = gsec%86400.0
    hour = (int)(sod/3600.0)
    minute = (int)((sod - hour*3600)/60.0)
    second = sod - hour*3600 - minute*60 
    return  hour, minute, second


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('\33[1;31m**Usage 1. gpsws2ymdhms:  ws2hms gpsweek gpssec\33[0m')
        print('\33[1;31m**      2. ymhms2gpsws :  hms2ws year month day hour minute second\33[0m')
        print('\33[1;31m**      3. ymd2yeardoy :  ymd2yd year month day \33[0m')
        print('\33[1;31m**      4. yeardoy2ymd :  yd2ymd year doy \33[0m')
        print('\33[1;31m**      5. wdoy2yeardoy:  wd2yd gweek gday\33[0m')
        sys.exit(1)
    else:
        mode = sys.argv[1]
        if (mode == 'ws2hms'):
            gweek = float(sys.argv[2])
            gsec = float(sys.argv[3])
            year, month, day, hour, minute, second = gpsws2ymdhms(gweek, gsec)
            print(year, month, day, hour, minute, second)
        elif (mode == 'hms2ws'):
            year = int(sys.argv[2])
            month = int(sys.argv[3])
            day = int(sys.argv[4])
            hour = int(sys.argv[5])
            minute = int(sys.argv[6])
            second = int(sys.argv[7])
            gweek, gsecond = ymdhms2gpsws(year, month, day, hour, minute, second)
            print(gweek, gsecond)
        elif (mode == 'ymd2yd'):
            year = int(sys.argv[2])
            month = int(sys.argv[3])
            day = int(sys.argv[4])
            year, doy = ymd2yeardoy(year, month, day)
            print(year, doy)
        elif (mode == 'yd2ymd'):
            year = int(sys.argv[2])
            doy = int(sys.argv[3])
            year, month, day = yeardoy2ymd(year, doy)
            print(year, month, day)
        elif (mode == 'wd2yd'):
            gweek = int(sys.argv[2])
            gday = int(sys.argv[3])
            year, doy = gpswd2ydoy(gweek, gday)
            print(year, doy)
        elif (mode == 'zh'):
            second = int(sys.argv[2])
            gps(second)