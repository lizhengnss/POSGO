/*--------------------------------------------------------------------------------
 * POSGO: a GNSS POSition system based on Graph Optimization
 *
 * Copyright (C) 2022 Satellite POD and Navigation Augmentation Group,
 *                    Wuhan University
 *
 *     Author : Zhen Li
 *    Contact : sdkjlizhen@foxmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * reference :
 *
 * Created by lizhen on 2022/4/21.
 *-------------------------------------------------------------------------------*/

#ifndef CTIMES_H
#define CTIMES_H

#include "Types.h"
#include "CString.h"

class CTimes {
public:
    /* add time --------------------------------------------------------------------
    * args   : STimes  t        I   STimes struct
    *          double sec       I   time to add (s)
    * return : STimes struct (t+sec)
    *-----------------------------------------------------------------------------*/
    static STimes timeadd(STimes t, double sec){
        double tt;
        t.sec += sec;
        tt = floor(t.sec);
        t.time += (int)tt;
        t.sec -= tt;
        return t;
    }

    /* time difference -------------------------------------------------------------
    * args   : STimes t1,t2    I   gtime structs
    * return : time difference (t1-t2) (s)
    *-----------------------------------------------------------------------------*/
    static double timediff(STimes t1, STimes t2){
        return difftime(t1.time, t2.time) + t1.sec - t2.sec;
    }

    /* gpstime to beidou navigation satellite system time --------------------------
    * args   : STimes t        I   time expressed in gpstime
    * return : time expressed in bdt
    * notes  : 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC, no leap seconds in BDT
    *-----------------------------------------------------------------------------*/
    static STimes gpst2bdt(STimes t){
        return timeadd(t, -14.0);
    }

    /* bdt to gpstime --------------------------------------------------------------
    * args   : STimes t        I   time expressed in bdt
    * return : time expressed in gpstime
    * notes  : see gpst2bdt()
    *-----------------------------------------------------------------------------*/
    static STimes bdt2gpst(STimes t){
        return timeadd(t, 14.0);
    }

    /* convert calendar day/time to STimes struct------------------------------
    * args   : vector ep       I   day/time {year,month,day,hour,min,sec}
    * return : STimes struct
    * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
    *-----------------------------------------------------------------------------*/
    static STimes epoch2time(const vector<double> ep){
        STimes time;
        const int doy[]={1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
        int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];
        if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) return time;

        /* leap year if year%4==0 in 1901-2099 */
        days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year%4 == 0 && mon >= 3 ? 1 : 0);
        sec = (int)floor(ep[5]);
        time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
        time.sec = ep[5] - sec;
        return time;
    }

    /* time to calendar day/time ---------------------------------------------------
    * convert STimes struct to calendar day/time
    * args   : STimes t         I   STimes struct
    * return   vector ep        O   day/time {year,month,day,hour,min,sec}
    * notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
    *-----------------------------------------------------------------------------*/
    static vector<double> time2epoch(STimes t){
        const int mday[]={ /* # of days in a month */
                31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
                31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
        };
        int days,sec,mon,day;
        vector<double>ep;

        /* leap year if year%4==0 in 1901-2099 */
        days = (int)(t.time / 86400);
        sec  = (int)(t.time - (time_t)days * 86400);
        for (day = days % 1461, mon = 0; mon < 48; mon++)
        {
            if (day >= mday[mon]) day -= mday[mon]; else break;
        }
        ep.push_back(1970 + days / 1461 * 4 + mon / 12);
        ep.push_back(mon % 12 + 1); ep.push_back(day + 1);
        ep.push_back(sec / 3600)  ; ep.push_back(sec % 3600 / 60);
        ep.push_back(sec % 60 + t.sec);
        return ep;
    }

    /* convert gpstime to utc considering leap seconds------------------------------
    * args   : STimes t        I   time expressed in gpstime
    * return : time expressed in utc
    * notes  : ignore slight time offset under 100 ns
    *-----------------------------------------------------------------------------*/
    static STimes gpst2utc(STimes t){
        STimes tu;

        for (int i = 0; gLeapsec[i][0] > 0; i++)
        {
            tu = timeadd(t, gLeapsec[i][6]);
            if (timediff(tu, epoch2time(gLeapsec[i])) >= 0.0) return tu;
        }
        return t;
    }

    /* convert utc to gpstime considering leap seconds------------------------------
    * args   : STimes t        I   time expressed in utc
    * return : time expressed in gpstime
    * notes  : ignore slight time offset under 100 ns
    *-----------------------------------------------------------------------------*/
    static STimes utc2gpst(STimes t){
        for (int i = 0; gLeapsec[i][0] > 0; i++)
        {
            if (timediff(t, epoch2time(gLeapsec[i])) >= 0.0)
            {
                return timeadd(t, -gLeapsec[i][6]);
            }

        }
        return t;
    }

    /* convert string to STimes struct----------------------------------------------
    * args   : string  buff     I   string ("... yyyy mm dd hh mm ss ...")
    * return : STimes   t       O   STimes struct
    *-----------------------------------------------------------------------------*/
    static STimes str2time(string buff){
        vector<double> ep;
        vector<string> sbuff;
        sbuff = CString::split(buff, " ");

        for (int i = 0; i < 6; i++) ep.push_back(strtod(sbuff[i].c_str(),NULL));
        if (ep[0] < 100.0) ep[0] += ep[0] < 80.0 ? 2000.0 : 1900.0;
        STimes t = epoch2time(ep);
        return t;
    }

    /* convert STimes struct to string----------------------------------------------
    * args   : STimes  t        I   STimes struct
    * return : string  time     O   string ("yyyy/mm/dd hh:mm:ss.sss")
    *-----------------------------------------------------------------------------*/
    static string time2str(STimes t){
        int n = 3;   //number of decimals
        vector<double> ep;
        std::stringstream time;

        if (1.0 - t.sec < 0.5 / pow(10.0, n)) {t.time++; t.sec = 0.0;};
        ep = time2epoch(t);
        time << std::fixed   << std::setprecision(0) << ep[0] << "/"
             << std::setw(2) << std::setfill('0')    << ep[1] << "/"
             << std::setw(2) << std::setfill('0')    << ep[2] << " "
             << std::setw(2) << std::setfill('0')    << ep[3] << ":"
             << std::setw(2) << std::setfill('0')    << ep[4] << ":"
             << std::setw(6) << std::setfill('0')    << std::setprecision(3) << ep[5];
        return time.str();
    }

    /* convert week and tow in gps time to STimes struct----------------------------
    * args   : int    week      I   week number in gps time
    *          double sec       I   time of week in gps time (s)
    * return : STimes struct
    *-----------------------------------------------------------------------------*/
    static STimes gpst2time(int week, double sec){
        STimes t = epoch2time({1980,1, 6,0,0,0});  /* gps time reference */

        if (sec < -1E9 || 1E9 < sec) sec = 0.0;
        t.time += (time_t)86400 * 7 * week + (int)sec;
        t.sec = sec - (int)sec;
        return t;
    }

    /* convert STimes struct to week and tow in gps time----------------------------
    * args   : STimes  t        I   STimes struct
    *          int    *week     IO  week number in gps time (NULL: no output)
    * return : time of week in gps time (s)
    *-----------------------------------------------------------------------------*/
    static double time2gpst(STimes t, int *week){
        STimes t0 = epoch2time({1980,1, 6,0,0,0});
        time_t sec = t.time - t0.time;
        int w = (int)(sec / (86400 * 7));
        if (week) *week = w;
        return (double)(sec - (double)w * 86400 * 7) + t.sec;
    }

    /* convert week and tow in beidou time (bdt) to STimes struct------------------
    * args   : int    week      I   week number in bdt
    *          double sec       I   time of week in bdt (s)
    * return : STimes struct
    *-----------------------------------------------------------------------------*/
    static STimes bdt2time(int week, double sec){
        STimes t = epoch2time({2006, 1, 1,0,0,0});      /* beidou time reference */
        if (sec < -1E9 || 1E9 < sec) sec = 0.0;
        t.time += (time_t)86400 * 7 * week + (int)sec;
        t.sec = sec - (int)sec;
        return t;
    }

    /* convert time to day of year--------------------------------------------------
    * args   : STimes t        I   STimes struct
    * return : day of year (days)
    *-----------------------------------------------------------------------------*/
    static double time2doy(STimes t){
        vector<double> ep;
        ep = time2epoch(t);
        ep[1] = ep[2] = 1.0;
        ep[3] = ep[4] = ep[5] = 0.0;
        return timediff(t, epoch2time(ep)) / 86400.0 + 1.0;
    }

    static double time2mjday(const STimes t){
        vector<double> ep;
        ep = time2epoch(t);
        return epoch2mjday(ep);
    }

    /* adjust time considering week handover -------------------------------------*/
    static STimes adjweek(STimes t, STimes t0){
        double tt = timediff(t,t0);
        if (tt < -302400.0) return timeadd(t,  604800.0);
        if (tt >  302400.0) return timeadd(t, -604800.0);
        return t;
    }

    /* adjust time considering week handover -------------------------------------*/
    static STimes adjday(STimes t, STimes t0){
        double tt = timediff(t, t0);
        if (tt <- 43200.0) return timeadd(t, 86400.0);
        if (tt >  43200.0) return timeadd(t,-86400.0);
        return t;
    }

private:
    static double epoch2mjday(vector<double> ep){
        int year = (int)ep[0];
        int mon = (int)ep[1];
        int day = (int)ep[2];
        double hour = ep[3] + ep[4] / 60.0 + ep[5] / 3600.0;
        if(mon <= 2)
        {
            year -= 1;
            mon  += 12;
        }
        double jd = (int)(365.25 * year) + (int)(30.6001 * (mon + 1)) + day + hour / 24.0 + 1720981.5;

        return jd2mjd(jd);
    }

    static double jd2mjd(const double jd){
        return jd - 2400000.5;
    }


public:
    static vector<vector<double>> gLeapsec;
};



#endif //CTIMES_H
