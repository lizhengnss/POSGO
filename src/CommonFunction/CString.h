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
 * Created by lizhen on 2022/4/24.
 *-------------------------------------------------------------------------------*/

#ifndef CSTRING_H
#define CSTRING_H


#include "Types.h"

class CString {
public:
    /* split tempdata by sep */
    static vector<string> split(string buff, const string& sep) {
        vector<string> tokens;
        tokens.clear();
        size_t lastPos = buff.find_first_not_of(sep, 0);
        size_t pos = buff.find(sep, lastPos);

        while (lastPos != string::npos)
        {
            tokens.emplace_back(buff.substr(lastPos, pos - lastPos));
            lastPos = buff.find_first_not_of(sep, pos);
            pos = buff.find(sep, lastPos);
        }
        return tokens;
    }

    /* conver string to number -----------------------------------------------
    * args   : string  s        I   string need to convert
    *          int    pos       I   start position of s need to convert
    *          int    num       I   number need to convert after 'pos'
    * return : STimes struct (t+sec)
    *-------------------------------------------------------------------------*/
    static double str2num(string s, int pos, int num){
        s = s.substr(pos, num);
        if (s.find("D") != string::npos)
        {
            s.replace(s.find("D"),1,"e");
        }
        return strtod(s.c_str(),nullptr);
    }

    static string trim(string &s){
        if( !s.empty() )
        {
            s.erase(0,s.find_first_not_of(" "));
            s.erase(s.find_last_not_of(" ") + 1);
        }
        return s;
    }

/*--------------------------------------GNSS-------------------------------------------------*/

    /* convert satellite prn to satellite id----------------------------------------
    * args   : string   prn       I   satellite id (Gnn,Rnn,Enn,Jnn,Cnn,Inn or Snn)
    * return : satellite id (0: error)
    *------------------------------------------------------------------------------*/
    static int satprn2id(string prn){
        char sys = prn[0];
        int id = str2num(prn, 1, 2);
        switch (sys)
        {
            case 'G': break;
            case 'C': id += NSATGPS; break;
            case 'R': id += NSATGPS + NSATBDS; break;
            case 'E': id += NSATGPS + NSATBDS + NSATGLO; break;
            default: return 0;
        }
        return id;
    }

    /* convert satellite id to satellite prn----------------------------------------
    * args   : string   id       I   satellite id
    * return : satellite prn
    *------------------------------------------------------------------------------*/
    static string satid2prn(int id){
        int sys = satsys(id);
        string prn;

        switch (sys)
        {
            case SYS_GPS : prn = "G";                                break;
            case SYS_BDS : prn = "C"; id -= NSATGPS;                 break;
            case SYS_GLO : prn = "R"; id -= NSATGPS+NSATBDS;         break;
            case SYS_GAL : prn = "E"; id -= NSATGPS+NSATBDS+NSATGLO; break;
        }

        if (id < 10) prn += "0";
        prn += std::to_string(id);
        return prn;
    }

    /* convert satellite number to satellite system---------------------------------
    * args   : string    prn       I   satellite prn
    * return : satellite system (SYS_GPS,SYS_GLO,...)
    *-----------------------------------------------------------------------------*/
    static int satsys(string prn){
        char sys = prn[0];
        switch (sys)
        {
            case 'G': return SYS_GPS;
            case 'C': return SYS_BDS;
            case 'R': return SYS_GLO;
            case 'E': return SYS_GAL;
            default : return 0;
        }
    }

    /* convert satellite id to satellite system------------------------------------
    * args   : int       sat       I   satellite number (1-MAXSAT)
    * return : satellite system (SYS_GPS,SYS_GLO,...)
    *-----------------------------------------------------------------------------*/
    static int satsys(int sat){
        int sys = SYS_NONE;
        if (sat <= 0 || MAXSAT < sat) sat = 0;
        else if (sat <= NSATGPS)
        {
            sys = SYS_GPS; sat += MINPRNGPS - 1;
        }
        else if ((sat -= NSATGPS) <= NSATBDS)
        {
            sys = SYS_BDS; sat += MINPRNBDS - 1;
        }
        else if ((sat -= NSATBDS) <= NSATGLO)
        {
            sys = SYS_GLO; sat += MINPRNGLO - 1;
        }
        else if ((sat-=NSATGLO) <= NSATGAL)
        {
            sys = SYS_GAL; sat += MINPRNGAL - 1;
        }
        else sat = 0;
        return sys;
    }

    /* bd2 or bd3 */
    static int checkbd23(string prn) {
        if(str2num(prn, 1, 2) > 18) return SYS_BD3;
        else                        return SYS_BDS;
    }

    /* test navi system (m=0:gps,1:bd2,2:glo,3:gal,4:bd3) */
    static bool testsystem(int sys, int m){
        switch (sys)
        {
            case SYS_GPS: return m==0;
            case SYS_BDS: return m==1;
            case SYS_GLO: return m==2;
            case SYS_GAL: return m==3;
            case SYS_BD3: return m==4;
            default     : return false;
        }
    }

};


#endif //CSTRING_H
