#!/usr/bin/python3
'''
@author    : shengyixu Created on 2022.8.20
 Purpose   : 绘制PosGO软件"静态"残差序列, 参考真值需要命令行输入
'''
from datetime import datetime
import sys
from math import radians, sin
from locale import atof
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import numpy as np
from mytime import gpsws2ymdhms
from com import std, rms, mean, maxabs, xyz_blh

RE = 6378137

class coord(object):
    def __init__(self):
        self.gpsw = []
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.b, self.l, self.h = 0.0, 0.0, 0.0
        self.max = []

    def setValue(self, a, b, c, type):
        self.detx, self.dety, self.detz = [], [], []
        if type == 'xyz':
            self.x, self.y, self.z = a, b, c
            self.b, self.l, self.h = xyz_blh(a, b, c)
        #    self.l = 180-self.l
        elif type == 'blh':
            self.b, self.l, self.h = a, b, c
        else:
            print('error parameter of function =setvalue=')
            exit(1)

    def getRes(self, x, y, z):
        self.detx.append(radians(x - self.b) * RE)
        self.dety.append(radians(y - self.l) *  RE / sin(radians(y)))
        self.detz.append(z - self.h)


    def statistic(self):
        self.xrms, self.yrms, self.zrms = rms(self.detx), rms(self.dety), rms(self.detz)
        self.xmean, self.ymean, self.zmean = mean(self.detx), mean(self.dety), mean(self.detz)
        self.max = [maxabs(self.detx), maxabs(self.dety), maxabs(self.detz)]

def DrawFigureStatic(_coord):
    """
    @author    : shengyixu Created on 2022.8.20
    Purpose    : 绘制散点图或折线图
    input      : 类coord的对象:_coord
    """
    # 时间格式：%Y-%m-%d %H:%M:%S，x轴显示：%H:%M:%S
    ymdhms = np.zeros((len(_coord.gpsw), 6), dtype=float)
    Time_hms = []
    for ieph in range(0, len(_coord.gpsw)):
        ymdhms[ieph][0], ymdhms[ieph][1], ymdhms[ieph][2], ymdhms[ieph][3], ymdhms[ieph][4], ymdhms[ieph][5] = gpsws2ymdhms(int(_coord.gpsw[ieph]), (_coord.gpsw[ieph] - int(_coord.gpsw[ieph]))*86400*7) 
        Time_hms.append("%04d-%02d-%02d %02d:%02d:%02d" % (ymdhms[ieph][0], ymdhms[ieph][1], ymdhms[ieph][2], ymdhms[ieph][3], ymdhms[ieph][4], ymdhms[ieph][5]))
    Time_hms = [datetime.strptime(date, "%Y-%m-%d %H:%M:%S") for date in Time_hms]

    ## draw figure
    plt.figure(dpi = 100, figsize=(10, 6.18))
    myFmt = mdates.DateFormatter('%H:%M:%S')

    plt.subplot(3,1,1)
    plt.plot_date(Time_hms[0:len(_coord.detx)], _coord.detx, fmt='b.', label = 'rms_B: ' + str(round(_coord.xrms, 3)) + 'm')
#    plt.plot(Time_hms[0:len(_coord.detx)], _coord.detx, 'blue' , label = 'rms_B: ' + str(round(_coord.xrms, 3)) + 'm')
    plt.legend(loc='upper right',fontsize=10)
    plt.ylabel('Latitude(m)')
    if max(_coord.max[0:1]) > 5:
        plt.ylim(-15, 15)
    else:
        plt.ylim(-max(_coord.max[0:1]), max(_coord.max[0:1]))
    plt.gca().xaxis.set_major_formatter(myFmt)
    plt.grid(True)

    plt.subplot(3,1,2)
    plt.plot_date(Time_hms[0:len(_coord.dety)], _coord.dety, fmt='r.', label = 'rms_L: ' + str(round(_coord.yrms, 3)) + 'm')
#    plt.plot(Time_hms[0:len(_coord.dety)], _coord.dety, 'red', label = 'rms_L: ' + str(round(_coord.yrms, 3)) + 'm')
    plt.legend(loc='upper right',fontsize=10)
    plt.ylabel('Longitude(m)')
    if max(_coord.max[0:1]) > 5:
        plt.ylim(-15, 15)
    else:
        plt.ylim(-max(_coord.max[0:1]), max(_coord.max[0:1]))
    plt.gca().xaxis.set_major_formatter(myFmt)
    plt.grid(True)

    plt.subplot(3,1,3)
    plt.plot_date(Time_hms[0:len(_coord.detz)], _coord.detz, fmt='g.', label = 'rms_H: ' + str(round(_coord.zrms, 3)) + 'm')
#    plt.plot(Time_hms[0:len(_coord.detz)], _coord.detz, 'green', label = 'rms_H: ' + str(round(_coord.zrms, 3)) + 'm')
    plt.legend(loc='upper right',fontsize=10)
    plt.ylabel('Height(m)')
    if max(_coord.max[0:1]) > 5:
        plt.ylim(-25, 25)
    else:
        plt.ylim(-max(_coord.max[0:1]), max(_coord.max[0:1]))
    plt.gca().xaxis.set_major_formatter(myFmt)
    plt.grid(True)
    plt.legend(loc='upper right',fontsize=10)

    plt.xlabel('Epoch')
    plt.savefig('residual.png', bbox_inches='tight', pad_inches=0.2)
    plt.show()

if __name__ == '__main__':
    
    InputCoor = np.zeros(3)
    if len(sys.argv) != 6:
        print('#usage   : AnalyzeStatic.py result_file CoorType [b/x] [l/y] [h/z]')
        print('CoorType : xyz or blh')
        print('[b/x][l/y][h/z] is the station coordinates required input')
        sys.exit(0)
    else:
        calFile = sys.argv[1]
        Type    = sys.argv[2]
        InputCoor[0] = atof(sys.argv[3])
        InputCoor[1] = atof(sys.argv[4])
        InputCoor[2] = atof(sys.argv[5])

    valueTrue = coord()

    valueTrue.setValue( InputCoor[0], InputCoor[1], InputCoor[2], Type)

    calValue = {}
    with open(calFile, 'r') as file:
        content = file.readlines()
        for eachLine in content:
            if eachLine[0] == "%":
                continue
            eachData = eachLine.split(",")
             # GPSweek(int) + GPSsecond => week(float)
            time = int(eachData[0]) + float(eachData[1])/86400.0/7.0
            valueTrue.gpsw.append(time)
            
            calValue.update({ time: {'b':float(eachData[2]), 'l':float(eachData[3]), 'h':float(eachData[4])} })

    for epoch, data in calValue.items():
        valueTrue.getRes(data['b'], data['l'], data['h'])

    valueTrue.statistic()

    DrawFigureStatic(valueTrue)