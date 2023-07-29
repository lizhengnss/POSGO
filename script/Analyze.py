
#!/usr/bin/python3
'''
@author    : shengyixu Created on 2022.8.20
Purpose    : 绘制PosGO软件"动态"残差序列, 参考值来自IE
'''
import sys
from mytime import gpsws2ymdhms
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from math import radians, sin
from com import std, rms, mean, maxabs
from readfile import ReadMyResult, ReadIERefResult, ReadGINSResult

class cStat(object):
    """
    @author    : shengyixu Created on 2022.8.20
    Purpose    : 残差序列以及数值特征
    """
    def __init__(self):
        self.gpsw = []
        self.dx, self.dy, self.dz = [], [], []
        self.rms = []
        self.std = []
        self.max = []
        self.mean = []
        self.fixratio = 0

RE = 6378137

def CalDifference(_cal, _ref):
    """
    @author    : shengyixu Created on 2022.8.20
    Purpose    : 获取PosGO计算值与IE参考值的残差序列
    input      : PosGO计算文件:_cal 
                    IE参考文件:_ref
    """
    result = {}
    for time, cal in _cal.items():

        if time in _ref.keys():
            result.update(
                {time: {'b': radians(cal["b"] - _ref[time]["b"]) * RE,
                        'l': radians(cal["l"] - _ref[time]["l"]) * RE / sin(radians(cal["b"])),
                        'h': cal["h"] - _ref[time]["h"],
                        'stat': cal['stat'],
                        }})
    return result


def StatisticResult(_det):
    """
    @author    : shengyixu Created on 2022.8.20
    Purpose    : 统计残差序列的数值特征
    input      : 残差序列:_det
    """
    all, fix = 0, 0
    _stat = cStat()
    for time, det in _det.items():
        all += 1
        _stat.gpsw.append(time)

        if abs(det["b"]) < 1e10:
            _stat.dx.append(det["b"])
        if abs(det["l"]) < 1e10:
            _stat.dy.append(det["l"])
        if abs(det["h"]) < 1e10:
            _stat.dz.append(det["h"])
        if det["stat"] == 1 or det["stat"] == 3:
            fix += 1

    _stat.std  = [std(_stat.dx)*100,    std(_stat.dy)*100,    std(_stat.dz)*100]
    _stat.rms  = [rms(_stat.dx)*100,    rms(_stat.dy)*100,    rms(_stat.dz)*100]
    _stat.mean = [mean(_stat.dx)*100,  mean(_stat.dy)*100,   mean(_stat.dz)*100]

    _stat.dx.clear()
    _stat.dy.clear()
    _stat.dz.clear()

    # 重新保存残差序列
    for time, det in _det.items():
        _stat.dx.append(det["b"])
        _stat.dy.append(det["l"])
        _stat.dz.append(det["h"])

    _stat.max  = [maxabs(_stat.dx)*100, maxabs(_stat.dy)*100, maxabs(_stat.dz)*100]

    print("Position Error (BLH cm):")
    print("RMS   %9.3f    %9.3f    %9.3f" % (_stat.rms[0], _stat.rms[1], _stat.rms[2]))
    print("STD   %9.3f    %9.3f    %9.3f" % (_stat.std[0], _stat.std[1], _stat.std[2]))
    print("MAX   %9.3f    %9.3f    %9.3f" % (_stat.max[0], _stat.max[1], _stat.max[2]))
    print("MEAN  %9.3f    %9.3f    %9.3f" % (_stat.mean[0], _stat.mean[1], _stat.mean[2]))

    print("All epoch:%7d, Fix epoch:%7d, Percentage:%7.3f" % (all, fix, fix / all * 100))

    _stat.fixratio = fix / all * 100

    return _stat


def ExportDifference(_file, _det):
    """
    @author    : shengyixu Created on 2022.8.20
    Purpose    : 将残差序列输出到文件中
    input      : 残差序列:_det
    output     : 残差文件:_file
    """
    with open(_file, 'w') as file:
        for time, each in _det.items():
            file.write('%10.3f, %9.3f, %9.3f, %9.3f, %2d \n' %
                       (time, each["b"], each["l"], each["h"], each["stat"])
                      )


def DrawFigure(_stat):
    """
    @author    : shengyixu Created on 2022.8.20
    Purpose    : 绘制散点图或折线图
    input      : 类cStat的对象:_stat
    """
    # 时间格式：%Y-%m-%d %H:%M:%S，x轴显示：%H:%M:%S
    ymdhms = np.zeros((len(_stat.gpsw), 6), dtype=float)
    Time_hms = []
    for ieph in range(0, len(_stat.gpsw)):
        ymdhms[ieph][0], ymdhms[ieph][1], ymdhms[ieph][2], ymdhms[ieph][3], ymdhms[ieph][4], ymdhms[ieph][5] = gpsws2ymdhms(int(_stat.gpsw[ieph]), (_stat.gpsw[ieph] - int(_stat.gpsw[ieph]))*86400*7) 
        Time_hms.append("%04d-%02d-%02d %02d:%02d:%02d" % (ymdhms[ieph][0], ymdhms[ieph][1], ymdhms[ieph][2], ymdhms[ieph][3], ymdhms[ieph][4], ymdhms[ieph][5]))
    Time_hms = [datetime.strptime(date, "%Y-%m-%d %H:%M:%S") for date in Time_hms]

    ## draw position
    plt.figure(dpi = 100, figsize=(10, 6.18))
    myFmt = mdates.DateFormatter('%H:%M:%S')
    plt.subplot(3,1,1)
    plt.plot_date(Time_hms[0:len(_stat.dx)], _stat.dx, fmt='b.' , label = 'rms_B: ' + str(round(_stat.rms[0], 3)) + 'cm')
#   plt.plot(Time_hms[0:len(_stat.dx)], _stat.dx, 'blue', label = 'rms_B: ' + str(round(_stat.rms[0], 3)) + 'cm')
    plt.legend(loc='upper right',fontsize=10)
    plt.ylabel('Latitude(m)')
    if max(_stat.max[0:2])/100 > 5:
        plt.ylim(-5, 5)
    else:
        plt.ylim(-max(_stat.max[0:1])/100,max(_stat.max[0:1])/100)
    plt.gca().xaxis.set_major_formatter(myFmt)
    plt.grid(True)

    plt.subplot(3,1,2)
    plt.plot_date(Time_hms[0:len(_stat.dy)], _stat.dy, fmt='r.' , label = 'rms_L: ' + str(round(_stat.rms[1], 3)) + 'cm')
#   plt.plot(Time_hms[0:len(_stat.dy)], _stat.dy, 'red', label = 'rms_L: ' + str(round(_stat.rms[1], 3)) + 'cm')
    plt.legend(loc='upper right',fontsize=10)
    plt.ylabel('Longitude(m)')
    plt.ylim(-max(_stat.max[0:1])/100,max(_stat.max[0:1])/100)
    if max(_stat.max[0:2])/100 > 5:
        plt.ylim(-5, 5)
    plt.gca().xaxis.set_major_formatter(myFmt)
    plt.grid(True)

    plt.subplot(3,1,3)
    plt.plot_date(Time_hms[0:len(_stat.dz)], _stat.dz, fmt='g.' , label = 'rms_H: ' + str(round(_stat.rms[2], 3)) + 'cm')
#   plt.plot(Time_hms[0:len(_stat.dz)], _stat.dz, 'green', label = 'rms_H: ' + str(round(_stat.rms[2], 3)) + 'cm')
    plt.legend(loc='upper right',fontsize=10)
    plt.ylabel('Height(m)')
    plt.ylim(-max(_stat.max[0:1])/100,max(_stat.max[0:1])/100)
    if max(_stat.max[0:2])/100 > 5:
        plt.ylim(-5, 5)
    plt.gca().xaxis.set_major_formatter(myFmt)
    plt.grid(True)

    plt.xlabel('Epoch')
    plt.savefig('residual.png', bbox_inches='tight', pad_inches=0.2)
    plt.show()


if __name__ == '__main__':

    if len(sys.argv) != 3 :
        print('#usage  :  Analyze.py result_file ref_file')
        print('#example:  Analyze.py my.txt ref.txt')
        sys.exit(0)

    if len(sys.argv) == 3 :
        calFile = sys.argv[1]
        refFile = sys.argv[2]

    detFile = "det.txt"

    calValue = ReadMyResult(calFile)
    refValue = ReadIERefResult(refFile)
#    refValue = ReadGINSResult(refFile)

    detValue = CalDifference(calValue, refValue)
    Stat = StatisticResult(detValue)

    ExportDifference(detFile, detValue)
    DrawFigure(Stat)