#!/usr/bin/python
import math

def rms(list):
	"""
    @author   : Lizhen
    Version   : 1.0 Created  2019.08.06
    Purpose   : 计算列表的均方根误差
    Input     : list     ——  待计算均方根的列表
    Output    : cal      ——  列表的均方根
	"""
	from math import sqrt
	cal = 0.0
	for each in list:
		cal += each * each
	cal = sqrt(cal / len(list))
	return cal

def std(list):
	"""
    @author   : Lizhen
    Version   : 1.0 Created  2019.08.10
    Purpose   : 计算列表的标准差
    Input     : list     ——  待计算标准差的列表
    Output    : cal      ——  列表的标准差
	"""
	from math import sqrt
	cal = 0.0
	list_mean = mean(list)
	for each in list:
		cal += (each - list_mean) ** 2
	cal = sqrt(cal / len(list))
	return cal

def mean(list):
	"""
    @author   : Lizhen
    Version   : 1.0 Created  2019.08.10
    Purpose   : 计算列表的平均值
    Input     : list     ——  待计算平均值的列表
    Output    : cal      ——  列表的平均值
	"""
	cal = 0.0
	for each in list:
		cal += each
	cal = cal / len(list)
	return cal


def maxabs(list):
	"""
    @author   : Lizhen
    Version   : 1.0 Created  2022.05.7
    Purpose   : 计算列表中绝对值最大的值
    Input     : list     ——  待计算平均值的列表
    Output    : cal      ——  绝对值最大值
    """
	cal = list[0]

	for each in list:
		if abs(each) > cal:
			cal = abs(each)
	return cal

def xyz2blh(x, y, z):
    """
    @author   : Lizhen
    Version   : 1.0 Created 2019.08.10
    Purpose   : xyz2blh
    """
    a = 6378137
    e2 = (1.0 / 298.257223563) * (2.0 - 1.0 / 298.257223563)
    r2 = x * x + y * y + z * z - z * z
    zz = z
    zk = 0

    while (math.fabs(zz - zk) >= 1e-4):
        zk = zz
        sinp = zz / math.sqrt(r2 + zz * zz)
        v = a / math.sqrt(1.0 - e2 * sinp * sinp)
        zz = z + v * e2 * sinp

    if r2 > 1e-12:
        b = math.atan(zz / math.sqrt(r2))
        l = math.atan2(y, x)
        h = math.sqrt(r2 + zz * zz) - v
    else:
        return 0, 0, 0
    return math.degrees(b), math.degrees(l), h


def xyz2neu(x, y, z, xr, yr, zr):
    """
    @author   : Lizhen
    Version   : 1.0 Created 2020.04.06
    Purpose   : 把某点在参心空间直角坐标系下的坐标（x, y, z)转为站心地平坐标(e,n,u).
    Input     : x ,y ,z  ——  定位结果的xyz值
    			xr,yr,zr ——  参考坐标的xyz值
    Output    : e, n ,u  ——  东方向，北方向，天顶方向的误差
    """
    dx = x - xr
    dy = y - yr
    dz = z - zr
    b, l, h = xyz2blh(xr, yr, zr)
    b = math.radians(b)
    l = math.radians(l)

    e = -math.sin(l) * dx + math.cos(l) * dy
    n = -math.sin(b) * math.cos(l) * dx - math.sin(b) * math.sin(l) * dy + math.cos(b) * dz
    u = math.cos(b) * math.cos(l) * dx + math.cos(b) * math.sin(l) * dy + math.sin(b) * dz

    return n, e, u


def blh2xyz(b, l, h):
    """
    @author   : Lizhen
    Version   : 1.0 Created 2020.04.06
    Purpose   : 把某点的大地坐标(b, l, h)转换为空间直角坐标（x, y, z).
    Input     : b ,l ,h  ——  大地纬度[-90, 90], 大地经度[-180, 180], 大地高(m)
    Output    : x, y, z  ——  空间直角坐标(m)
    """
    a2 = 6378137 * 6378137  # 地球长半轴，即赤道半径，单位 m
    b2 = 6356752.31414 * 6356752.31414 # 地球短半轴，即大地坐标系原点到两级的距离, 单位 m

    b = math.radians(b)  # 角度转为弧度
    l = math.radians(l)  # 角度转为弧度

    e = math.sqrt((a2 - b2) / (a2))  # 参考椭球的第一偏心率
    N = math.sqrt(a2) / math.sqrt(1 - e * e * math.sin(b) * math.sin(b))  # 卯酉圈半径, 单位 m

    x = (N + h) * math.cos(b) * math.cos(l)
    y = (N + h) * math.cos(b) * math.sin(l)
    z = (N * (1 - e * e) + h) * math.sin(b)
    return x, y, z  # 返回空间直角坐标(x, y, z)
