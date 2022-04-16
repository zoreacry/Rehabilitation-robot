%for real
clc
clear
Roll_r=200;     % unit: mm
Mo_corner=20;                      %应该是输入位置的0~4069？代表360度，后面换算

Len_cha=(pi*2*Roll_r)/360*Mo_corner;           %绳长变化量。

