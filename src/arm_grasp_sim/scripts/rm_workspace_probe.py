#!/usr/bin/env python3
"""工作空间实测: 扫候选抓取点, 打印实际到达位置。用法: python3 ~/rm_workspace_probe.py"""
import time
from robomaster import robot

pos = {}
ep = robot.Robot()
print('[1] 连接 ...'); ep.initialize(conn_type='ap'); print('    OK')
ep.robotic_arm.sub_position(freq=10, callback=lambda m: pos.update(x=m[0], y=m[1]))
ep.robotic_arm.recenter().wait_for_completed(timeout=15); time.sleep(1)
print('    recenter 后 pos=(%s,%s)' % (pos.get('x'), pos.get('y')))

# 候选点: (目标x, 目标y) mm —— 覆盖低位抓取可能用到的区域
CANDS = [(160,40),(180,40),(200,40),(220,40),(240,40),(270,60),(200,20),(240,20)]
print('[2] 依次探测 %d 个点, 每个停3秒读实际位置' % len(CANDS))
input('确认臂周围无障碍, 按回车开始 ...')
for tx, ty in CANDS:
    ep.robotic_arm.moveto(x=tx, y=ty).wait_for_completed(timeout=15)
    time.sleep(3)
    print('  目标(%3d,%3d) -> 实际(%s,%s)  x误差=%s y误差=%s' % (
        tx, ty, pos.get('x'), pos.get('y'),
        None if pos.get('x') is None else pos['x']-tx,
        None if pos.get('y') is None else pos['y']-ty))
print('[3] 回 recenter ...')
ep.robotic_arm.recenter().wait_for_completed(timeout=15)
ep.close(); print('完成。把全部输出带回来。')
