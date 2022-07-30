#!/usr/bin/python3.6
from platform import machine
#import opt_calc
import numpy as np
import numpy.linalg as LA
import math
from scipy.optimize import minimize
from scipy.optimize import minimize_scalar
import subprocess as sp
#import visualizer
import random
import time
import datetime
import os.path
# from params import *

th_scale = 0.3
S = np.diag([1.,1.,th_scale])
pS = np.diag([1.,1.,1/th_scale])

##### その場で使うような簡単な関数 #####
### 経路周りの関数
# 初期位置と経路データから経路を計算する
def calc_route(init, route_data,max_accel,accel_ratio):
    vias = []
    t = 0
    
    for route in route_data:
        tf = route[0]
        t += tf / math.sqrt(max_accel)
        r,v,a = opt_calc.calc(t,init,route_data,max_accel,accel_ratio)
        vias.append([r,v,a])
    return vias

    # 1本のベクトルからデータを整える
def shape(x):
    routes = []
    for i in range(0, len(x), 7):
        routes.append([x[i], x[i+1:i+4], x[i+4:i+7]])
    return routes

# フルデータから経路に変換する
def fulldata_to_route(opt_route):
    ans = []
    for route in opt_route:
        ans.append([route[0], route[-1][0], route[-1][1]])
    return ans

# データを1本のベクトルに展開する
def expand(routes):
    x = []
    for route in routes:
        x.append(route[0])
        for i in range(3):
            x.append(route[1][i])
        for i in range(3):
            x.append(route[2][i])
    return x

    # 初期位置と1本のベクトルから経路データから経路を計算する
def calc_route_fromx(init, x,max_accel,accel_ratio):
    return calc_route(init, shape(x),max_accel,accel_ratio)

# 経路データから時間を抽出する
def calc_time_fromx(x):
    return sum([route[0] for route in shape(x)])

def calc_time_shape(vias_data):
    t = 0
    for vias in vias_data:
        t += vias[0]
    return t

def calc_time_shape_n(vias_data,n):
    t = 0
    for i in range(n):
        t += vias_data[i][0]
    return t

# vias dataを出力する
def str_route_data(routes):
    moji = "vias_data = [\n"
    for route in routes:
        moji += "    "
        moji += str([route[0], list(route[1]), list(route[2])]) + ",\n"
    moji += "]"
    return moji

# わーい
def str_full_route(init, routes,max_accel,accel_ratio):
    vias = calc_route(init, routes,max_accel,accel_ratio)
    moji = "  {x = " + str(list(init)) + ", v = [0.0, 0.0, 0.0]},\n"
    for i in range(len(vias)-1):
        x,v,a = vias[i]
        moji += "  {type= \"PosFixed\", x = " + str(list(x))
        moji += ",v = " + str(list(v)) + "},\n"
    x,v,a = vias[-1]
    moji += "  {x = " + str(list(x)) + ", v = [0.0, 0.0, 0.0]},\n"
    return moji

### 計算するあたり
# 回転行列
def Rot(th):
    c,s = np.cos(th),np.sin(th)
    return np.array([[c,-s],[s,c]])

# n回転したθを取る
def theta_remainder(theta):
    twopi = 2.0 * math.pi
    theta = theta % twopi
    if theta > math.pi:
        theta -= twopi
    return theta

# 任意の四角形と点の内外判定
def rect_point_inout(r0,point,machine_rect):
    r = np.array(r0)
    R = Rot(r[-1])
    machine_rect_np = np.array(machine_rect)
    machine_rect_v = []
    for mp in machine_rect_np:
        machine_rect_v.append(R@mp + r[:2])
    ans = -float('inf')
    for i in range(4):
        v1 = machine_rect_v[i-1] - machine_rect_v[i]
        v2 = point - machine_rect_v[i]
        ans = max(ans,v1[0]*v2[1]-v1[1]*v2[0])
    return ans

# 任意の円と点の内外判定
def circle_point_inout(r0,point,machine_radius):
    r = np.array(r0)
    ans = ((r[0]-point[0])**2+(r[1]-point[1])**2) - machine_radius**2#内側に入っていれば負を返す
    return ans

# 円との距離
def circle_distance(r0, circle_pos, machine_rect, circle_r):
    # print("machine is defined as rectangle")
    R = Rot(r0[-1])
    machine_rect_np = np.array(machine_rect)
    machine_rect_v = []
    for mp in machine_rect_np:
        machine_rect_v.append(R@mp + r0[:2])
    ans = float('inf')
    for v in machine_rect_v:
        #なんやこれ
        ans = min(ans,(circle_pos[0] - v[0]) ** 2 + (circle_pos[1] - v[1]) ** 2 - circle_r ** 2)
    return ans

#円との距離、機体を円で定義したバージョン
def circle_distance_circle_machine(r0, circle_pos, machine_size, circle_r):
    # print("machine is defined as circle")
    # machine size defined as circle is expected to be the shape below.
    # machine_size = np.array([r, center]) ? 
    return (circle_pos[0] - r0[0])**2 + (circle_pos[1] - r0[1])**2 - (circle_r + machine_size[0])**2

# 経路と円の距離
def circle_route_strict(init, x, area_num, circle_pos, machine_size, circle_r, N,max_accel,accel_ratio):
    routes = shape(x)
    t = calc_time_shape_n(routes,area_num) / math.sqrt(max_accel)
    ans = float('inf')
    tf = routes[area_num][0] / math.sqrt(max_accel)
    for i in range(N):
        t += tf / N
        r = opt_calc.calc(t,init,routes,max_accel,accel_ratio)[0]
        if(is_machine_defined_as_circle(machine_size)):
            ans = min(ans, circle_distance_circle_machine(r, circle_pos, machine_size, circle_r))
        else:
            ans = min(ans, circle_distance(r, circle_pos, machine_size, circle_r))
    return ans

# 壁と経路の距離
def wall_route_strict(init, x, area_num, wall_pos, machine_size, N_route,max_accel,accel_ratio):
    routes = shape(x)
    t = calc_time_shape_n(routes,area_num) / math.sqrt(max_accel)
    ans = float('inf')
    tf = routes[area_num][0] / math.sqrt(max_accel)
    for i in range(N_route):
        t += tf / N_route
        r = opt_calc.calc(t,init,routes,max_accel,accel_ratio)[0]

        if(is_machine_defined_as_circle(machine_size)):
            wall_min_pos = calc_wall_minimum_point(wall_pos,r[:2])
            ans = min(ans, circle_point_inout(r, wall_min_pos, machine_size[0]))
        else:
            R = Rot(r[-1])
            machine_rect_np = np.array(machine_size)
            machine_rect_v = []
            for mp in machine_rect_np:
                machine_rect_v.append(R@mp + r[:2])

            for v in machine_rect_v:
                wall_min_pos = calc_wall_minimum_point(wall_pos,v)
                ans = min(ans, rect_point_inout(r, wall_min_pos, machine_size))
    return ans

#機体は円として定義されているか
def is_machine_defined_as_circle(machine_size):
    return machine_size.size == 3

def check_in_zone(pos,zone):
    return min(pos[0]-zone[0][0], zone[0][1]-pos[0],
                     pos[1]-zone[1][0], zone[1][1]-pos[1])

def hitter_in_zone_strict(init, x, area_num,zone,machine_rect, N):
    routes = shape(x)

    r_,v_ = [S@np.array(init[0]), S@np.array(init[1])]
    i = 0
    while True:
        tf,raw_px,raw_pv0 = routes[i]
        px_,pv0_ = [pS@np.array(raw_px), pS@np.array(raw_pv0)]
        if i < area_num:
            r_,v_ = opt_calc.opt_route(tf, px_, pv0_, r_, v_)
            i += 1
        else:
            break

    ans = float('inf')
    for i in range(N):
        t = i * tf / N
        r = pS@(opt_calc.opt_route(t, px_, pv0_, r_, v_)[0])
        for j in range(4):
            pos = r[:2] + Rot(r[-1])@machine_size[j]
            ans = min(ans,max(check_in_zone(pos,zone),check_in_zone(pos,ball_zone_1),check_in_zone(pos,ball_zone_2)))
    return ans

# ゾーンに入っているかを見る
def in_zone(r0, machine_size, Xrange, Yrange):
    r = np.array(r0)
    ans = float('inf')
    for i in range(4):
        pos = r[:2] + Rot(r[-1])@machine_size[i]
        ans = min([ans,
            pos[0]-Xrange[0], Xrange[1]-pos[0],
            pos[1]-Yrange[0], Yrange[1]-pos[1]])
    return ans

# マシン中心がエリアに入ってるかを見る
def center_in_area(r, stricts):
    hoges = [[r[i]-stricts[i][0], stricts[i][1]-r[i]] for i in range(len(r))]
    ans = 0.0
    for p in hoges:
        for x in p:
            if x < 0:
                ans -= (x**2)
    return ans

# 各経路の時間
def forward_check(x, num):
    return x[7*num]

# 指定位置にいるかチェック
def distination_check(x,distination,eff):
    eff_diag = np.diag(eff)
    np_d = np.array(distination)
    np_x = np.array(x)
    e = eff_diag@(np_d - np_x)
    return e@e

def speed_check(x,distination,eff):
    # ans = float('inf')
    # for i in range(3):
    #     ans = min(ans,(abs(distination[i]) - abs(x[i])) * eff[i])
    # return ans
    eff_diag = np.diag(eff)
    np_d = np.array(distination)
    np_x = np.array(x)
    e = eff_diag@(np_d - np_x)
    return e@e

# 停止しているかチェック
def stop(v0):
    v = np.array(v0)
    return v@v

# 指定の方向以外速度0かチェック
def v_axis_restriction(v0,restriction):
    restriction_diag = np.diag(restriction)
    v = restriction_diag@np.array(v0)
    return v@v
    #return v0[0]**2 + v0[2]**2


# def route_in_zone(init, x, machine_size, Xrange, Yrange):
#     ans = float('inf')
#     S = np.diag([1.,1.,0.3])
#     pS = np.diag([1.,1.,1/0.3])
#     r_ = S@np.array(init)
#     v_ = S@np.array([0,0,0],dtype=float)
#     route_data = shape(x)
#     j = 0
#     for route in route_data:
#         tf,raw_px,raw_pv0 = route
#         px_ = pS@np.array(raw_px)
#         pv0_= pS@np.array(raw_pv0)
#         N = 3
#         for i in range(N):
#             ans = min(ans, in_zone(opt_calc.opt_route(i*tf/N, px_, pv0_, r_, v_,accel_ratio[j])[0],machine_size, Xrange,Yrange))
#         r_,v_ = opt_calc.opt_route(tf, px_, pv0_, r_, v_,accel_ratio[j])
#         r,v = pS@r_, pS@v_
#         j += 1
#     return ans

# 壁との最小距離の点を求める
def calc_wall_minimum_point(wall_pos,machine_pos):
    a = np.array([wall_pos[1][0] - wall_pos[0][0],wall_pos[1][1] - wall_pos[0][1]])
    b = np.array([machine_pos[0] - wall_pos[0][0],machine_pos[1] - wall_pos[0][1]])
    r = (a@b)/(a@a)
    if r <= 0:
        return [wall_pos[0][0],wall_pos[0][1]]
    elif r >= 1:
        return [wall_pos[1][0],wall_pos[1][1]]
    else:
        return [wall_pos[0][0] + r * a[0],wall_pos[0][1] + r * a[1]]

def route_not_rotate(x,area_num,max_accel,accel_ratio):
    vias = calc_route_fromx([[0., 0., 0.],[0.,0.,0.]], x,max_accel,accel_ratio)
    if area_num == 0:
        dtheta = vias[area_num][0][2] - 0
        domega = vias[area_num][1][2] - 0
    else:
        dtheta = vias[area_num][0][2] - vias[area_num - 1][0][2]
        domega = vias[area_num][1][2] - vias[area_num - 1][1][2]
    return (dtheta**2) + (domega**2)

def last_route_const_acc(x,max_accel,accel_ratio):
    vias = calc_route_fromx([0., 0., 0.], x,max_accel,accel_ratio)
    x0 = np.array(vias[-2][0][:2])
    x1 = np.array(vias[-1][0][:2])
    dx = x1 - x0
    e = dx / LA.norm(dx)
    vertical = np.array([[0,-1],[1,0]]) @ e
    v = np.array(vias[-2][1][:2])
    return v@vertical# 変位から求めた速度ベクトルと終点から一つ前の分割点での速度が平行な時0になる



# コールバック
iteration_count = 0
def basic_callback(x):
    global iteration_count
    print(iteration_count,calc_time_fromx(x))
    iteration_count += 1

# 最適化計算を行う
def optimize(init_r, x0, cons,max_accel,accel_ratio, maxiter=100, visualize=True):
    result = minimize(
        calc_time_fromx,
        x0 = x0,
        constraints = cons,
        method = "SLSQP",
        options = {"maxiter":maxiter},
        callback = lambda x:basic_callback(x),
    )
    opt_x = result["x"]
    opt_route = shape(opt_x)
    init_cost = calc_time_fromx(x0)
    final_cost = calc_time_fromx(opt_x)

    print("=====END====")
    print(result)
    
    print("cost:", init_cost, "->", final_cost)

    print("~~~ for ROP ~~~")
    print(str_full_route(init_r, opt_route,max_accel,accel_ratio))
    print("~~~~~~")

    print("~~~ copy&paste below~~~")
    print(str_route_data(opt_route))
    print("~~~~~~")
    
    if visualize:
        visualizer.draw_route(init_r, opt_route,max_accel,accel_ratio)
    return result


# def gen_in_field(init,machine_size):
#     return {
#         'type':'ineq',
#         'fun':lambda x:route_in_zone(init,x,machine_size,field_area[0],field_area[1])
#     }

def gen_in_area(init,via_num,machine_size,area,max_accel,accel_ratio):
    return {
        'type':'ineq',
        'fun':lambda x:in_zone(calc_route_fromx(init,x,max_accel,accel_ratio)[via_num][0],machine_size,area[0],area[1])
    }

# 目的地に到達しているかチェック
def gen_distination_check(init,via_num,distination,max_accel,accel_ratio,eff = [1,1,1]):
    return {
        'type':'eq',
        'fun':lambda x:distination_check(calc_route_fromx(init,x,max_accel,accel_ratio)[via_num][0],distination,eff)
    }

# 目標速度に到達しているかチェック
def gen_speed_check(init,via_num,distination,max_accel,accel_ratio,eff = [1,1,1]):
    return {
        'type':'eq',
        'fun':lambda x:speed_check(calc_route_fromx(init,x,max_accel,accel_ratio)[via_num][1],distination,eff)
    }

# 停止しているかチェック
def gen_stop_check(init, via_num,max_accel,accel_ratio):
    return {
        'type':'eq',
        'fun':lambda x:stop(calc_route_fromx(init, x,max_accel,accel_ratio)[via_num][1])
    }

# 時間を逆回ししていないことを確認
def gen_forward_check(num, minimum_value=0.0):
    return {
        'type':'ineq',
        'fun':lambda x:forward_check(x, num) - minimum_value
    }

def gen_wall_route_strict(init_r, area_num, wall_pos, machine_size, N_route,max_accel,accel_ratio):
    return {
        'type':'ineq',
        'fun':lambda x:wall_route_strict(init_r, x, area_num, wall_pos, machine_size, N_route,max_accel,accel_ratio)
    }

def gen_circle_route_strict(init_r,area_num,circle_pos,machine_size,circle_r,N,max_accel,accel_ratio):
    return {
        'type':'ineq',
        'fun':lambda x:circle_route_strict(init_r, x, area_num, circle_pos, machine_size, circle_r, N,max_accel,accel_ratio)
    }

# 最後のrouteで回転しない制約
def gen_route_not_rotate_strict(area_num,max_accel,accel_ratio):
    return {
        'type':'eq',
        'fun':lambda x:route_not_rotate(x,area_num,max_accel,accel_ratio)
    }

# 最後のrouteで等加速度運動をする制約
def gen_last_route_const_acc_strict():
    return {
        'type':'eq',
        'fun':lambda x:last_route_const_acc(x)
    }

def gen_last_route_move_y_only_strict(init, via_num,max_accel,accel_ratio):
    return {
        'type':'eq',
        'fun':lambda x:v_axis_restriction(calc_route_fromx(init, x,max_accel,accel_ratio)[via_num][1],[1,0,1])
    }

def gen_center_in_area(init, via_num, stricts,max_accel,accel_ratio):
    return {
        'type':'ineq',
        'fun':lambda x:center_in_area(calc_route_fromx(init, x,max_accel,accel_ratio)[via_num][0], stricts)
    }

def generate_random_x0(vias_data):
    ret = []
    for i in range(len(vias_data)):
        ret.append([random.uniform(2,5),[random.uniform(-10,10),random.uniform(-10,10),random.uniform(-10,10)],[random.uniform(-10,10),random.uniform(-10,10),random.uniform(-10,10)]])
    return expand(ret)

def optimize_loop(vias_data,init_r, x0, cons,max_accel,accel_ratio, maxiter=200,visualize=False):
    file_name = str(os.path.abspath(os.path.dirname(__file__))) + "/route_txt/" + str(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')) + ".txt"
    min_time = float('inf')
    count = 0
    file_write_count = 0

    while True:
        print(count,"回目")
        print(file_write_count,"回")
        global iteration_count
        iteration_count = 0
        result = optimize(init_r, x0, cons,max_accel,accel_ratio,maxiter,visualize)
        if result["success"]:
            if min_time > calc_time_fromx(result["x"]):
                print("===============================")
                print(calc_time_fromx(result["x"]))
                print(str_route_data(shape(result["x"])))
                min_time = calc_time_fromx(result["x"])
                with open(file_name,'a') as log_file:
                    file_write_count += 1
                    log_file.write(str_route_data(shape(result["x"])) + "\n\n")
        if count % 5 == 0:
            x0 = generate_random_x0(vias_data)
        else:
            x0 = result["x"]
        count += 1