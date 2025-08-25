# Pygame 3D model of a drone, 2025, S. Diane
import numpy as np
import pygame
import math
import time

pygame.init()

# Размер окна
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("3D drone visualization with Euler angles")
ROT_SCREEN=(0.5,0,0.5) #roll, pitch, yaw
CAM_DIST=5 #distance from camera to scene origin

def draw_text(screen, s, x, y, sz=15, с=(0, 0, 0)):  # отрисовка текста
    screen.blit(pygame.font.SysFont('Comic Sans MS', sz).render(s, True, с), (x, y))

# DEBUG=False
DEBUG=True

clock = pygame.time.Clock()

def arr_to_str(arr, prec=3, sep="\t"): #конвертирует одномерный массив в строку
    return sep.join([f"{round(v, prec)}" for v in arr])

def lim_ang(ang): # ограничение угла в пределах +/-pi
    while ang > math.pi: ang -= 2 * math.pi
    while ang <= -math.pi: ang += 2 * math.pi
    return ang

def lim_abs(val, amp): # ограничение значения по абсолютной величине
    return min(amp, max(-amp, val))
def nonlin(a, b, nominal=1): # нелинейная функция (степенная)
    return math.pow(abs(a/nominal), b-1) * a
def lin_nonlin(a, gamma, nominal=1): # линейно-нелинейная функция
    return a if abs(a)<nominal else math.pow(abs(a/nominal), gamma-1) * a
def lin_nonlin_sat(v, gamma, th1, th2): # линейно-нелинейная функция с насыщением
    return lim_abs(lin_nonlin(v, gamma, th1), th2)
def shift_to_zero(v, delta): # уменьшение значения по абсолютной величине
    return max(0, v-delta) if v>0 else min(0, v+delta)
def shift_to(v, target, delta): # сдвиг значения к целевой переменной
    return max(target, v-delta) if v>target else min(target, v+delta)

# Функция для проекции 3D точки в 2D
def project_point(x, y, z, roll, pitch, yaw):
    cp,sp,cy,sy,cr,sr=math.cos(pitch), math.sin(pitch), \
                      math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll)
    z, x = z*cp - x*sp, x*cp + z*sp
    xs, ys, zs = x*cy - y*sy, -z*cr - y*sr, y*cr - z*sr
    scale=150 #pixels per meter
    factor = CAM_DIST / (CAM_DIST + zs)
    x_proj = xs * factor * scale + WIDTH // 2
    y_proj = ys * factor * scale + int(HEIGHT *2/3)
    return int(x_proj), int(y_proj)

# Функция отрисовки осей
def draw_axes(screen, size):
    base_dirs = [(size, 0, 0), (0, size, 0), (0, 0, size)]
    colors=[(255,100,100), (100,255,100), (100,100,255)]
    for (rx, ry, rz), c in zip(base_dirs, colors):
        # Проецируем точки на экран
        start_2d = project_point(0,0,0, *ROT_SCREEN)
        end_2d = project_point(rx, ry, rz, *ROT_SCREEN)
        pygame.draw.line(screen, c, start_2d, end_2d, 2)

# Функция отрисовки дрона
def draw_drone(screen, pos, size, angles):
    x0, y0, z0 = pos
    roll, pitch, yaw = angles
    base_dirs = [(size, 0, 0), (0, size, 0), (-size, 0, 0), (0, -size, 0)]
    colors=[(255,0,0), (0,255,0), (150,150,0), (0,150,150)]
    # Рассматриваем каждое направление, вращаем его по всем осям
    for (dx, dy, dz), c in zip(base_dirs, colors):
        rx, ry, rz = drone.rotate_point(dx, dy, dz)
        # Концы линий
        start_point = (x0, y0, z0)
        end_point = (x0 + rx, y0 + ry, z0 + rz)
        # Проецируем точки на экран
        start_2d = project_point(*start_point, *ROT_SCREEN)
        end_2d = project_point(*end_point, *ROT_SCREEN)
        pygame.draw.line(screen, c, start_2d, end_2d, 4)

# Основной цикл
running = True
angle_x = 0
angle_y = 0
angle_z = 0

class Val:
    def __init__(self, max_integral=10):
        self.v, self.prev, self.integral=0,None,0
        self.max_integral=max_integral
    def set(self, v):
        self.prev, self.v = (v if self.prev is None else self.v), v
        self.integral=lim_abs(self.integral+v, self.max_integral)
    def delta(self):
        return self.v-self.prev

class PIDLog:
    def __init__(self):
        self.errors=[]
        self.factors=[]
        self.influences=[]
    def add_errors(self, arr):
        self.errors.append(arr)
    def add_factors(self, arr):
        self.factors.append(arr)
    def add_influences(self, arr):
        self.influences.append(arr)
    def save_to_file(self, file):
        with open(file, "w") as f:
            f.write("LOG (P, I, D channels):\n")
            f.write("\nERRORS (check situation):\n")
            f.writelines([f"{i}: "+arr_to_str(ln)+"\n" for i, ln in enumerate(self.errors)])
            f.write("\nFACTORS (check interpretation):\n")
            f.writelines([f"{i}: "+arr_to_str(ln)+"\n" for i, ln in enumerate(self.factors)])
            f.write("\nINFLUENCES (check responsiveness):\n")
            f.writelines([f"{i}: "+arr_to_str(ln)+"\n" for i, ln in enumerate(self.influences)])

class Drone:
    def __init__(self, position=(0, 0, 0)):
        # Положение и ориентация
        self.x, self.y, self.z = position #глобальные координаты
        self.zero_state()
        # Скорости вращений моторов
        self.rpms = [0.0, 0.0, 0.0, 0.0]  # 4 мотора
        self.control_rpms = [0.0, 0.0, 0.0, 0.0]  # уставки по скоростям моторов
        # Общие параметры
        self.ray_len = 0.23  # размер луча
        # Масса и моменты инерции
        self.mass = 1.2  # кг
        self.inrt_x, self.inrt_y, self.inrt_z = self.calc_inertia()
        self.max_motor_speed = 7000  # Ограничение скорости моторов (об/мин)
        self.kyaw = 0.0001 #ньютон-метр на обоорт (1 Н*м при 10 тыс. об/мин)
        self.k_pacif=0 #коэффициент пасификации
    def get_mat(self):
        cr, cp, cy = math.cos(self.roll), math.cos(self.pitch), math.cos(self.yaw)
        sr, sp, sy = math.sin(self.roll), math.sin(self.pitch), math.sin(self.yaw)
        mrol = [[1, 0, 0, 0], [0, cr, -sr, 0], [0, sr, cr, 0], [0, 0, 0, 1]]  # x
        mpit = [[cp, 0, sp, 0], [0, 1, 0, 0], [-sp, 0, cp, 0], [0, 0, 0, 1]]  # y
        myaw = [[cy, -sy, 0, 0], [sy, cy, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]  # z
        return np.array(mrol) @ mpit @ myaw
    def rotate_point(self, x, y, z):
        return (self.get_mat()@[x, y, z, 1])[:3]
    def un_rotate_point(self, x, y, z):
        return (np.linalg.inv(self.get_mat())@[x, y, z, 1])[:3]
    def calc_thrust_newtons(self, rpm):
        return 8e-8*rpm*rpm+5e-5*rpm #1000 оборотов дает примерно 1 ньютон для одного двигателя
    def calc_inertia(self): #учтем три слагаемых - моторы, лучи и корпус
        m_motor, m_ray = 0.05, 0.1
        m_other=self.mass-m_motor*4-m_ray*4
        r1, r2, r3 = self.ray_len, self.ray_len/2, self.ray_len/4
        ix=iy=m_motor*r1*r1/2*2 + m_ray*r2*r2/2 * 2 + m_other*r3*r3/2
        iz=m_motor*r1*r1/2*4 + m_ray*r2*r2/2 * 4 + m_other*r3*r3/2
        k_gyro=3 #TODO: fix #упрощенный учет раскрученности винтов (dynamic inertia from the gyroscopic reaction effect)
        return ix*k_gyro, iy*k_gyro, iz
    def get_sum_thrust(self):
        return sum([self.calc_thrust_newtons(v) for v in self.rpms])
    def zero_state(self):
        self.ax = self.ay = self.az = 0  # глобальные ускорения
        self.ax_l = self.ay_l = self.az_l = 0  # локальные ускорения (измеренные)
        self.vx = self.vy = self.vz = 0  # глобальные скорости
        self.vx_l = self.vy_l = self.vz_l = 0  # локальные скорости (измеренные)
        self.roll, self.pitch, self.yaw = 0, 0, 0 # глобальные углы Эйлера
        fps=30; maxi=10*fps*1 # память о 10 секундах по 1 метру или 1 радиану
        self.ex, self.ey, self.ez, self.edist = Val(maxi), Val(maxi), Val(maxi), Val(maxi)
        self.eroll, self.epitch, self.eyaw = Val(maxi), Val(maxi), Val(maxi)
        self.v_roll, self.v_pitch, self.v_yaw = 0, 0, 0
        self.a_roll, self.a_pitch, self.a_yaw = 0, 0, 0
    def get_pos(self): return (self.x, self.y, self.z)
    def get_pose(self): return (self.roll, self.pitch, self.yaw)
    def get_rot_vels(self): return (self.v_roll, self.v_pitch, self.v_yaw)
    def update(self, dt):
        self.rpms[0] = shift_to(self.rpms[0], self.control_rpms[0], 10000*dt)
        self.rpms[1] = shift_to(self.rpms[1], self.control_rpms[1], 10000*dt)
        self.rpms[2] = shift_to(self.rpms[2], self.control_rpms[2], 10000*dt)
        self.rpms[3] = shift_to(self.rpms[3], self.control_rpms[3], 10000*dt)
        self.rpms = [max(0, min(v, self.max_motor_speed)) for v in self.rpms]

        # Обновляем углы на основе скоростей моторов
        v0,v1,v2,v3=self.rpms
        f0,f1,f2,f3=[self.calc_thrust_newtons(v) for v in self.rpms]
        Mx = self.ray_len * (f1 - f2)
        My = self.ray_len * (f2 - f0)
        Mz = self.kyaw * (-v0-v2+v1+v3)
        # Обновляем позицию
        F = f0+f1+f2+f3
        a = F/self.mass
        self.ax, self.ay, self.az = self.rotate_point(0, 0, a)
        self.az-=9.8
        self.ax_l, self.ay_l, self.az_l = self.un_rotate_point(self.ax, self.ay, self.az)

        # Расчет скоростей
        self.vx += self.ax*dt
        self.vy += self.ay*dt
        self.vz += self.az*dt
        self.vx_l, self.vy_l, self.vz_l = self.un_rotate_point(self.vx, self.vy, self.vz)

        # Расчет координат
        self.x += self.vx*dt
        self.y += self.vy*dt
        self.z += self.vz*dt

        if self.z<0:
            self.z=0
            self.zero_state()

        self.z=max(0,self.z)
        # Регулируем углы
        self.a_roll += Mx/self.inrt_x * dt
        self.a_pitch += My/self.inrt_y * dt
        self.a_yaw += Mz/self.inrt_z * dt
        self.v_roll += self.a_roll * dt
        self.v_pitch += self.a_pitch * dt
        self.v_yaw += self.a_yaw * dt
        self.roll += self.v_roll * dt
        self.pitch += self.v_pitch * dt
        self.yaw += self.v_yaw * dt

    def get_up_vec(self):
        return (self.rotate_point(0,0,1))

    def get_default_rpm(self):
        k = np.dot(self.get_up_vec(), (0, 0, 1))
        thrust=self.mass * 9.8 / 4 / k
        rpm = find_inv_x(self.calc_thrust_newtons, thrust, 0, 10000, step=10)
        if DEBUG:
            print(f"Default RPM: {int(rpm)}")
        return rpm

    def stabilize(self, dt, roll, pitch, yaw, target_pos=None):
        horizon=1*dt
        rpm = self.get_default_rpm()+110
        # удержание квадрокоптера на начальной высоте чере ПД-регулирование скорости
        # позиция не учитывается т.к. нет надежных датчиков для ее получения
        predicted_vz=self.vz+horizon*self.az
        rpm += 0 - 1000*predicted_vz
        print(f"pr_vz = {predicted_vz}")

        predicted_pitch = self.pitch + horizon * self.v_pitch
        predicted_roll = self.roll + horizon * self.v_roll

        if target_pos:  # позиционное управление
            r = np.subtract(target_pos, self.get_pos())
            if np.linalg.norm(r) > 0.1:
                rx, ry, rz = self.rotate_point(*r)
                corr_z = 1000 * rz
                pitch += 0.3 * rx
                roll -= 0.3 * ry

        e_roll=roll - predicted_roll
        e_pitch=pitch - predicted_pitch

        #методы фазовой декомпозиции+нечеткой логики+пасификации
        maxang=math.pi/2
        maxrotvel=maxang / 5

        # ОШИБКА ПО СКОРОСТИ
        vroll_ = min(maxrotvel, abs(self.v_roll)) / maxrotvel
        vpitch_ = min(maxrotvel, abs(self.v_pitch)) / maxrotvel
        near_stable_or_stable = (1 - vroll_) * (1 - vpitch_)
        stable = math.pow(near_stable_or_stable, 4)
        near_stable = (near_stable_or_stable - stable)/0.63 # https://www.wolframalpha.com/input?i=max(x-x^4)
        nonstable = 1 - near_stable_or_stable
        ww1 = [stable, near_stable, nonstable]
        s1 = sum(ww1)
        ww1 = [w/s1 for w in ww1]

        # ОШИБКА ПО ПОВОРОТУ
        roll_=min(maxang, abs(e_roll))/maxang
        pitch_=min(maxang, abs(e_pitch))/maxang
        near_hor_or_hor = (1 - roll_) * (1 - pitch_)
        hor=math.pow(near_hor_or_hor, 4)
        near_hor = (near_hor_or_hor - hor)/0.63 # https://www.wolframalpha.com/input?i=max(x-x^4)
        nonhor= 1 - near_hor_or_hor
        ww2=[hor, near_hor, nonhor]
        s2 = sum(ww2)
        ww2 = [w / s2 for w in ww2]

        mat=[[0.01, 0.1, 1], [0.1, 0.1, 0.1], [1, 0.1, 0.01]]
        mat=mat / np.sum(mat, axis=1)[:, np.newaxis] #нормализация
        kp=np.array(ww1)@mat@ww2

        # возведение в степень позволяет ограничить регулировнание только критическими случаями
        # kp=math.pow(kp, 2)
        # извлечение корня позволяет расширить область регулировнания
        kp=math.pow(kp, 0.3)

        self.k_pacif+=0.5*(kp-self.k_pacif)
        k=1000*self.k_pacif

        predicted_vx=self.vx_l + horizon*self.ax_l
        predicted_vy=self.vy_l + horizon*self.ay_l

        corr_x = k * predicted_vx
        corr_y = k * predicted_vy
        corr_x*=-1

        corr_x=int(lim_abs(corr_x, 10000))
        corr_y=int(lim_abs(corr_y, 10000))
        # corr_z=0

        if DEBUG:
            print(f"ww1 {arr_to_str(ww1, 3,', ')}\n"
                  f"ww2 {arr_to_str(ww2, 3,', ')}\n"
                  f"k_pacif={drone.k_pacif:.3f}, k {k:.3f}\n"
                  f"corr_x {corr_x}, corr_y {corr_y}")

        e_roll, e_pitch, e_yaw = self.eroll, self.epitch, self.eyaw
        e_roll.set(lim_ang(roll - self.roll))
        e_pitch.set(lim_ang(pitch - self.pitch))
        e_yaw.set(lim_ang(yaw - self.yaw))

        #регулирование наклона
        pid1 = [500, 10, 5000]
        ee_roll = [e_roll.v, e_roll.integral * dt, e_roll.delta() / dt]
        pid2 =[500, 10, 5000]
        ee_pitch = [e_pitch.v, e_pitch.integral * dt, e_pitch.delta() / dt]

        d_roll = np.dot(pid1, ee_roll)
        d_pitch = np.dot(pid2, ee_pitch)

        #регулирование поворота
        pid3 = [500, 0, 10000]
        eez = [e_yaw.v, e_pitch.integral * dt, e_yaw.delta() / dt]

        d_yaw = np.dot(pid3, eez)

        self.control_rpms[0] = rpm - d_pitch - d_yaw - corr_x + corr_z
        self.control_rpms[1] = rpm + d_roll + d_yaw + corr_y + corr_z
        self.control_rpms[2] = rpm + d_pitch - d_yaw + corr_x + corr_z
        self.control_rpms[3] = rpm - d_roll + d_yaw - corr_y + corr_z

        self.check_motors()

    def check_motors(self):
        if DEBUG:
            if any(v<0 or v>self.max_motor_speed for v in self.rpms):
                print(f"Limit reached: {self.control_rpms}")

    def draw(self, screen):
        draw_drone(screen, (self.x, self.y, self.z), self.ray_len,
                   (self.roll, self.pitch, self.yaw))


fps=30
dt=1/fps

t_sim=0
frame=0
drone = None
dd=[]
# Начальные возмущенные значения углов
def reset():
    global drone, dd, t_sim, frame
    drone = Drone((0, 0, 1))
    drone.roll += 0.03  # крен
    drone.pitch += 0.03  # тангаж
    drone.yaw += 0.03  # рыскание
    drone.rpms=[5500,5500,5500,5500]
    dd = []
    t_sim=0
    frame=0

# target=[0, 0, 0.5]
target=[0, 0.5, 1.5]

#подбор значения по монотонной нелинейной функции
def find_inv_x(f, y, xmin, xmax, step=0.1):
    xx=np.arange(xmin, xmax, step)
    return xx[np.argmin([abs(y-f(x)) for x in xx])]

PAUSE=False

reset()
while running and t_sim<100:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_p:
                PAUSE=not PAUSE
            if event.key == pygame.K_r:
                reset()

    if PAUSE:
        time.sleep(0.1)
        continue

    if DEBUG:
        print(f"TIME: {t_sim:.2f}")

    drone.stabilize(dt, 0, 0, 0, target)

    drone.update(dt)

    d=np.linalg.norm(np.subtract(drone.get_pos(), target))
    dd.append(d)

    screen.fill((255, 255, 255))
    draw_axes(screen, 1)
    drone.draw(screen)

    draw_text(screen, f"time = {t_sim:.3f}", 5, 5)
    draw_text(screen, f"xyz = {arr_to_str(drone.get_pos(), 3, ', ')}", 5, 25)
    draw_text(screen, f"rpy = {arr_to_str(drone.get_pose(), 3, ', ')}", 5, 45)
    draw_text(screen, f"r'p'y' = {arr_to_str(drone.get_rot_vels(), 3, ', ')}", 5, 65)
    draw_text(screen, f"pacif = {drone.k_pacif:.3f}", 5, 85)

    if DEBUG:
        print(f"xyz={arr_to_str(drone.get_pos(), 3, ', ')}\n"
              f"rpy={arr_to_str(drone.get_pose(), 3, ', ')}\n"
              f"r'p'y'={arr_to_str(drone.get_rot_vels(), 3, ', ')}\n"
              f"sum(rpms)={sum(drone.rpms)}")

    pygame.display.flip()
    clock.tick(fps)
    frame+=1
    t_sim+=dt

print(f"D_avg={np.mean(dd)}")

pygame.quit()
