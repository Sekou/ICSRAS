#2025, S. Diane
#Script for image annotation

import sys, pygame, numpy as np, math, os, re
import tkinter as tk; from tkinter import filedialog
from os import listdir
from matplotlib.pyplot import draw_all
from sympy import sec

root = tk.Tk(); root.withdraw()
pygame.font.init()

def get_some_colors():
    return [(220, 0, 0), (0, 220, 0), (0, 0, 220),(150, 150, 0), (0, 150, 150), (150, 0, 150),
     (120, 50, 50), (50, 120, 50), (50, 50, 120), (100, 100, 50), (50, 100, 100), (100, 50, 100)]

COLORS=get_some_colors()
FOLDER_IMAGES="images/" # place your images here for annotation
FOLDER_LABELS="labels/" # results are saved here

def draw_text(screen, s, x, y, sz=15, с=(0, 0, 0)):  # отрисовка текста
    screen.blit(pygame.font.SysFont('Comic Sans MS', sz).render(s, True, с), (x, y))
def draw_text2(screen, s, x, y, sz=15):  # контрастная отрисовка текста
    draw_text(screen, s, x, y, с=(0, 0, 0))
    draw_text(screen, s, x + 2, y + 2, с=(255, 255, 255))
def draw_multiline_text(screen, text, pos, sz=25, color=(0,0,0), transf=False):
    for i,t in enumerate(text.split("\n")): # отрисовка многострочного текста
        draw_text2(screen, t, pos[0], pos[1]+0.7*sz*i, sz)

def check_create_dir(dir):
    os.makedirs(dir, exist_ok=True)
    return dir
def check_create_file_dir(file):
    file_=file.replace("\\", "/")
    dir = file_[:file_.rfind("/")+1]
    os.makedirs(dir, exist_ok=True)
    return dir
def get_dir_name(file_path):
    res = file_path.replace("\\", "/")
    return file_path[:res.rfind("/")]
def get_short_name(file_path):
    pathname, extension = os.path.splitext(file_path)
    return pathname.replace("\\", "/").split('/')[-1]+extension

def load_image(file_name):
    myimage = pygame.image.load(file_name)
    imagerect = myimage.get_rect()
    sz=[imagerect.w, imagerect.h]
    return myimage, sz

def get_label_name(im_name):
    im_name_=im_name.replace("\\", "/")
    a, b=FOLDER_IMAGES, FOLDER_LABELS
    i=im_name_.rfind("/")
    dir, name = im_name_[:i] +"/"+ b, im_name_[i + 1:]
    lb_name = dir + re.sub("(\.jpg|\.jpeg|\.png)", ".txt", name)
    return lb_name

def try_open_labels(lb_file, sz):
    rects=[]
    if lb_file:
        with open(lb_file, "r") as f:
            text = f.read()
            if text:
                try: bb = eval(text); rects = [MyRect(*b[:4], b[4]) for b in bb]
                except: lines = text.split("\n"); rects = [MyRect(0, 0, 0, 0, 0).parse_yolo_str(l, *sz) for l in lines]
    return rects

class MyRect:
    def __init__(self, x, y, w, h, id_class=-1):
        self.id_class=id_class
        self.selected=False
        self.x, self.y, self.w, self.h = x, y, w, h
    def get_bb(self):
        return [self.x, self.y, self.w, self.h]
    def get_yolo_str(self, img_w, img_h):
        x,y=(self.x+self.w/2)/img_w, (self.y+self.h/2)/img_h
        return f"{self.id_class} {x:.3f} {y:.3f} {self.w/img_w:.3f} {self.h/img_h:.3f}"
    def parse_yolo_str(self, line, img_w, img_h):
        vv=line.split(" ")
        self.id_class = int(vv[0])
        self.x, self.y, self.w, self.h = [float(v) for v in vv[1:]]
        self.x, self.y, self.w, self.h = (self.x-self.w/2)*img_w, (self.y-self.h/2)*img_h, self.w*img_w, self.h*img_h
        return self
    def draw(self, screen):
        w=4 if self.selected else 2
        pygame.draw.rect(screen, COLORS[self.id_class%len(COLORS)], self.get_bb(), w)
        draw_text2(screen, str(self.id_class), self.x, self.y)
    def fix(self):
        if self.w < 0: self.w, self.x = -self.w, self.x + self.w
        if self.h < 0: self.h, self.y = -self.h, self.y + self.h
        return self
    def copy(self):
        return MyRect(self.x, self.y, self.w, self.h, self.id_class)
    def contains(self, pt):
        return self.x<=pt[0]<=self.x+self.w and self.y<=pt[1]<=self.y+self.h

def sorted_rects(rects, sz):
    # sorting rects 1. by class 2. by Y-coordinate, 3. by X-coordinate
    a, b, c = sz[1] * sz[0], sz[1], 1
    return sorted(rects, key=lambda r: r.id_class * a + r.y * b + r.x * c)

def try_change_rect(rect, key, mod, kmv=5):
    if mod & pygame.KMOD_CTRL:
        if key == pygame.K_UP: rect.y, rect.h=rect.y-kmv/2, rect.h+kmv
        if key == pygame.K_DOWN: rect.y, rect.h=rect.y+kmv/2, rect.h-kmv
        if key == pygame.K_RIGHT: rect.x, rect.w=rect.x-kmv/2, rect.w+kmv
        if key == pygame.K_LEFT: rect.x, rect.w=rect.x+kmv/2, rect.w-kmv
    else:
        if key == pygame.K_UP: rect.y-=kmv
        if key == pygame.K_DOWN: rect.y+=kmv
        if key == pygame.K_RIGHT: rect.x+=kmv
        if key == pygame.K_LEFT: rect.x-=kmv

def main():
    myimage, sz = load_image("test.jpg")
    screen = pygame.display.set_mode(sz)

    pygame.display.set_caption('Image annotation, 2025, by S. Diane')
    timer = pygame.time.Clock()
    fps = 10; dt = 1 / fps

    rect=None; sel_rect=None; rects=[]; mem_rects=[]; last_key= None; last_mod = None
    curr_id=0; file_path=None; last_img_dir=None; last_img_path=None
    show_help=False; show_hint=False; hint_lines=[]
    if os.path.isfile("hint.txt"):
        with open("hint.txt", "r", encoding="utf-8") as f:
            hint_lines=f.readlines()

    down=False
    img_files=[]

    check_create_dir(FOLDER_IMAGES)
    check_create_dir(FOLDER_LABELS)

    def try_sel_rect():
        nonlocal sel_rect
        for r in rects: r.selected = False
        rr = [r for r in rects if r.contains(ev.pos)]
        if len(rr) > 0: (sel_rect := rr[0]).selected = True
        else: sel_rect = None

    def draw_all(screen):
        screen.fill((255, 255, 255))
        screen.blit(myimage, myimage.get_rect())
        if rect: r = rect.copy().fix(); r.draw(screen)
        for r in rects: r.draw(screen)
        draw_text2(screen, f"New obj. id = '{curr_id}'", 5, 5)
        if last_img_path: draw_text2(screen, f"File = '{get_short_name(last_img_path)}'", 5, 25)
        else: draw_text2(screen, f"Press 'i' to load an image. Press 'h' for help", 5, 25)
        draw_text2(screen, f"Num labels = {len(rects)}", 5, 45)
        if show_help: draw_multiline_text(screen, "\n".join([
            "c - clear labels", "d - delete selected or last label", "n - duplicate label",
            "l - load pixel/YOLO labels", "i - load image", "g - show hint", "y - save YOLO labels",
            "s - save pixel labels", "z - restore recent labels", "arrows - move labels",
            "(0-9) - change new label number", "shift+(0-9) - change selected label number",
            "SPACE - next image", "BACKSPACE - previous image"]), (5, 65))
        if show_hint: draw_multiline_text(screen, "".join(hint_lines), (sz[0] * 2 // 3, 5))
        pygame.display.flip(); timer.tick(fps)

    def open_img(file_path):
        nonlocal last_img_path, last_img_dir, myimage, sz, screen
        nonlocal mem_rects, rects, rect, curr_id, lb_file
        last_img_path, last_img_dir = file_path, get_dir_name(file_path)
        myimage, sz = load_image(file_path)
        screen = pygame.display.set_mode(sz)
        mem_rects = rects
        rects, rect, curr_id = [], None, 0
        lb_file = get_label_name(file_path)
        if os.path.isfile(lb_file): rects = try_open_labels(lb_file, sz)

    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT: sys.exit(0)
            if ev.type == pygame.MOUSEBUTTONDOWN:
                if pygame.key.get_pressed()[pygame.K_LSHIFT]:
                    try_sel_rect()
                    if sel_rect:
                        d, moving=np.subtract(ev.pos, (sel_rect.x, sel_rect.y)), True
                        while moving:
                            for ev in pygame.event.get():
                                if ev.type == pygame.MOUSEMOTION:
                                    sel_rect.x=ev.pos[0]-d[0]; sel_rect.y=ev.pos[1]-d[1]
                                    draw_all(screen)
                                if ev.type == pygame.MOUSEBUTTONUP: moving=False; break
                rect=MyRect(*ev.pos, 1, 1, curr_id)
                down=True
            if ev.type == pygame.MOUSEBUTTONUP:
                down=False
                if rect and abs(rect.w*rect.h)>50: rects.append(sel_rect:=rect.copy().fix())
                else: try_sel_rect()
                rect=None
            if ev.type == pygame.MOUSEMOTION:
                if not down and pygame.mouse.get_pressed()[0]:
                    rect = MyRect(*ev.pos, 1, 1, curr_id)
                    down = True
                if down: rect.w, rect.h = ev.pos[0]-rect.x, ev.pos[1]-rect.y
            if ev.type == pygame.KEYUP:
                last_key, last_mod=None, None
            if ev.type == pygame.KEYDOWN:
                last_key, last_mod=ev.key, ev.mod
                if pygame.K_0 <= ev.key <= pygame.K_9:
                    curr_id=ev.key-pygame.K_0
                    if pygame.key.get_pressed()[pygame.K_LSHIFT] and sel_rect:
                        sel_rect.id_class=curr_id
                if ev.key == pygame.K_s:
                    with open(tmp_name:="annotation.txt", "w") as f:
                        rr=[[r.id_class]+r.get_bb() for r in sorted_rects(rects, sz)]
                        f.write(str(rr))
                        print("Saved:", tmp_name)
                if ev.key == pygame.K_y:
                    with open(tmp_name:="annotation_yolo.txt", "w") as f:
                        ss=[r.get_yolo_str(*sz) for r in sorted_rects(rects, sz)]
                        f.write("\n".join(ss))
                        print("Saved:", tmp_name)
                    if file_path:
                        lb_file=get_label_name(file_path)
                        check_create_file_dir(lb_file)
                        with open(lb_file, "w") as f:
                            f.write("\n".join(ss))
                            print("Saved:", lb_file)
                if ev.key == pygame.K_i:
                    dir=last_img_dir if last_img_dir else FOLDER_IMAGES
                    file_path = filedialog.askopenfilename(initialdir=dir)
                    if file_path:
                        open_img(file_path)
                        img_files = [f for f in listdir(last_img_dir) if
                                     os.path.isfile(os.path.join(last_img_dir, f)) and re.match(".+(jpg|jpeg|png)",f)]
                        img_files=sorted(img_files, key=lambda x: int(re.sub('[^0-9]','', x)))
                if ev.key == pygame.K_l:
                    lb_file = filedialog.askopenfilename(initialdir=FOLDER_LABELS)
                    rects=try_open_labels(lb_file, sz)
                if ev.key == pygame.K_c: rects, rect = [], None
                if ev.key == pygame.K_n:
                    if sel_rect: sel_rect.selected=False
                    sel_rect=r=(sel_rect if sel_rect else rects[-1]).copy()
                    r.selected=True
                    r.x+=10; r.y+=10
                    rects.append(r)
                if ev.key == pygame.K_d:
                    if sel_rect: rects=[r for r in rects if r!=sel_rect]
                    else: rects = rects[:-1]
                    sel_rect = rect = None
                if ev.key == pygame.K_z: rects = mem_rects
                if ev.key == pygame.K_h: show_help=not show_help
                if ev.key == pygame.K_g: show_hint=not show_hint
                if ev.key == pygame.K_SPACE:
                    i = img_files.index(get_short_name(last_img_path))
                    if i<len(img_files)-1: open_img(file_path:=os.path.join(last_img_dir, img_files[i+1]))
                if ev.key == pygame.K_BACKSPACE:
                    i = img_files.index(get_short_name(last_img_path))
                    if i>0: open_img(file_path:=os.path.join(last_img_dir, img_files[i-1]))

        if last_key:
            if last_mod & pygame.KMOD_SHIFT:
                for r in rects: try_change_rect(r, last_key, pygame.KMOD_NONE)
            elif sel_rect: try_change_rect(sel_rect, last_key, last_mod)

        draw_all(screen)


if __name__ == "__main__": main()

