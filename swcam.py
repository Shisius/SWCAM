from tkinter import *
import numpy as np

def g_start():
    return 'G17 G21 G90 G94 G54\n'

def g01_xy(x,y,speed):
    return 'G01 X%4.4f Y%4.4f F%4.4f\n' % (x, y, speed)

def g01_list(point_list, speed):
    code = ''
    for p in point_list:
        code += g01_xy(p[0], p[1], speed)
    return code

def g00_z(z):
    code = 'G00 Z%4.4f\n' % z
    return code

def g01_z(z, speed):
    code = 'G01 Z%4.4f F%4.4f\n' % (z,speed)
    return code

def g00_xy(x,y):
    code = 'G00 X%4.4f Y%4.4f\n' % (x,y)
    return code

def g_home():
    code = g00_z(10)
    code += g00_xy(0,0)
    return code

def g02_rxy(r, x, y, depth, speedxy, speedz):
    code = g00_xy(x-r, y)
    code += g01_z(-depth, speedz)
    code += 'G02 X%4.4f Y%4.4f I%4.4f J%4.4f F%4.4f\n' % (x-r, y, r, 0.0, speedxy)
    return code

def g02_rxyn(r, x, y, depth, speedxy, speedz, n):
    code = g00_xy(x-r, y)
    cur_depth = 0
    for i in range(n):
        cur_depth += depth
        code += g01_z(-cur_depth, speedz)
        code += 'G02 X%4.4f Y%4.4f I%4.4f J%4.4f F%4.4f\n' % (x-r, y, r, 0.0, speedxy)
    return code

def g_cwbow_rpn(r, x1, y1, x2, y2):
    code = g00_xy(x1, y1)
    return code

def g_drill(x, y, depth, speedz):
    code = g00_xy(x, y)
    code += g01_z(-depth, speedz)
    code += g01_z(0, speedz)
    return code

def g_drill_n(x, y, depth, speedz, stepz, dofast=False):
    code = g00_xy(x, y)
    d = 0
    while d<depth:
        if dofast:
            code += g00_z(-d + 0.1)
        else:
            code += g00_z(0.1)
        d+=stepz
        code += g01_z(-d, speedz)
    code += g00_z(0.1)
    return code

def g_drill_points(point_list, depth, speedz, upz, stepz, dofast=False):
    code = g00_z(upz)
    for p in point_list:
        code += g_drill_n(p[0], p[1], depth, speedz, stepz, dofast)
        code += g00_z(upz)
    return code

def g_drill_quadro(x, y, a, depth, speedz, upz, stepz):
    points = [[x-a/2, y-a/2],
              [x+a/2, y-a/2],
              [x-a/2, y+a/2],
              [x+a/2, y+a/2]]
    return g_drill_points(points, depth, speedz, upz, stepz)

def g_drill_4_rect(x, y, ax, ay, depth, speedz, upz, stepz):
    points = [[x-ax/2, y-ay/2],
              [x+ax/2, y-ay/2],
              [x-ax/2, y+ay/2],
              [x+ax/2, y+ay/2]]
    return g_drill_points(points, depth, speedz, upz, stepz)

def g_drill_nema17(x, y, depth, speedz, upz):
    return g_drill_quadro(x, y, 31, depth, speedz, upz, depth)

def hex_points(x, y, w):
    l = w / 3**0.5
    p = [[x-l/2, y-w/2],
         [x+l/2, y-w/2],
         [x+l, y],
         [x+l/2, y+w/2],
         [x-l/2, y+w/2],
         [x-l,y]]
    return p

def g_hex(x, y, w, depth, speedxy, speedz, upz):
    points = hex_points(x, y, w)
    code = g00_z(upz)
    code += g00_xy(points[0][0], points[0][1])
    code += g01_z(-depth, speedz)
    code += g01_list(points, speedxy)
    code += g00_z(upz)
    return code
    

class CamObject:

    def __init__(self, cvs, cntr):
        self.cvs = cvs
        self.cntr = cntr


class Circle(CamObject):

    def __init__(self):
        pass

class CamLine:

    def __init__(self, start, end):
        if len(start) > 2:
            self.start = np.array(start)
        else:
            self.start = np.array([start[0], start[1], 0])
        if len(end) > 2:
            self.end = np.array(end)
        else:
            self.end = np.array([end[0], end[1], 0])
        self.canvas_id = 0

    def draw_xy(self, canvas):
        self.canvas_id = canvas.create_line(self.start[0], canvas.winfo_height() - self.start[1], self.end[0], canvas.winfo_height() - self.end[1])

    def mill(self, place, settings):
        pass

    def move(self, pos):
        if len(pos) == 2:
            pos = [pos[0], pos[1], 0]
        self.start += pos
        self.end += pos

    def rotate(self, angle):
        pass

    def mirror(self, dim):
        self.start[dim] *= -1
        self.end[dim] *= -1

    def todxf(self, dxf_doc):
        dxf_doc.add_line((self.start[0],self.start[1]), (self.end[0], self.end[1]))


if __name__ == "__main__":
    tool_d = 4
    tool_sxy = 50
    tool_sz = 20

    sheet_upz = 5
    sheet_x = 46
    sheet_y = 27 + 21 + 9
    sheet_th = 3
    
    code = g_start()
    
    code += g_home()

    f = open('example.gcode', 'wb')
    f.write(bytes(code, encoding='UTF-8'))
    f.close()
    
    
