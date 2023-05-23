from tkinter import *

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
    code = g00_z(5)
    code += g00_xy(0,0)
    return code

def g02_rxy(r, x, y, depth, speedxy, speedz):
    code = g00_xy(x-r, y)
    code += g01_z(-depth, speedz)
    code += 'G02 X%4.4f Y%4.4f I%4.4f J%4.4f F%4.4f\n' % (x-r, y, r, 0.0, speedxy)
    return code

def g_drill(x, y, depth, speedz):
    code = g00_xy(x, y)
    code += g01_z(-depth, speedz)
    return code

def g_drill_points(point_list, depth, speedz, upz):
    code = g00_z(upz)
    for p in point_list:
        code += g_drill(p[0], p[1], depth, speedz)
        code += g00_z(upz)
    return code

def g_drill_nema17(x, y, depth, speedz, upz):
    points = [[x-31/2, y-31/2],
              [x+31/2, y-31/2],
              [x-31/2, y+31/2],
              [x+31/2, y+31/2]]
    return g_drill_points(points, depth, speedz, upz)

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

if __name__ == "__main__":
    tool_r = 3
    tool_sxy = 100
    tool_sz = 50

    sheet_upz = 1
    sheet_x = 42
    sheet_y = 50
    sheet_th = 4
    
    code = g_start()
    # nema17
    code += g_drill_nema17(sheet_x/2, sheet_y - 21, sheet_th, tool_sz, sheet_upz)

    # hex mount
    code += g_hex(sheet_x/2 - 17, sheet_y - 21, 8 - tool_r, 2.5, tool_sxy, tool_sz, sheet_upz)
    code += g_hex(sheet_x/2 + 17, sheet_y - 21, 8 - tool_r, 2.5, tool_sxy, tool_sz, sheet_upz)

    code += g_home()

    f = open('task.gcode', 'wb')
    f.write(bytes(code, encoding='UTF-8'))
    f.close()
    
    
