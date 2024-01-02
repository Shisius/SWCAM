from tkinter import *
import numpy as np
import math

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

def g02_rxys(r, x, y, depth, speedxy, speedz, stepz):
    code = g00_xy(x-r, y)
    cur_depth = 0
    while cur_depth < depth:
        cur_depth += stepz
        if cur_depth > depth:
            cur_depth = depth
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
        if d > depth:
            d = depth
        code += g01_z(-d, speedz)
    code += g00_z(0.1)
    return code

def g_drill_points(point_list, depth, speedz, upz, stepz, dofast=False):
    code = g00_z(upz)
    for p in point_list:
        code += g_drill_n(p[0], p[1], depth, speedz, stepz, dofast)
        code += g00_z(upz)
    return code

def g_drill_quadro(x, y, a, depth, speedz, upz, stepz, dofast=False):
    points = [[x-a/2, y-a/2],
              [x+a/2, y-a/2],
              [x-a/2, y+a/2],
              [x+a/2, y+a/2]]
    return g_drill_points(points, depth, speedz, upz, stepz, dofast)

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

def swcam_az(start, end):
    return math.atan2(end[0] - start[0], end[1] - start[1])

def swcam_add_az(az1, az2):
    naz = az1 + az2
    naz -= 2*math.pi * (naz // (2*math.pi))
    return naz
    
def swcam_dazcw(az1,az2):
    if az2 >= az1:
        return az2-az1
    else:
        return 2*math.pi + az2 - az1

def swcam_dazccw(az1,az2):
    return swcam_dazcw(az2, az1)

def swcam_rotate_z(vec, az):
    newv = vec[:]
    newv[0] = vec[0] * math.cos(az) + vec[1] * math.sin(az)
    newv[1] = vec[0] * (-1) * math.sin(az) + vec[1] * math.cos(az)
    return newv

# Cutdepth should be 0.15*d for d < 3 or 0.3*d for d > 3
CAM_TOOL_DEFAULT = {"diameter": 1.0,
                    "feedrate": 1.0,
                    "drillrate": 1.0,
                    "cutdepth": 0.15,
                    "safez": 10.0,
                    "overlap": 50.0}

CAM_MILL_COLOR = '#AA0000'

class CamObject:

    def __init__(self):
        self.canvas_id = None
        self.mill_ids = []

    def clear_mill(self, canvas):
        for _id in self.mill_ids:
            canvas.delete(_id)

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
        self.canvas_id = None
        self.az = swcam_az(self.start, self.end)
        self.mstart = self.start
        self.mend = self.end
        self.mill_ids = []

    def draw_xy(self, canvas):
        if not (self.canvas_id is None):
            canvas.delete(self.canvas_id)
        self.canvas_id = canvas.create_line(self.start[0], canvas.winfo_height() - self.start[1], self.end[0], canvas.winfo_height() - self.end[1])

    def draw_mill(self, canvas):
        self.mill_ids += [canvas.create_line(self.mstart[0], canvas.winfo_height() - self.mstart[1], self.mend[0], canvas.winfo_height() - self.mend[1], 
                          outline=CAM_MILL_COLOR)]

    def clear_mill(self, canvas):
        for _id in self.mill_ids:
            canvas.delete(_id)

    # Place - distance to the instrument rotation axis. Positive - rightside, Negative - leftside. 
    # Prev_az - azimuth of the previous vector
    # Next_az - azimuth of the next vector
    # Returns (start, end) mill points
    def mill(self, place, prev_az, next_az):
        self.az = swcam_az(self.start, self.end)
        # Start
        self.mstart = self.start
        if place == 0:
            return (self.start, self.end)
        if place > 0:
            daz_start = swcam_dazcw(self.az, swcam_add_az(prev_az, math.pi))
        else:
            daz_start = -1 * swcam_dazccw(self.az, swcam_add_az(prev_az, math.pi))
        d_vec = np.array([0, place, 0]) / abs(math.sin(daz_start/2))
        d_vec = swcam_rotate_z(d_vec, self.az + daz_start/2)
        self.mstart = self.start + d_vec
        # End
        if place > 0:
            daz_end = swcam_dazcw(next_az, swcam_add_az(self.az, math.pi))
        else:
            daz_end = -1 * swcam_dazccw(next_az, swcam_add_az(self.az, math.pi))
        d_vec = np.array([0, place, 0]) / abs(math.sin(daz_end/2))
        d_vec = swcam_rotate_z(d_vec, next_az + daz_start/2)
        self.mend = self.end + d_vec
        return (self.mstart, self.mend)

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

# Angle > 0 => CW, < 0 => CCW
# Problem: angles > 180 for center mode
class CamArc:

    def __init__(self, start, end, center = None, angle = 0, cw = True):
        if len(start) > 2:
            self.start = np.array(start)
        else:
            self.start = np.array([start[0], start[1], 0])
        if len(end) > 2:
            self.end = np.array(end)
        else:
            self.end = np.array([end[0], end[1], 0])
        self.canvas_id = None
        self.mill_ids = []
        self.angle = angle
        self.radius = 0;
        self.cw = cw
        #self.center = center
        if center is None:
            if self.angle < 0:
                self.cw = False
            self.radius = sum((self.end - self.start)**2)**0.5 / (2 * math.sin(abs(self.angle)/2))
            az_rot_z = swcam_az(self.start, self.end) + np.sign(self.angle)*(math.pi - abs(self.angle)) / 2
            self.center = self.start + swcam_rotate_z([0, self.radius, 0], az_rot_z)
        else:
            if (len(center) > 2):
                self.center = np.array(center)
            else:
                self.center = np.array([center[0], center[1], 0])
            self.radius = sum((self.center-self.start)**2)**0.5
            self.angle = math.acos( (sum((self.end-self.start)**2)/(2*self.radius**2)) - 1 )
            if not self.cw:
                self.angle *= -1
        self.az = [swcam_az(self.start, self.end) - self.angle/2, swcam_az(self.start, self.end) + self.angle/2]
        self.mstart = self.start
        self.mend = self.end
        self.mangle = self.angle
        self.mrad = self.radius

    def draw_xy(self, canvas):
        if not (self.canvas_id is None):
            canvas.delete(self.canvas_id)
        self.canvas_id = canvas.create_arc(self.center[0] - self.radius, canvas.winfo_height() - (self.center[1] - self.radius), 
                                           self.center[0] + self.radius, canvas.winfo_height() - (self.center[1] + self.radius), 
                                           start=180 - self.az[1]*180/math.pi, extent=self.angle*180/math.pi, style=ARC)

    def clear_mill(self, canvas):
        for _id in self.mill_ids:
            canvas.delete(_id)

    def mill(self, place, prev_az, next_az):
        # if mill radius <= 0 - G01 at the same point
        mill_r = self.radius - place
        if mill_r <= 0:
            return (self.center, self.center, self.center)
        mstart = self.start
        mend = self.end

        return (mstart, mend, self.center)

    def move(self, pos):
        if len(pos) == 2:
            pos = [pos[0], pos[1], 0]
        self.start += pos
        self.end += pos
        self.center += pos

class CamProfile:

    def __init__(self):
        self.draw_lines = []
        self.mill_lines = []
        self.gcode = ''

    def draw_xy(self, canvas):
        for l in self.lines:
            l.draw_xy(canvas)

    def mill(self, place):
        pass

class CamDrill:

    def __init__(self, x, y, d = 1):
        self.pos = np.array([x,y,0])
        self.diam = d
        self.canvas_id = None
        self.gcode = ''

    def draw_xy(self, canvas):
        if not (self.canvas_id is None):
            canvas.delete(self.canvas_id)
        self.canvas_id = canvas.create_oval(self.diam, self.diam, self.pos[0] - self.diam/2, canvas.winfo_height() - self.pos[1] + self.diam/2)

    def mill(self, depth, tool):
        self.gcode = g00_z(tool["safez"])
        self.gcode += g_drill_n(self.pos[0], self.pos[1], depth, tool["drillrate"], tool["cutdepth"], dofast=True)
        self.gcode += g00_z(tool["safez"])

    def move(self, pos):
        if len(pos) == 2:
            pos = [pos[0], pos[1], 0]
        self.pos += pos

# Problem: place < 0
class CamCircle:

    def __init__(self, x, y, d):
        self.pos = np.array([x,y,0])
        self.diam = d
        self.canvas_id = None
        self.gcode = ''
        self.mill_r = [d/2]
        self.mill_ids = []

    def draw_xy(self, canvas):
        if not (self.canvas_id is None):
            canvas.delete(self.canvas_id)
        self.canvas_id = canvas.create_oval(self.pos[0] - self.diam/2, canvas.winfo_height() - (self.pos[1] - self.diam/2),
                                            self.pos[0] + self.diam/2, canvas.winfo_height() - (self.pos[1] + self.diam/2))

    def draw_mill(self, canvas):
        self.clear_mill(canvas)
        for mr in self.mill_r:
            self.mill_ids += [canvas.create_oval(self.pos[0] - mr, canvas.winfo_height() - (self.pos[1] - mr), 
                                                 self.pos[0] + mr, canvas.winfo_height() - (self.pos[1] + mr),
                                                 outline=CAM_MILL_COLOR)]

    def clear_mill(self, canvas):
        for _id in self.mill_ids:
            canvas.delete(_id)

    # depth - cutting depth or list[start_depth, fin_depth]. Depth = -1 * z_position.
    def mill(self, depth, place, tool):
        start_depth = 0
        end_depth = 0
        if hasattr(depth, '__len__'):
            if (len(depth) > 1):
                start_depth = depth[0]
                end_depth = depth[1]
            else:
                end_depth = depth[0]
        else:
            end_depth = depth
        # cut_depth = end_depth - start_depth
        mill_cut = tool["feedrate"]
        self.mill_r = [] #self.diam/2
        self.gcode = g00_z(tool["safez"])
        if place > 0:
            self.mill_r -= tool['diameter']/2
            if abs(place) > tool['diameter']/2:
                self.mill_r -= place - tool['diameter']/2
            mill_cut *= (self.diam - tool['diameter'])/self.diam
            self.gcode += g02_rxys(self.mill_r, self.pos[0], self.pos[1], depth, tool["feedrate"], tool["drillrate"], tool["cutdepth"])
        elif place < 0: 
            self.mill_r += [self.diam/2 + tool['diameter']/2]
        else:
            self.mill_r = [self.diam/2]
            mill_cut *= (self.diam - tool['diameter'])/self.diam
            self.gcode += g00_xy(self.pos[0]-self.mill_r[0], self.pos[1])
            self.gcode += g00_z(start_depth)
            self.gcode += g02_rxys(self.mill_r[0], self.pos[0], self.pos[1], end_depth, tool["feedrate"], tool["drillrate"], tool["cutdepth"])
        self.gcode += g00_z(tool["safez"])

    def move(self, pos):
        if len(pos) == 2:
            pos = [pos[0], pos[1], 0]
        self.pos += pos

class CamTask:

    def __init__(self):
        self.profiles = []
        self.tool = CAM_TOOL_DEFAULT

    def set_tool(self):
        pass

    def add_drill(self, x, y, depth):
        pass

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

    #f = open('example.gcode', 'wb')
    #f.write(bytes(code, encoding='UTF-8'))
    #f.close()
    
    
