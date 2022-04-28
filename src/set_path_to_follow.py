#!/usr/bin/env python3

import rospy
import traceback 
import numpy as np
from mobrob_util.msg import ME439PathSpecs
from mobrob_util.msg import ME439Color
from std_msgs.msg import Bool
import me439_mobile_robot_class_v02 as m439rbt

WHEEL_WIDTH = rospy.get_param('/wheel_width_model')
BODY_LENGTH = rospy.get_param('/body_length')
WHEEL_DIAMETER = rospy.get_param('/wheel_diameter_model')
WHEEL_RADIUS = WHEEL_DIAMETER/2.0
ARENA_SIZE_X = 1.0
ARENA_SIZE_Y = 1.0
PATH_FILE_SVG = '/home/pi/catkin_ws/src/moblightdraw/src/SVGtest2.svg'


def convert_svg_to_path_specs(svg_file, xlength=ARENA_SIZE_X, ylength=ARENA_SIZE_Y):
    svg_coords, svg_colors = parse_svg_for_paths(svg_file)
    scaled_coords = scale_coords_to_arena(svg_coords, xlength, ylength)
    path_specs = convert_coords_to_path_specs(scaled_coords)
    
    return path_specs, svg_colors

def parse_svg_for_paths(svg_file):
    all_coords = []
    all_colors = []
    with open(svg_file,'r') as svgfile:
        for line in svgfile:
            l = line.strip()
            if(l[0:3] == 'd="'):
                coords = l[4:-1].split(" ")
                coords = [float(x) for x in coords][:2]
                all_coords.append(coords)
            elif(l[0:12] == 'stroke="rgb('):
                color = l[12:-2].split(" ")
                color = [int(x.strip(",")) for x in color]
                all_colors.append(color)
                
    return(np.array(all_coords), np.array(all_colors))

def scale_coords_to_arena(coords, dx, dy):
    xsvg = coords[:,0]
    ysvg = coords[:,1]  # Note that Y is Down in SVG!
    
    xmin = np.min(xsvg)
    xmax = np.max(xsvg)
    xrng = xmax - xmin
    xctr = xmin + xrng/2
    ymin = np.min(ysvg)
    ymax = np.max(ysvg)
    yrng = ymax - ymin
    yctr = ymin + yrng/2

    # Shift and scale. 
    k = min([dx/xrng, dy/yrng])
    yscaled = -1*(ysvg-ymax)*k   # Note that Y is Down in SVG!
    xscaled = (xsvg-xmin)*k
    
    scaled_coords = np.vstack((xscaled,yscaled)).transpose()
    
    return scaled_coords

def convert_coords_to_path_specs(coords):
## path_specs are:  [Xorigin, Yorigin, Theta_init, Rsigned, Length]

    coords = np.append([[0.,0.]],coords,axis=0)  # start at 0
    coords = np.append(coords,[[0.,0.]],axis=0)  # end at 0
    
    path_specs = np.ndarray((0,5)).astype(float)
    for ii in range(1,len(coords)):
        xorigin = coords[ii-1][0]
        yorigin = coords[ii-1][1]
        displacement = coords[ii]-coords[ii-1]
        angle = np.arctan2(-displacement[0], displacement[1])   # Remember the angle is measured from the +y axis. 
        rsigned = np.inf
        length = np.linalg.norm(displacement)
        
        path_specs = np.append(path_specs, [[xorigin, yorigin, angle, rsigned, length]], axis=0)
    
    return path_specs

def talker(): 
    global path_specs, segment_number, path_complete, colors

    rospy.init_node('set_path_to_follow', anonymous=False)

    pub_segment_specs = rospy.Publisher('/path_segment_spec', ME439PathSpecs, queue_size=1)

    pub_colors = rospy.Publisher('/led_color', ME439Color, queue_size=1)

    pub_path_complete = rospy.Publisher('/path_complete', Bool, queue_size=1)

    sub_complete = rospy.Subscriber('/segment_complete', Bool, increment_segment)

    path_segment_spec = ME439PathSpecs()

    color_spec = ME439Color()
   
    r = rospy.Rate(10) # N Hz
    try: 
        while not rospy.is_shutdown():
            color_spec.red = path_specs[segment_number, 0]
            color_spec.green = path_specs[segment_number, 1]
            color_spec.blue = path_specs[segment_number, 2]
            pub_colors.publish(color_spec)

            path_segment_spec.x0 = path_specs[segment_number,0]
            path_segment_spec.y0 = path_specs[segment_number,1]
            path_segment_spec.theta0 = path_specs[segment_number,2]
            path_segment_spec.Radius = path_specs[segment_number,3]
            path_segment_spec.Length = path_specs[segment_number,4]
            pub_segment_specs.publish(path_segment_spec) 
            
            pub_path_complete.publish(path_complete)
            if path_complete.data:
                return
            
            r.sleep()

    except Exception:
        traceback.print_exc()
        pass
        
def increment_segment(msg_in):
    global segment_number, path_complete

    segment_number = segment_number + 1
    
    # If that was the last segment (new segment number exceeds the indices 
    # available), decrement so as not to confuse the system, and tell the 
    # downstream programs that the path is complete 
    if segment_number >= path_specs.shape[0]:
        path_complete.data = True
        segment_number = segment_number - 1
    else: 
        path_complete.data = False
    

robot = m439rbt.robot(WHEEL_WIDTH, BODY_LENGTH, WHEEL_RADIUS)

path_specs, colors = convert_svg_to_path_specs(PATH_FILE_SVG, xlength=1., ylength=1.)

# fix the PATH_SPECS to drive a line from the initial position to start of path
if not all( path_specs[0,0:2] == robot.r_center_world) :
    path_specs = np.append(np.array([robot.specify_line(robot.r_center_world[0], robot.r_center_world[1],path_specs[0,0],path_specs[0,1])]),path_specs,axis=0)

segment_number = 0
path_complete = Bool()
    
if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
