#!/usr/bin/env python

import math
import numpy as np

# world frame offset
world_offset = np.array([-37.0, -22.0, 0.0, 0.0])

def writeCircle(name, radius, center_point, heading, data_points, ccw):
     
     x = center_point[0]
     y = center_point[1]
     z = center_point[2]
     file = open(name, 'w')

     angle_discretization = (2*math.pi) / float(data_points) * float(ccw)
     angle = 0
     for val in range(data_points):
          z_computed = radius*math.cos(angle)
          y_computed = radius*math.sin(angle)
          angle +=angle_discretization
          # x = start_point_x + round(x, 6)
          y_computed = y + y_computed
          z_computed = z + z_computed
          if(x < 0.01 and x > -0.01):
               x = 0.0
          if(z_computed < 0.01 and z_computed > -0.01):
               z = 0.0
          if(y_computed < 0.01 and y_computed > -0.01):
               y_computed = 0.0 

          x_val_print = np.format_float_positional(x, trim='-') 
          y_val_print = np.format_float_positional(y_computed, trim='-') 
          z_val_print = np.format_float_positional(z_computed, trim='-') 
          heading_val_print = np.format_float_positional(heading, trim='-') 

          file.write(str(x_val_print)[0:5]+" "+str(y_val_print)[0:5]+" "+str(z_val_print)[0:5]+" "+str(heading_val_print)[0:5]+"\n")


     file.close()


def writeEight(name, radius, center_point, heading, data_points, ccw):

     x = center_point[0]
     y = center_point[1]
     z = center_point[2]
     file = open(name, 'w')

     angle_discretization = (2*math.pi) / float(data_points) * float(ccw)
     angle = -math.pi / 2
     for val in range(data_points):
          z_computed = radius*math.cos(angle)
          y_computed = radius*math.sin(angle)
          angle +=angle_discretization
          # x = start_point_x + round(x, 6)
          y_computed = y + y_computed + radius
          z_computed = z + z_computed
          if(x < 0.01 and x > -0.01):
               x = 0.0
          if(z_computed < 0.01 and z_computed > -0.01):
               z = 0.0
          if(y_computed < 0.01 and y_computed > -0.01):
               y_computed = 0.0 

          x_val_print = np.format_float_positional(x, trim='-') 
          y_val_print = np.format_float_positional(y_computed, trim='-') 
          z_val_print = np.format_float_positional(z_computed, trim='-') 
          heading_val_print = np.format_float_positional(heading, trim='-') 

          file.write(str(x_val_print)[0:5]+" "+str(y_val_print)[0:5]+" "+str(z_val_print)[0:5]+" "+str(heading_val_print)[0:5]+"\n")


     ## for the other direction 
     ccw = ccw * -1.0
     angle_discretization = (2*math.pi) / float(data_points) * float(ccw)
     angle = math.pi/2
     for val in range(data_points):
          z_computed = radius*math.cos(angle)
          y_computed = radius*math.sin(angle)
          angle +=angle_discretization
          # x = start_point_x + round(x, 6)
          y_computed = y + y_computed - radius 
          z_computed = z + z_computed
          if(x < 0.01 and x > -0.01):
               x = 0.0
          if(z_computed < 0.01 and z_computed > -0.01):
               z = 0.0
          if(y_computed < 0.01 and y_computed > -0.01):
               y_computed = 0.0 

          x_val_print = np.format_float_positional(x, trim='-') 
          y_val_print = np.format_float_positional(y_computed, trim='-') 
          z_val_print = np.format_float_positional(z_computed, trim='-') 
          heading_val_print = np.format_float_positional(heading, trim='-') 

          file.write(str(x_val_print)[0:5]+" "+str(y_val_print)[0:5]+" "+str(z_val_print)[0:5]+" "+str(heading_val_print)[0:5]+"\n")


     file.close()



def computeLine( start_point, end_point):
    if(end_point[0] - start_point[0] == 0):
        slope = 0.0
    else:
        slope = (end_point[1] - start_point[1]) / (end_point[0] - start_point[0])

    y_intercept = start_point[1] - (slope * start_point[0])
    return slope,y_intercept


def writeLine( name, start_point, end_point, line, number_data_pts, heading, x_rel_dist_change, angle_change):

     file = open(name, 'a')
     
     diff_y = end_point[1] - start_point[1]
     discretization =  float(diff_y) / float(number_data_pts)
     x_discretization = x_rel_dist_change / float(number_data_pts)

     if angle_change == True:
          discretization_angle = 2* math.pi / float(number_data_pts)
     else:
          discretization_angle = 0.0
     x_val = start_point[0]
     y_val = start_point[1] + discretization

     

     for val in range(number_data_pts):
          y_val = y_val + discretization
          z_val = line[0] * y_val + line[1]
          x_val = x_val + x_discretization


          if(x_val < 0.01 and x_val > -0.01):
               start_point[0] = 0.0
          if(z_val < 0.01 and z_val > -0.01):
               z = 0.0
          if(y_val < 0.01 and y_val > -0.01):
               y_val = 0.0 
          
          heading = heading + discretization_angle

          y_val_print = np.format_float_positional(y_val, trim='-') 
          x_val_print = np.format_float_positional(x_val, trim='-') 
          z_val_print = np.format_float_positional(z_val, trim='-') 
          heading_val_print = np.format_float_positional(heading, trim='-') 

          file.write(str(x_val_print)[0:5]+" "+str(y_val_print)[0:5]+" "+str(z_val_print)[0:5]+" "+str(heading_val_print)[0:5]+"\n")
     
     file.close()


def writeStar(name, start_point, second, third, fourth, fifth, heading, data_pts_per_line):

     line_1 = computeLine(start_point=[start_point[1],start_point[2]], end_point=[second[1],second[2]])
     line_2 = computeLine(start_point=[second[1],second[2]], end_point=[third[1],third[2]])
     line_3 = computeLine(start_point=[third[1],third[2]], end_point=[fourth[1],fourth[2]])
     line_4 = computeLine(start_point=[fourth[1],fourth[2]], end_point=[fifth[1],fifth[2]])
     line_5 = computeLine(start_point=[fifth[1],fifth[2]], end_point=[start_point[1],start_point[2]])

     file_name = name
     file = open(file_name, "w")

     number_data_pts = data_pts_per_line

     writeLine(file_name, start_point, second, line_1, number_data_pts, 0.0, 0.0, angle_change=True)
     writeLine(file_name, second, third, line_2, number_data_pts, 0.0, 0.0, angle_change=True)
     writeLine(file_name, third, fourth, line_3, number_data_pts, 0.0, 0.0, angle_change=True)
     writeLine(file_name, fourth, fifth, line_4, number_data_pts, 0.0, 0.0, angle_change=True)
     writeLine(file_name, fifth, start_point, line_5, number_data_pts, 0.0, 0.0, angle_change=True)

     file.close()

def main():

#### TX1 ####
     tx1_directory = "trajectory_files/tx1/"
# Line   
     line_tx1_name = tx1_directory + "line.txt" 
     file = open(line_tx1_name, 'w')

     tx1_start = np.array([4.0, 4.0, 3.0, 0.0]) + world_offset
     tx1_end   = np.array([5.0, -4.0, 5.0, 0.0]) + world_offset

     tx1_line = computeLine(start_point=[tx1_start[1], tx1_start[2]], end_point=[tx1_end[1], tx1_end[2]] )

     writeLine(name=line_tx1_name, start_point=tx1_start, end_point=tx1_end, line=tx1_line, number_data_pts=30, heading=tx1_start[3], x_rel_dist_change= 1.0, angle_change=False)
     writeLine(name=line_tx1_name, start_point=tx1_end, end_point=tx1_start, line=tx1_line, number_data_pts=30, heading=tx1_start[3], x_rel_dist_change= -1.0, angle_change=False)
     file.close()

## center point for circle motions 
     tx1_center_point = np.array([4.0, 0.0, 4.0, 0.0]) + world_offset
#### Circle 
     writeCircle(name= tx1_directory + "circle.txt" , radius=0.75, center_point=tx1_center_point, heading=tx1_center_point[3], data_points=60, ccw=1)
#### Eight
     writeEight(name= tx1_directory  + "eight.txt", radius=1.3, center_point=tx1_center_point, heading=tx1_center_point[3], data_points=50, ccw=1)
### Star
     tx1_start_point = np.array([4.0, 0.0, 6.0, 0.0] ) + world_offset
     tx1_left_down = np.array([4.0, 2.0, 3.0, 0.0] )+ world_offset
     tx1_right_up = np.array([4.0, -2.5, 5.5, 0.0]) + world_offset
     tx1_left_up = np.array([4.0, 2.5, 5.5, 0.0] )+ world_offset
     tx1_right_down = np.array([4.0, -2.0, 3.0, 0.0]) + world_offset
     writeStar(name=tx1_directory + "star.txt",start_point=tx1_start_point,second=tx1_left_down,third=tx1_right_up,fourth=tx1_left_up,fifth=tx1_right_down, heading=0.0, data_pts_per_line=15)
#
# TX2 #
     tx2_directory = "trajectory_files/tx2/"
#### Line   
     line_tx2_name = tx2_directory + "line.txt" 
     file = open(line_tx2_name, 'w')

     tx2_start = np.array([8.0, 4.0, 4.0, 0.0 ]) + world_offset
     tx2_end =   np.array([9.0, -4.0, 6.0, 0.0 ]) + world_offset

     tx2_line = computeLine(start_point=[tx2_start[1], tx2_start[2]], end_point=[tx2_end[1], tx2_end[2]] )

     writeLine(name=line_tx2_name, start_point=tx2_start, end_point=tx2_end, line=tx2_line, number_data_pts=30, heading=tx2_start[3], x_rel_dist_change=1.0, angle_change=False)
     writeLine(name=line_tx2_name, start_point=tx2_end, end_point=tx2_start, line=tx2_line, number_data_pts=30, heading=tx2_start[3], x_rel_dist_change=-1.0, angle_change=False)
     file.close()

## center point for circle motions 
     tx2_center_point = np.array([8.0, 0.0, 4.0, 0.0]) + world_offset
#### Circle 
     writeCircle(name= tx2_directory + "circle.txt" , radius=1.6, center_point=tx2_center_point, heading=tx2_start[3], data_points=30, ccw=1)
#### Eight
     writeEight(name= tx2_directory + "eight.txt", radius=1.6, center_point=tx2_center_point, heading=tx2_start[3], data_points=50, ccw=1)
#### Star 
     tx2_start_point = np.array([8.0, 0.0, 6.0, 0.0] ) + world_offset
     tx2_left_down = np.array([8.0, 2.0, 3.0, 0.0] )+ world_offset
     tx2_right_up = np.array([8.0, -2.5, 5.5, 0.0]) + world_offset
     tx2_left_up = np.array([8.0, 2.5, 5.5, 0.0] )+ world_offset
     tx2_right_down = np.array([8.0, -2.0, 3.0, 0.0]) + world_offset
     writeStar(name=tx2_directory + "star.txt",start_point=tx2_start_point,second=tx2_left_down,third=tx2_right_up,fourth=tx2_left_up,fifth=tx2_right_down, heading=0.0, data_pts_per_line=15)

# RX #
     rx_directory = "trajectory_files/rx/"
#### Horizontal Line
     line_rx_name = rx_directory + "horizontal_line.txt" 
     file = open(line_rx_name, 'w')

     rx_start = np.array([1.5, 4.0, 4.0, 1.223] ) + world_offset
     rx_end =   np.array([-1.5, -4.0, 6.0, 1.223])  + world_offset

     rx_line = computeLine(start_point=[rx_start[1], rx_start[2]], end_point=[rx_end[1], rx_end[2]] )

     writeLine(name=line_rx_name, start_point=rx_start, end_point=rx_end, line=rx_line, number_data_pts=30, heading=rx_start[3], x_rel_dist_change=-3, angle_change=False)
     writeLine(name=line_rx_name, start_point=rx_end, end_point=rx_start, line=rx_line, number_data_pts=30, heading=rx_start[3], x_rel_dist_change=3, angle_change=False)
     file.close()

#### Pitch Line
     pitch_line_rx_name = rx_directory + "pitch_line.txt" 
     file = open(pitch_line_rx_name, 'w')

     pitch_rx_start = np.array([0.0, 0.0, 4.0, 1.223] ) + world_offset
     pitch_rx_end =   np.array([-4.0, 0.0, 4.0, 1.223])  + world_offset

     # stupid hack to overcome line computation 
     rx_pitch_line = [0.0, pitch_rx_start[2]]

     writeLine(name=pitch_line_rx_name, start_point=pitch_rx_start, end_point=pitch_rx_end, line=rx_pitch_line, number_data_pts=20, heading=pitch_rx_start[3], x_rel_dist_change=-4.0, angle_change=False)
     writeLine(name=pitch_line_rx_name, start_point=pitch_rx_end, end_point=pitch_rx_start, line=rx_pitch_line, number_data_pts=20, heading=pitch_rx_start[3], x_rel_dist_change=4.0, angle_change=False)
     file.close()

### RX Rotation 
     rx_rotation_name = rx_directory + "rotation.txt" 
     file = open(rx_rotation_name, 'w')

     rx_start_rot = np.array([0.0, 0.0, 4.0, 0.0] ) + world_offset
     rx_line_rot = computeLine(start_point=[rx_start_rot[1], rx_start_rot[2]], end_point=[rx_start_rot[1], rx_start_rot[2]] )
     
     writeLine(name=rx_rotation_name, start_point=rx_start_rot, end_point=rx_start_rot, line=rx_line_rot, number_data_pts=8, heading=rx_start_rot[3], x_rel_dist_change=0.0, angle_change=True)

###
     rx_hover_name = rx_directory + "hover.txt" 
     file = open(rx_hover_name, 'w')
     z_height = world_offset[2] + 4.0 # meter
     file.write(str(world_offset[0])[0:5]+" "+str(world_offset[1])[0:5]+" "+str(z_height)[0:5]+" "+str(world_offset[3])[0:5])
     file.close()

if __name__ == "__main__":
    main()