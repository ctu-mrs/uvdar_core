import math

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

          file.write(str(x)[0:5] +" "+str(y_computed)[0:5]+" "+str(z_computed)[0:5]+" "+str(heading)[0:5]+"\n")

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

          file.write(str(x)[0:5] +" "+str(y_computed)[0:5]+" "+str(z_computed)[0:5]+" "+str(heading)[0:5]+"\n")

     
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

          file.write(str(x)[0:5] +" "+str(y_computed)[0:5]+" "+str(z_computed)[0:5]+" "+str(heading)[0:5]+"\n")


     file.close()



def computeLine( start_point, end_point):
    if(end_point[0] - start_point[0] == 0):
        slope = 0.0
    else:
        slope = (end_point[1] - start_point[1]) / (end_point[0] - start_point[0])

    y_intercept = start_point[1] - (slope * start_point[0])
    return slope,y_intercept


def writeLine( name, start_point, end_point, line, number_data_pts, heading, x_rel_dist_change):

     file = open(name, 'a')
     
     diff_y = end_point[1] - start_point[1]
     discretization =  float(diff_y) / float(number_data_pts)
     x_discretization = x_rel_dist_change / float(number_data_pts)
     discretization_angle = 0.0 #1.57/ float(number_data_pts)
     x_val = start_point[0]
     x_val = start_point[0] + x_discretization
     y_val = start_point[1] + discretization


     for val in range(number_data_pts):
          y_val = y_val + discretization
          z = line[0] * y_val + line[1]
          x_val = x_val + x_discretization
          

          if(x_val < 0.001 and x_val > -0.001):
               start_point[0] = 0.0
          if(z < 0.001 and z > -0.001):
               z = 0.0
          if(y_val < 0.001 and y_val > -0.001):
               y_val = 0.0 
          
          heading = heading - discretization_angle

          file.write(str(x_val)[0:5]+" "+str(y_val)[0:5]+" "+str(z)[0:5]+" "+str(heading)[0:5]+"\n")
     
     file.close()


def compute_star(name, start_point, second, third, fourth, fifth, heading, data_pts_per_line):
     line_1 = computeLine(start_point[1], start_point[2], second[1], second[2])
     line_2 = computeLine(second[1], second[2], third[1], third[2])
     line_3 = computeLine(third[1], third[2], fourth[1], fourth[2])
     line_4 = computeLine(fourth[1], fourth[2], fifth[1], fifth[2])
     line_5 = computeLine(fifth[1], fifth[2], start_point[1], start_point[2])


     file_name = name + "_" + str(start_point[0]) + "_" + str(start_point[1]) + "_" + str(start_point[2]) + ".txt"  
     file = open(file_name, "w")
     file.close()

     number_data_pts = data_pts_per_line

     writeLine(file_name, start_point, second, line_1, number_data_pts, 0.0)
     writeLine(file_name, second, third, line_2, number_data_pts, 0.0)
     writeLine(file_name, third, fourth, line_3, number_data_pts, 0.0)
     writeLine(file_name, fourth, fifth, line_4, number_data_pts, 0.0)
     writeLine(file_name, fifth, start_point, line_5, number_data_pts, 0.0)

def compute_star2(name, start_point, second, third, fourth, fifth, heading, data_pts_per_line):
     line_1 = computeLine(start_point[1], start_point[2], second[1], second[2])
     line_2 = computeLine(second[1], second[2], third[1], third[2])
     line_3 = computeLine(third[1], third[2], fourth[1], fourth[2])
     line_4 = computeLine(fourth[1], fourth[2], fifth[1], fifth[2])
     line_5 = computeLine(fifth[1], fifth[2], start_point[1], start_point[2])


     file_name = name + "_" + str(start_point[0]) + "_" + str(start_point[1]) + "_" + str(start_point[2]) + ".txt"  
     file = open(file_name, "w")
     file.close()

     number_data_pts = data_pts_per_line

     writeLine2(file_name, start_point, second, line_1, number_data_pts, 0.0)
     writeLine2(file_name, second, third, line_2, number_data_pts, 0.0)
     writeLine2(file_name, third, fourth, line_3, number_data_pts, 0.0)
     writeLine2(file_name, fourth, fifth, line_4, number_data_pts, 0.0)
     writeLine2(file_name, fifth, start_point, line_5, number_data_pts, 0.0)
     


def main():

#### TX1 ####
     tx1_directory = "trajectory_files/tx1/"
# Line   
     line_tx1_name = tx1_directory + "line.txt" 
     file = open(line_tx1_name, 'w')

     tx1_start = [4.0, 4.0, 4.0]
     tx1_end =   [5.0, -4.0, 6.0]

     tx1_line = computeLine(start_point=[tx1_start[1], tx1_start[2]], end_point=[tx1_end[1], tx1_end[2]] )

     writeLine(name=line_tx1_name, start_point=tx1_start, end_point=tx1_end, line=tx1_line, number_data_pts=30, heading=0.0, x_rel_dist_change= 1.0)
     writeLine(name=line_tx1_name, start_point=tx1_end, end_point=tx1_start, line=tx1_line, number_data_pts=30, heading=0.0, x_rel_dist_change= -1.0)
## center point for circle motions 
     tx1_center_point = [4.0, 0.0, 4.0]
#### Circle 
     writeCircle(name= tx1_directory + "circle.txt" , radius=1.3, center_point=tx1_center_point, heading=0.0, data_points=60, ccw=1)
#### Eight
     writeEight(name= tx1_directory  + "eight.txt", radius=1.3, center_point=tx1_center_point, heading=0.0, data_points=50, ccw=1)


# TX2 #
     tx2_directory = "trajectory_files/tx2/"
#### Line   
     line_tx2_name = tx2_directory + "line.txt" 
     file = open(line_tx2_name, 'w')

     tx2_start = [8.0, 4.0, 4.0]
     tx2_end =   [9.0, -4.0, 6.0]

     tx2_line = computeLine(start_point=[tx2_start[1], tx2_start[2]], end_point=[tx2_end[1], tx2_end[2]] )

     writeLine(name=line_tx2_name, start_point=tx2_start, end_point=tx2_end, line=tx2_line, number_data_pts=30, heading=0.0, x_rel_dist_change=1.0)
     writeLine(name=line_tx2_name, start_point=tx2_end, end_point=tx2_start, line=tx2_line, number_data_pts=30, heading=0.0, x_rel_dist_change=-1.0)

## center point for circle motions 
     tx2_center_point = [8.0, 0.0, 4.0]
#### Circle 
     writeCircle(name= tx2_directory + "circle.txt" , radius=1.6, center_point=tx2_center_point, heading=0.0, data_points=60, ccw=1)
#### Eight
     writeEight(name= tx2_directory + "eight.txt", radius=1.6, center_point=tx2_center_point, heading=0.0, data_points=50, ccw=1)


     # writeEight(name="trajectory_files/tx2/tx2", radius=1.3, x=8.0, y=0.0, z=4.0, heading=0.0, data_points=50, ccw=-1)
     # name="trajectory_files/"
     # start_point = [-29.0, -22.0, 6.0]
     # left_down = [-29.0, -20.0, 3.0]
     # right_up = [-29.0, -24.5, 5.5]
     # left_up = [-29.0, -20.5, 5.5]
     # right_down = [-29.0, -24.0, 3.0]
     # compute_star(name=name, start_point=right_up, second=left_up, third=right_down, fourth=start_point, fifth=left_down, heading=0.0, data_pts_per_line=20)

     # start_point = [-31.0, -22.0, 6.0]
     # left_down = [-33.0, -20.0, 3.0]
     # right_up = [-31.0, -24.5, 5.5]
     # left_up = [-31.0, -20.5, 5.5]
     # right_down = [-33.0, -24.0, 3.0]
     # compute_star2(name, start_point, left_down, right_up, left_up, right_down, heading=0.0, data_pts_per_line=20) 

     # file = open("file.txt", "w")
     # l = computeLine(-20,4,-22,4)
     # writeLine(name="file.txt", start_point=[-37.0,-24.0, 4.0], end_point=[-37.0,-20.0, 4.0], line=l, number_data_pts=60, heading=1.57)
     # writeLine(name="file.txt", start_point=[0.0, 4.0, 4.0], end_point=[0.0, -4.0, 4.0], line=l, number_data_pts=120, heading=0.869)

if __name__ == "__main__":
    main()