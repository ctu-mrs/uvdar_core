import math

def writeCircle(name, radius, x, y, z, heading, data_points, ccw):
     file_name = name + "_" + str(x) + "_" + str(y) + "_" + str(z) + "_" + str(heading) + ".txt"  
     file = open(file_name, 'w')

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
          print(y_computed)
          file.write(str(x)[0:5] +" "+str(y_computed)[0:5]+" "+str(z_computed)[0:5]+" "+str(heading)[0:5]+"\n")

     file.close()



def computeLine(first_x, first_y, second_x, second_y):
    if(second_x - first_x == 0):
        slope = 0.0
    else:
        slope = (second_y - first_y) / (second_x - first_x)

    y_intercept = first_y - (slope * first_x)
    return slope,y_intercept


def writeLine( name, start_point, end_point, line, number_data_pts, heading):
     file = open(name, 'a')
     diff = end_point[1] - start_point[1]
     discretization =  float(diff) / float(number_data_pts)
     y_val = start_point[1] + discretization
     for val in range(number_data_pts):
          y_val = y_val + discretization
          z = line[0] * y_val + line[1]

          if(start_point[0] < 0.001 and start_point[0] > -0.001):
               start_point[0] = 0.0
          if(z < 0.001 and z > -0.001):
               z = 0.0
          if(y_val < 0.001 and y_val > -0.001):
               y_val = 0.0 
          file.write(str(start_point[0])[0:5]+" "+str(y_val)[0:5]+" "+str(z)[0:5]+" "+str(heading)[0:5]+"\n")
     
     file.close()


def compute_star(name, start_point, second, third, fourth, fifth, heading, data_pts_per_line):
     line_1 = computeLine(start_point[1], start_point[2], second[1], second[2])
     line_2 = computeLine(second[1], second[2], third[1], third[2])
     line_3 = computeLine(third[1], third[2], fourth[1], fourth[2])
     line_4 = computeLine(fourth[2], fourth[2], fifth[1], fifth[2])
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
     

def main():
     writeCircle(name="trajectory_files/two_tx/tx2_circle", radius=1, x=8.0, y=-0.0, z=4.0, heading=0.0, data_points=60, ccw=1)
     # writeCircle(name="trajectory_files/two_tx/tx1_circle", radius=1, x=4.0, y=0.0, z=4.0, heading=0.0, data_points=100, ccw=1)
     
     # name="trajectory_files/star"
     # start_point = [3.0, 0.0, 6.0]
     # left_down = [3.0, 2.0, 3.0]
     # left_up = [3.0, 2.5, 5.5]
     # right_down = [3.0, -2.0, 3.0]
     # right_up = [3.0, -2.5, 5.5]
     # compute_star(name, start_point, left_down, left_up, right_down, right_up, heading=0.0) 

     # name="trajectory_files/star"
     # first = [12.0, 2.5, 5.5]
     # second = [12.0, -2.0, 3.0]
     # third = [12.0, 0.0, 6.0]
     # fourth = [12.0, 2.0, 3.0]
     # fifth = [12.0, -2.5, 5.5]
     # compute_star(name, first, second, third, fourth, fifth, heading=0.0, data_pts_per_line=40) 

if __name__ == "__main__":
    main()
