import math

# circle right 
# radius = 1
# start_point_x = 10.0
# start_point_y = -0.5
# start_point_z = 4.0
# heading = 0.0
# data_points=70

# file_name = "circle_right_" + str(start_point_x) + "_" + str(start_point_y) + "_" + str(start_point_z) + ".txt"  
# file = open(file_name, 'w')

# angle_discretization = (2*math.pi) / float(data_points) 
# angle = 0
# #circle
# for val in range(data_points):
#     # x = radius*math.cos(angle)
#     z = radius*math.cos(angle)
#     y = radius*math.sin(angle)
#     angle +=angle_discretization
#     # x = start_point_x + round(x, 6)
#     y = start_point_y + y
#     z = start_point_z + z
    
#     file.write(str(start_point_x)[0:5] +" "+str(y)[0:5]+" "+str(z)[0:5]+" "+str(heading)[0:5]+"\n")

# file.close()

# circle left
# radius = 1
# start_point_x = 3.5
# start_point_y = 0.5
# start_point_z = 4.0
# heading = 0.0
# data_points=70

# file_name = "circle_left_" + str(start_point_x) + "_" + str(start_point_y) + "_" + str(start_point_z) + ".txt"  
# file = open(file_name, 'w')

# angle_discretization = (2*math.pi) / float(data_points) 
# angle = 0
# #circle
# for val in range(data_points):
#     # x = radius*math.cos(angle)
#     z = radius*math.cos(-angle)
#     y = radius*math.sin(-angle)
#     angle +=angle_discretization
#     # x = start_point_x + round(x, 6)
#     y = start_point_y + float((str(y)[0:4]))
#     z = start_point_z + float((str(z)[0:4]))
#     file.write(str(start_point_x)[0:5]+" "+str(y)[0:5]+" "+str(z)[0:5]+" "+str(heading)[0:5]+"\n")

# file.close()



def computeLine(first_x, first_y, second_x, second_y):
    if(second_x - first_x == 0):
        slope = 0.0
    else:
        slope = (second_y - first_y) / (second_x - first_x)

    y_intercept = first_y - (slope * first_x)
    return slope,y_intercept


# star 
start_point_x = 3.0
start_point_y = 0.0
start_point_z = 6.0
left_down_y = 2.0
left_down_z = 3.0
left_up_y = 2.5
left_up_z = 5.5
right_down_y = -2.0
right_down_z = 3.0
right_up_y = -2.5
right_up_z = 5.5

line_1 = computeLine(start_point_y, start_point_z, left_down_y, left_down_z)
line_2 = computeLine(left_down_y, left_down_z,  right_up_y, right_up_z)
line_3 = computeLine(right_up_y, right_up_z, left_up_y, left_up_z)
line_4 = computeLine(left_up_y, left_up_z, right_down_y, right_down_z)
line_5 = computeLine(right_down_y, right_down_z, start_point_y, start_point_z)


heading = 0.0
data_points=120

file_name = "star" + str(start_point_x) + "_" + str(start_point_y) + "_" + str(start_point_z) + ".txt"  
file = open(file_name, 'w')

points_per_line = int(data_points / 4)
diff = left_down_y - start_point_y
discretization =  float(diff) / float(points_per_line)
start_point_copy_y = start_point_y
for val in range(points_per_line):
    start_point_copy_y = start_point_copy_y + discretization
    y_val = start_point_copy_y
    z = line_1[0] * y_val + line_1[1]
    if(start_point_x < 0.001 and start_point_x > -0.001):
         start_point_x = 0.0
    if(z < 0.001 and z > -0.001):
         z = 0.0
    if(y_val < 0.001 and y_val > -0.001):
         y_val = 0.0 
    file.write(str(start_point_x)[0:5]+" "+str(y_val)[0:5]+" "+str(z)[0:5]+" "+str(heading)[0:5]+"\n")

diff = right_up_y -  left_down_y
discretization =  float(diff) / float(points_per_line)
start_point_copy_y = left_down_y
for val in range(points_per_line):
    start_point_copy_y = start_point_copy_y + discretization
    y_val = start_point_copy_y
    z = line_2[0] * y_val + line_2[1]
    if(start_point_x < 0.001 and start_point_x > -0.001):
         start_point_x = 0.0
    if(z < 0.001 and z > -0.001):
         z = 0.0
    if(y_val < 0.001 and y_val > -0.001):
         y_val = 0.0 
    file.write(str(start_point_x)[0:5]+" "+str(y_val)[0:5]+" "+str(z)[0:5]+" "+str(heading)[0:5]+"\n")

diff = left_up_y - right_down_y
discretization =  float(diff) / float(points_per_line)
start_point_copy_y = right_down_y
for val in range(points_per_line):
    start_point_copy_y = start_point_copy_y + discretization
    y_val = start_point_copy_y
    z = line_3[0] * y_val + line_3[1]
    if(start_point_x < 0.001 and start_point_x > -0.001):
         start_point_x = 0.0
    if(z < 0.001 and z > -0.001):
         z = 0.0
    if(y_val < 0.001 and y_val > -0.001):
         y_val = 0.0 
    file.write(str(start_point_x)[0:5]+" "+str(y_val)[0:5]+" "+str(z)[0:5]+" "+str(heading)[0:5]+"\n")

diff = right_down_y - left_up_y
discretization =  float(diff) / float(points_per_line)
start_point_copy_y = left_up_y
for val in range(points_per_line):
    start_point_copy_y = start_point_copy_y + discretization
    y_val = start_point_copy_y
    z = line_4[0] * y_val + line_4[1]
    if(start_point_x < 0.001 and start_point_x > -0.001):
         start_point_x = 0.0
    if(z < 0.001 and z > -0.001):
         z = 0.0
    if(y_val < 0.001 and y_val > -0.001):
         y_val = 0.0 
    file.write(str(start_point_x)[0:5]+" "+str(y_val)[0:5]+" "+str(z)[0:5]+" "+str(heading)[0:5]+"\n")

diff = start_point_y - right_down_y
discretization =  float(diff) / float(points_per_line)
start_point_copy_y = right_down_y
for val in range(points_per_line):
    start_point_copy_y = start_point_copy_y + discretization
    y_val = start_point_copy_y
    z = line_5[0] * y_val + line_5[1]
    if(start_point_x < 0.001 and start_point_x > -0.001):
         start_point_x = 0.0
    if(z < 0.001 and z > -0.001):
         z = 0.0
    if(y_val < 0.001 and y_val > -0.001):
         y_val = 0.0 
    file.write(str(start_point_x)[0:5]+" "+str(y_val)[0:5]+" "+str(z)[0:5]+" "+str(heading)[0:5]+"\n")


file.close()