#!/usr/bin/env python
# _*_ coding: utf_8 _*_
"""
fablab_lasercut

This program prepare prepare rectangular shapes for assembly with a laser cut
"""

# from solid import *
import numpy as np
import math


def point_2_vector(point_a, point_b):
    """ Function doc
    point_2_vector return a vector associated to a segment
    @param point_a: numpy start point of the segment (cartesian)
    @param point_b: numpy end point of the segment (cartesian)
    @return RETURN: vector tuple
    """
    return tuple(point_b[i]-point_a[i] for i in range(0, len(point_a)))

def get_normal_sized(point_a, point_b, center, length):
    """ Function doc
    get_normal_sized return a vector representing the inside normal for
    a segment a polygon
    @param point_a: numpy start point of the segment (cartesian)
    @param point_b: numpy end point of the segment (cartesian)
    @param center: numpy center of the polygon (cartesian)
    @param length: length of the vector
    @return RETURN: vector with the correct length
    """
    # Get vector
    (d_x, d_y) = point_2_vector(point_a, point_b)
    # Get normal vectors
    normals = [[-d_y, d_x], [d_y, -d_x]]
    # Find the inside normal
    dist_norm_center = [np.linalg.norm(center-(point_b+n)) for n in normals]
    if dist_norm_center[0] < dist_norm_center[1]:
        normal = normals[0]
    else:
        normal = normals[1]
    # sized the normal to fit the depth
    return np.array(normal) / np.linalg.norm(normal) * length
    # end

def cut_rect(size_x, size_y, interv, depth, margin, segment_reduce_at_start=None):
    """ Function doc
    cut_rect create a rectangulare polygon with teeths
    @param size_x: X size
    @param size_y: Y size
    @return RETURN: a list of polygon points
    """
    return cut_polygon([[0, 0], [size_x, 0], [size_x, size_y], [0, size_y]],
                       interv, depth, margin, segment_reduce_at_start)

def cut_polygon(polygon, interv, depth, margin, segment_reduce_at_start=None):
    """ Function doc
    cut_rect create a rectangulare polygon with teeths
    @param size_x: X size
    @param size_y: Y size
    @return RETURN: a list of polygon points
    """

    # add the origin also as the final point and we create the original polygon
    polygon.append(polygon[0])
    polygon_orig = np.array(polygon)

    # prepare the final polygon and give its size
    polygon_final = np.array([polygon[0]])

    # If segment_reduce_at_start is not provide full it with zeros
    if segment_reduce_at_start is None:
        segment_reduce_at_start = [0 for i in range(0, len(polygon)-1)]

    # calculate the center of the polygon
    center = np.mean(polygon_orig, axis=0)

    # get the normals of each segment
    norm_sized = []
    for i in range(0, len(polygon)-1):
        point_a = polygon_orig[i]
        point_b = polygon_orig[i+1]
        norm_sized.append(get_normal_sized(point_a, point_b, center, depth))

    # split segments in 'interv' intervals
    for i in range(0, len(polygon)-1):
        point_a = polygon_orig[i]
        point_b = polygon_orig[i+1]

        # get a splited vector
        vect_interv = np.array(list(point_2_vector(point_a,
                                                   point_b))) / interv

        # create interval
        for j in range(0, interv):
            point_c = point_a+j*vect_interv
            # for the first point we may need to reduce in X and Y
            if j == 0:
                if segment_reduce_at_start[i]:
                    point_c += norm_sized[i]
                if segment_reduce_at_start[i-1] or (interv+1)%2:
                    point_c += norm_sized[i-1]
                polygon_final = np.vstack([polygon_final, point_c])
            # for the other intervals we add 2 points
            else:
                if (j+segment_reduce_at_start[i]) % 2:
                    polygon_final = np.vstack([polygon_final, point_c])
                    polygon_final = np.vstack([polygon_final,
                                               point_c+norm_sized[i]])
                else:
                    polygon_final = np.vstack([polygon_final,
                                               point_c+norm_sized[i]])
                    polygon_final = np.vstack([polygon_final, point_c])

    # we return the polygon without the redundant first point
    return polygon_final[1:].tolist()

def translate (vector, obj):
    """ Function translate
    return openscad translate command
    @param vector: [x, y, z] translation matrix
    @param obj: text object to translate
    """
    return "translate({}){{{}}}".format(vector, obj)

def polygon (points, extrude=0):
    """ Function translate
    return openscad polygon command
    @param points: points of the polygon
    """
    if extrude > 0:
        extrude = "linear_extrude(height={})".format(extrude)
    else:
        extrude = ""
    return "{} polygon(points={});".format(extrude,  points)

def rotate (angles, obj):
    """ Function rotate
    return openscad rotate command
    @param angles: [x, y, z] angles matrix
    @param obj: text object to translate
    """
    return "rotate({}){{{}}}".format(angles, obj)

def set_house(depth, interv, x, y, z, fact, gap, alpha, margin):
    """ Function set house
    create module to build a house with openscad
    @param depth: the depth of the material
    @param interv: how much teeth do you want
    @param x: x size
    @param y: y size
    @param z: z size (of the wall, not the top roof
    @param fact: the factor to apply to z to get the total width of wall+roof
    @param gap: text object to translate
    @param alpha: the alpha to apply to the color
    """

    floor = polygon(cut_rect(size_x=x,
                             size_y=y,
                             interv=interv,
                             depth=depth,
                             margin=margin,
                             segment_reduce_at_start=[1, 0, 1, 0]), depth)
    print("module floor(){", 'color("IndianRed", {})'.format(alpha), floor, "}")

    wall1 = polygon(cut_rect(size_x=z,
                             size_y=y,
                             interv=interv,
                             depth=depth,
                             margin=margin,
                             segment_reduce_at_start=[1, 1, 1, 1]), depth)
    print("module wall1(){", 'color("Yellow", {})'.format(alpha), wall1, "}")

    wall2 = polygon(cut_rect(size_x=z,
                             size_y=y,
                             interv=interv,
                             depth=depth,
                             margin=margin,
                             segment_reduce_at_start=[1, 1, 1, 1]), depth)
    print("module wall2(){", 'color("Yellow", {})'.format(alpha), wall2, "}")

    wall_gable1 = polygon(cut_polygon(polygon = [[0, 0], [x, 0],
                                          [x, z], [x/2, z*fact],
                                          [0, z]],
                                      interv=interv,
                                      margin=margin,
                                      depth=depth), depth)
    print("module wall_gable1(){", 'color("Violet", {})'.format(alpha), wall_gable1, "}")

    wall_gable2 = polygon(cut_polygon(polygon = [[x/2, 0], [x, z*(fact-1)],
                                          [x, z*fact], [0, z*fact],
                                          [0, z*(fact-1)]],
                                      interv=interv,
                                      margin=margin,
                                      depth=depth), depth)
    print("module wall_gable2(){", 'color("Violet", {})'.format(alpha), wall_gable2, "}")

    roof1 = polygon(cut_rect(size_x=math.sqrt((x/2)**2+(z*(fact-1))**2),
                             size_y=y,
                             interv=interv,
                             depth=depth,
                             margin=margin,
                             segment_reduce_at_start=[1, 0, 1, 0]), depth)
    print("module roof1(){", 'color("PaleTurquoise", {})'.format(alpha), roof1, "}")


    roof2 = polygon(cut_rect(size_x=math.sqrt((x/2)**2+(z*(fact-1))**2),
                             size_y=y,
                             interv=interv,
                             depth=depth,
                             margin=margin,
                             segment_reduce_at_start=[1, 1, 1, 0]), depth)
    print("module roof2(){", 'color("DeepSkyBlue", {})'.format(alpha), roof2, "}")

def set_cube(depth, interv, x, y, z, gap, alpha, margin):
    """ Function set house
    create module to build a house with openscad
    @param depth: the depth of the material
    @param interv: how much teeth do you want
    @param x: x size
    @param y: y size
    @param z: z size (of the wall, not the top roof
    @param gap: text object to translate
    @param alpha: the alpha to apply to the color
    """

    floor = polygon(cut_rect(size_x=x,
                             size_y=y,
                             interv=interv,
                             depth=depth,
                             margin=margin,
                             segment_reduce_at_start=[1, 0, 1, 0]), depth)
    print("module floor(){", 'color("IndianRed", {})'.format(alpha), floor, "}")

    wall1 = polygon(cut_rect(size_x=z,
                             size_y=y,
                             interv=interv,
                             depth=depth,
                             margin=margin,
                             segment_reduce_at_start=[1, 1, 1, 1]), depth)
    print("module wall1(){", 'color("Yellow", {})'.format(alpha), wall1, "}")

    wall2 = polygon(cut_rect(size_x=z,
                             size_y=y,
                             interv=interv,
                             depth=depth,
                             margin=margin,
                             segment_reduce_at_start=[1, 1, 1, 1]), depth)
    print("module wall2(){", 'color("Yellow", {})'.format(alpha), wall2, "}")

    wall3 = polygon(cut_rect(size_x=y,
                             size_y=z,
                             interv=interv,
                             depth=depth,
                             margin=margin,
                             segment_reduce_at_start=[0, 0, 0, 0]), depth)
    print("module wall3(){", 'color("Violet", {})'.format(alpha), wall3, "}")

    wall4 = polygon(cut_rect(size_x=y,
                             size_y=z,
                             interv=interv,
                             depth=depth,
                             margin=margin,
                             segment_reduce_at_start=[0, 0, 0, 0]), depth)
    print("module wall4(){", 'color("Violet", {})'.format(alpha), wall4, "}")

    top = floor
    print("module top(){", 'color("PaleTurquoise", {})'.format(alpha), top, "}")

def print_2d_house(x, y, z, fact, gap, depth):
    print(translate([0, 0, 0], "floor();"))
    print(translate([x+gap, 0, 0], "wall1();" ))
    print(translate([-z-gap, 0, 0], "wall2();"))
    print(translate([0, y+gap, 0], "wall_gable1();"))
    print(translate([0, -(z*fact+gap), 0], "wall_gable2();"))
    print(translate([x+z+2*gap, 0, 0], "roof1();"))
    print(rotate([0, 180, 0], translate([z+2*gap, 0, -depth], "roof2();")))

def print_3d_house(x, y, z, fact, gap, depth):
    print(translate([0, 0, 0], "floor();"))
    print(translate([x-gap+depth, 0, 0], rotate([0, -90, 0], "wall1();" )))
    print(translate([gap, 0, 0], rotate([0, -90, 0], "wall2();")))
    print(translate([0, y-gap+depth, 0], rotate([90, 0, 0], "wall_gable1();")))
    print(translate([0, gap, 0], rotate([90, 0, 0], "wall_gable1();")))
    angle_r = math.atan(z*(fact-1)/(x/2))
    angle = math.degrees(angle_r)
    print(translate([x/2-math.sin(angle_r)*depth, -gap+depth, z*fact-math.cos(angle_r)*depth-gap+depth], rotate([0, angle, 0] , "roof1();")))
    print(translate([math.sin(angle_r)*depth, gap-depth, z-math.cos(angle_r)*depth+gap-depth], rotate([0, -angle, 0], "roof2();")))

def print_2d_cube(x, y, z, gap, depth):
    print(translate([0, 0, 0], "floor();"))
    print(translate([x+gap, 0, 0], "wall1();" ))
    print(translate([-z-gap, 0, 0], "wall2();"))
    print(translate([0, y+gap, 0], "wall3();"))
    print(translate([0, -(z+gap), 0], "wall4();"))
    print(translate([x+z+2*gap, 0, 0], "top();"))

def print_3d_cube(x, y, z, gap, depth):
    print(translate([0, 0, 0], "floor();"))
    print(translate([x-gap+depth, 0, 0], rotate([0, -90, 0], "wall1();" )))
    print(translate([gap, 0, 0], rotate([0, -90, 0], "wall2();")))
    print(translate([0, y-gap+depth, 0], rotate([90, 0, 0], "wall3();")))
    print(translate([0, gap, 0], rotate([90, 0, 0], "wall4();")))
    print(translate([0, 0, z-gap],  "top();"))



def main():
    """ Function doc
    main function
    """

    depth = 2
    interv = 9
    x = 100
    y = 100
    z = 50
    fact = 1.5
    gap = depth
    alpha = 0.5
    margin = 0.1

    set_house(depth, interv, x, y, z, fact, -1*gap, alpha, margin)
    print_2d_house(x, y, z, fact, gap, depth)
    # print_3d_house(x, y, z, fact, gap, depth)

    # set_cube(depth, interv, x, y, z, -1*gap, alpha, margin)
    # print_2d_cube(x, y, z, gap, depth)
    # print_3d_cube(x, y, z, gap, depth)

    return 0

if __name__ == '__main__':
    main()
