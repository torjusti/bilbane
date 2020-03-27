import arcade
import math
import pyglet.gl as gl
def create_arc_outline(center_x: float, center_y: float, width: float,
                     height: float, color: arcade.Color,
                     start_angle: float, end_angle: float,
                     border_width: float = 1, tilt_angle: float = 0,
                     num_segments: int = 256):
    """
        Creates arc outline
    """
    unrotated_point_list = []

    inside_width = width - border_width / 2
    outside_width = width + border_width / 2
    inside_height = height - border_width / 2
    outside_height = height + border_width / 2

    for segment in range(num_segments + 1):
        theta = math.pi / 180 * (start_angle + segment * (end_angle - start_angle) / num_segments)

        x1 = inside_width * math.cos(theta)
        y1 = inside_height * math.sin(theta)

        x2 = outside_width * math.cos(theta)
        y2 = outside_height * math.sin(theta)

        unrotated_point_list.append([x1, y1])
        unrotated_point_list.append([x2, y2])

    if tilt_angle == 0:
        uncentered_point_list = unrotated_point_list
    else:
        uncentered_point_list = []
        for point in unrotated_point_list:
            uncentered_point_list.append(arcade.rotate_point(point[0], point[1], 0, 0, tilt_angle))

    point_list = []
    for point in uncentered_point_list:
        point_list.append((point[0] + center_x, point[1] + center_y))
    shape_mode = gl.GL_LINE_STRIP
    return arcade.create_line_generic(point_list, color, shape_mode, border_width)
