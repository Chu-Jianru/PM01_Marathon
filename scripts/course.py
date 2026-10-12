#!/usr/bin/env python3
"""Course geometry. Bend radius 60 m."""
import math


# Running path is the reference line. The right lane is 3.5 m wide.
# Another 3.5 m on the left is the passing lane. Outer edges are the boundaries.
RIGHT_EDGE = -1.75
DIVIDER = 1.75
LEFT_EDGE = 5.25
BARRIER = 7.6
SIDEWALK = 9.2
STEP = 0.2
GROUND_PITCH = math.radians(1.0)

# (kind, length_or_angle). "straight" uses meters, "arc" uses radians.
# Positive arc is a left bend. The block repeats to lengthen the road.
_BLOCK = (
    ("straight", 22.0),
    ("arc", math.radians(25.0)),
    ("straight", 18.0),
    ("arc", math.radians(-25.0)),
    ("straight", 22.0),
)
SEGMENTS = _BLOCK * 4
ARC_RADIUS = 60.0


def ground_z(x):
    return x * math.tan(GROUND_PITCH)

# (name, distance_along_m, lateral_m, radius_m, height_m, r, g, b)
# lateral > 0 is to the left of the direction of travel.
OBSTACLES = (
    ("lane_cone", 55.0, -0.85, 0.18, 0.90, 0.9, 0.35, 0.05),
    ("lane_cone_b", 170.0, -0.85, 0.18, 0.90, 0.9, 0.35, 0.05),
)


def _append_straight(points, length, step):
    x, y, yaw = points[-1]
    traveled = step
    while traveled < length - 1e-6:
        points.append((x + traveled * math.cos(yaw), y + traveled * math.sin(yaw), yaw))
        traveled += step
    points.append((x + length * math.cos(yaw), y + length * math.sin(yaw), yaw))


def _append_arc(points, radius, angle, step):
    x, y, yaw = points[-1]
    direction = 1.0 if angle >= 0.0 else -1.0
    left_x = -math.sin(yaw)
    left_y = math.cos(yaw)
    center_x = x + direction * radius * left_x
    center_y = y + direction * radius * left_y
    vx = x - center_x
    vy = y - center_y
    length = radius * abs(angle)
    count = max(1, int(round(length / step)))
    for index in range(1, count + 1):
        delta = angle * (index / float(count))
        cosine = math.cos(delta)
        sine = math.sin(delta)
        rotated_x = vx * cosine - vy * sine
        rotated_y = vx * sine + vy * cosine
        points.append((center_x + rotated_x, center_y + rotated_y, yaw + delta))


def centerline(step=STEP):
    points = [(0.0, 0.0, 0.0)]
    for kind, value in SEGMENTS:
        if kind == "straight":
            _append_straight(points, value, step)
        else:
            _append_arc(points, ARC_RADIUS, value, step)
    return points


def offset_path(points, lateral):
    offset = []
    for x, y, yaw in points:
        left_x = -math.sin(yaw)
        left_y = math.cos(yaw)
        offset.append((x + lateral * left_x, y + lateral * left_y, yaw))
    return offset


def pose_at_distance(points, distance):
    traveled = 0.0
    for index in range(1, len(points)):
        x0, y0, _ = points[index - 1]
        x1, y1, yaw1 = points[index]
        segment = math.hypot(x1 - x0, y1 - y0)
        if traveled + segment >= distance:
            if segment < 1e-6:
                return points[index]
            fraction = (distance - traveled) / segment
            yaw = points[index - 1][2]
            return (
                x0 + fraction * (x1 - x0),
                y0 + fraction * (y1 - y0),
                yaw + fraction * _angle_delta(yaw, yaw1),
            )
        traveled += segment
    return points[-1]


def _angle_delta(start, end):
    delta = end - start
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def course_length(points=None):
    points = points if points is not None else centerline()
    length = 0.0
    for index in range(1, len(points)):
        x0, y0, _ = points[index - 1]
        x1, y1, _ = points[index]
        length += math.hypot(x1 - x0, y1 - y0)
    return length


def write_world(path):
    """Write the world file."""
    points = centerline(step=3.0)
    chunks = ["""<?xml version="1.0"?>
<sdf version="1.4">
  <world name="marathon_course">
    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.75 0.82 0.9 1</background>
      <shadows>false</shadows>
    </scene>
    <include><uri>model://sun</uri></include>
""" + _road_model(points) + """
    <physics type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_update_rate>500</real_time_update_rate>
      <real_time_factor>1</real_time_factor>
    </physics>
"""]
    solid = centerline(step=2.0)
    for side, lateral in (("left_edge", LEFT_EDGE), ("right_edge", RIGHT_EDGE)):
        for index, (x, y, yaw) in enumerate(offset_path(solid, lateral)):
            chunks.append(_box_model(
                "paint_{}_{}".format(side, index), x, y, ground_z(x) + 0.02, yaw,
                2.05, 0.12, 0.02, (0.95, 0.95, 0.95),
            ))
    dashes = centerline(step=4.0)
    for index, (x, y, yaw) in enumerate(offset_path(dashes, DIVIDER)):
        chunks.append(_box_model(
            "paint_divider_{}".format(index), x, y, ground_z(x) + 0.02, yaw,
            1.2, 0.12, 0.02, (0.95, 0.95, 0.95),
        ))
    for item in obstacle_poses(centerline()):
        chunks.append(_cylinder_model(
            item["name"], item["x"], item["y"], ground_z(item["x"]) + item["height"] * 0.5, item["yaw"],
            item["radius"], item["height"], item["color"], collide=True,
        ))
    dense = centerline(step=1.0)
    length = course_length(dense)
    distance = 24.0
    index = 0
    while distance < length - 15.0:
        x, y, yaw = pose_at_distance(dense, distance)
        side = 1.0 if index % 2 == 0 else -1.0
        chunks.append(_placed_box(
            "building_{}".format(index), x, y, yaw, side * 16.0,
            14.0, 8.0, 8.0, (0.55, 0.52, 0.48),
        ))
        chunks.append(_placed_cylinder(
            "tree_{}".format(index), x, y, yaw, -side * 9.0,
            0.35, 6.0, (0.15, 0.35, 0.16),
        ))
        distance += 40.0
        index += 1
    post = 12.0
    post_index = 0
    while post < length - 8.0:
        x, y, yaw = pose_at_distance(dense, post)
        chunks.append(_placed_cylinder(
            "barrier_{}".format(post_index), x, y, yaw, BARRIER,
            0.06, 1.05, (0.75, 0.75, 0.78),
        ))
        post += 16.0
        post_index += 1
    for name, along, lateral in (
            ("walker_0", 40.0, SIDEWALK),
            ("walker_1", 180.0, SIDEWALK),
    ):
        x, y, yaw = pose_at_distance(dense, along)
        chunks.append(_placed_cylinder(
            name, x, y, yaw, lateral, 0.25, 1.7, (0.75, 0.2, 0.2),
        ))
    chunks.append("  </world>\n</sdf>\n")
    with open(path, "w") as handle:
        handle.write("".join(chunks))


def _cylinder_model(name, x, y, z, yaw, radius, height, color, collide):
    collision = ""
    if collide:
        collision = (
            "        <collision name='collision'><geometry><cylinder>"
            "<radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>\n"
        ).format(r=radius, h=height)
    return (
        "    <model name='{name}'>\n"
        "      <static>true</static>\n"
        "      <pose>{x} {y} {z} 0 0 {yaw}</pose>\n"
        "      <link name='link'>\n"
        "{collision}"
        "        <visual name='visual'>\n"
        "          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>\n"
        "          <material><ambient>{cr} {cg} {cb} 1</ambient><diffuse>{cr} {cg} {cb} 1</diffuse></material>\n"
        "        </visual>\n"
        "      </link>\n"
        "    </model>\n"
    ).format(
        name=name, x=x, y=y, z=z, yaw=yaw, r=radius, h=height,
        cr=color[0], cg=color[1], cb=color[2], collision=collision,
    )


def _box_model(name, x, y, z, yaw, length, width, height, color, collide=False):
    collision = ""
    if collide:
        collision = (
            "        <collision name='collision'><geometry><box>"
            "<size>{length} {width} {height}</size></box></geometry></collision>\n"
        ).format(length=length, width=width, height=height)
    return (
        "    <model name='{name}'>\n"
        "      <static>true</static>\n"
        "      <pose>{x} {y} {z} 0 0 {yaw}</pose>\n"
        "      <link name='link'>\n"
        "{collision}"
        "        <visual name='visual'>\n"
        "          <geometry><box><size>{length} {width} {height}</size></box></geometry>\n"
        "          <material><ambient>{cr} {cg} {cb} 1</ambient><diffuse>{cr} {cg} {cb} 1</diffuse></material>\n"
        "        </visual>\n"
        "      </link>\n"
        "    </model>\n"
    ).format(
        name=name, x=x, y=y, z=z, yaw=yaw,
        length=length, width=width, height=height,
        cr=color[0], cg=color[1], cb=color[2], collision=collision,
    )


def _offset_xy(x, y, yaw, lateral):
    return x + lateral * -math.sin(yaw), y + lateral * math.cos(yaw)


def _placed_box(name, x, y, yaw, lateral, length, width, height, color):
    px, py = _offset_xy(x, y, yaw, lateral)
    return _box_model(name, px, py, ground_z(px) + height * 0.5, yaw, length, width, height, color, collide=True)


def _placed_cylinder(name, x, y, yaw, lateral, radius, height, color):
    px, py = _offset_xy(x, y, yaw, lateral)
    return _cylinder_model(
        name, px, py, ground_z(px) + height * 0.5, yaw, radius, height, color, collide=True,
    )


def _road_model(points):
    """Sloped boxes. A Gazebo plane collision stays horizontal."""
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    margin = 25.0
    x0 = min(xs) - margin
    x1 = max(xs) + margin
    size_y = max(40.0, (max(ys) - min(ys)) + 2.0 * margin)
    center_y = 0.5 * (min(ys) + max(ys))
    thickness = 0.5
    tile = 40.0
    alpha = GROUND_PITCH
    pitch = -alpha
    chunks = []
    x = x0
    index = 0
    while x < x1 - 1e-6:
        length = min(tile + 0.4, x1 - x)
        surface_x = x + 0.5 * length
        surface_z = ground_z(surface_x)
        center_x = surface_x + math.sin(alpha) * thickness * 0.5
        center_z = surface_z - math.cos(alpha) * thickness * 0.5
        chunks.append(
            (
                "    <model name='road_{index}'>\n"
                "      <static>true</static>\n"
                "      <pose>{cx} {cy} {cz} 0 {pitch} 0</pose>\n"
                "      <link name='link'>\n"
                "        <collision name='collision'><geometry><box>"
                "<size>{length} {width} {height}</size></box></geometry></collision>\n"
                "        <visual name='visual'>\n"
                "          <geometry><box><size>{length} {width} {height}</size></box></geometry>\n"
                "          <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse></material>\n"
                "        </visual>\n"
                "      </link>\n"
                "    </model>\n"
            ).format(
                index=index, cx=center_x, cy=center_y, cz=center_z, pitch=pitch,
                length=length, width=size_y, height=thickness,
            )
        )
        x += tile
        index += 1
    return "".join(chunks)


def obstacle_poses(points=None):
    points = points if points is not None else centerline()
    poses = []
    for name, distance, lateral, radius, height, red, green, blue in OBSTACLES:
        x, y, yaw = pose_at_distance(points, distance)
        left_x = -math.sin(yaw)
        left_y = math.cos(yaw)
        poses.append({
            "name": name,
            "x": x + lateral * left_x,
            "y": y + lateral * left_y,
            "yaw": yaw,
            "radius": radius,
            "height": height,
            "color": (red, green, blue),
        })
    return poses
