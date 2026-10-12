#!/usr/bin/env python3
"""Road geometry for a short piece of the marathon course.

The yaw table allows about 0.09 rad/s at 3.5 m/s, so bends use a 60 m
radius. Lane lines are the camera output: a center line and two boundaries.
"""
import math


HALF_WIDTH = 2.0
STEP = 0.2

# (kind, length_or_angle). "straight" uses meters, "arc" uses radians.
# Positive arc is a left bend.
SEGMENTS = (
    ("straight", 22.0),
    ("arc", math.radians(25.0)),
    ("straight", 18.0),
    ("arc", math.radians(-25.0)),
    ("straight", 22.0),
)
ARC_RADIUS = 60.0

# (name, distance_along_m, lateral_m, radius_m, height_m, r, g, b)
# lateral > 0 is to the left of the direction of travel.
OBSTACLES = (
    ("low_clutter", 12.0, 1.8, 0.25, 0.35, 0.45, 0.45, 0.45),
    ("cone_a", 26.0, 1.25, 0.18, 0.90, 0.9, 0.35, 0.05),
    ("spectator", 18.0, 3.2, 0.28, 1.70, 0.15, 0.35, 0.75),
    ("person", 50.0, 2.4, 0.25, 1.70, 0.75, 0.2, 0.2),
    ("cone_b", 78.0, 1.6, 0.18, 0.90, 0.9, 0.35, 0.05),
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
    """Write the Gazebo world so every object exists before the robot moves."""
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
    for side, lateral in (("left", HALF_WIDTH), ("right", -HALF_WIDTH)):
        for index, (x, y, yaw) in enumerate(offset_path(points, lateral)):
            chunks.append(_box_model(
                "paint_{}_{}".format(side, index), x, y, 0.01, yaw,
                0.7, 0.12, 0.02, (0.95, 0.95, 0.95), collide=False,
            ))
    for item in obstacle_poses(centerline()):
        chunks.append(_cylinder_model(
            item["name"], item["x"], item["y"], item["height"] * 0.5, item["yaw"],
            item["radius"], item["height"], item["color"], collide=True,
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


def _box_model(name, x, y, z, yaw, length, width, height, color, collide):
    return (
        "    <model name='{name}'>\n"
        "      <static>true</static>\n"
        "      <pose>{x} {y} {z} 0 0 {yaw}</pose>\n"
        "      <link name='link'>\n"
        "        <visual name='visual'>\n"
        "          <geometry><box><size>{length} {width} {height}</size></box></geometry>\n"
        "          <material><ambient>{cr} {cg} {cb} 1</ambient><diffuse>{cr} {cg} {cb} 1</diffuse></material>\n"
        "        </visual>\n"
        "      </link>\n"
        "    </model>\n"
    ).format(
        name=name, x=x, y=y, z=z, yaw=yaw,
        length=length, width=width, height=height,
        cr=color[0], cg=color[1], cb=color[2],
    )


def _road_model(points):
    """Grey road large enough to hold the whole segment.

    The default Gazebo ground plane is 100 m square, so a course past
    x = 50 m used to run off the grey square into the sky colour.
    """
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    margin = 40.0
    center_x = 0.5 * (min(xs) + max(xs))
    center_y = 0.5 * (min(ys) + max(ys))
    size_x = max(80.0, (max(xs) - min(xs)) + 2.0 * margin)
    size_y = max(40.0, (max(ys) - min(ys)) + 2.0 * margin)
    return (
        "    <model name='road'>\n"
        "      <static>true</static>\n"
        "      <pose>{cx} {cy} 0 0 0 0</pose>\n"
        "      <link name='link'>\n"
        "        <collision name='collision'>\n"
        "          <geometry><plane><normal>0 0 1</normal><size>{sx} {sy}</size></plane></geometry>\n"
        "        </collision>\n"
        "        <visual name='visual'>\n"
        "          <geometry><plane><normal>0 0 1</normal><size>{sx} {sy}</size></plane></geometry>\n"
        "          <material>\n"
        "            <ambient>0.45 0.45 0.45 1</ambient>\n"
        "            <diffuse>0.45 0.45 0.45 1</diffuse>\n"
        "          </material>\n"
        "        </visual>\n"
        "      </link>\n"
        "    </model>\n"
    ).format(cx=center_x, cy=center_y, sx=size_x, sy=size_y)


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
