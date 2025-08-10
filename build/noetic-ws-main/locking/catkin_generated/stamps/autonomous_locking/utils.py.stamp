import math

import rospy

NFZ_ZONES = []  # geometry_msgs/Point (x=lat, y=lon, z=radius)
WEIGHTS = {
    "distance": 1.0,
    "tail_angle": 2.0,
    "heading_alignment": 1.5,
    "zone_score": 3.0,
}


def load_cost_weights():
    global WEIGHTS
    try:
        WEIGHTS["distance"] = rospy.get_param("~cost_weights/distance", 1.0)
        WEIGHTS["tail_angle"] = rospy.get_param("~cost_weights/tail_angle", 2.0)
        WEIGHTS["heading_alignment"] = rospy.get_param(
            "~cost_weights/heading_alignment", 1.5
        )
        WEIGHTS["zone_score"] = rospy.get_param("~cost_weights/zone_score", 3.0)
        rospy.loginfo("[Util] Ağırlıklar ROS parametreleri üstündne yüklendi.")
    except KeyError:
        rospy.logwarn(
            "[Util] Ağırlıklarlar parametrelerde bulunamadı, varsayılan değerler kullanılıyor."
        )


def set_zones(zone_list):
    global NFZ_ZONES
    NFZ_ZONES = zone_list


def compute_zone_score(target):
    tx, ty = target.pose.position.x, target.pose.position.y
    for zone in NFZ_ZONES:
        zx, zy, r = zone.x, zone.y, zone.z
        dist = math.sqrt((tx - zx) ** 2 + (ty - zy) ** 2)
        if dist < r:
            return 1.0
        elif dist < r + 0.0003:
            return 0.5
    return 0.0


def estimate_heading(current_pose, previous_pose):
    if not previous_pose:
        return 0.0
    dx = current_pose.position.x - previous_pose.position.x
    dy = current_pose.position.y - previous_pose.position.y
    if abs(dx) < 1e-5 and abs(dy) < 1e-5:
        return 0.0
    heading_rad = math.atan2(dy, dx)
    return math.degrees(heading_rad) % 360


def compute_cost(own_pose, target, previous_pose):
    dx = target.pose.position.x - own_pose.position.x
    dy = target.pose.position.y - own_pose.position.y
    dz = target.pose.position.z - own_pose.position.z
    distance = math.sqrt(dx**2 + dy**2 + dz**2)

    angle_to_target = math.degrees(math.atan2(dy, dx))
    tail_angle = abs((angle_to_target - target.heading + 360) % 360)
    if tail_angle > 180:
        tail_angle = 360 - tail_angle

    own_heading = estimate_heading(own_pose, previous_pose)
    heading_alignment = abs((own_heading - target.heading + 360) % 360)
    if heading_alignment > 180:
        heading_alignment = 360 - heading_alignment

    zone_score = compute_zone_score(target)

    norm_d = distance / 1000.0
    norm_tail = tail_angle / 180.0
    norm_align = heading_alignment / 180.0

    cost = (
        WEIGHTS.get("distance") * norm_d
        + WEIGHTS.get("tail_angle") * norm_tail
        + WEIGHTS.get("heading_alignment") * norm_align
        + WEIGHTS.get("zone_score") * zone_score
    )

    return distance, tail_angle, heading_alignment, zone_score, cost
