"""Moving obstacles of the dynamic scenarios: their shapes, their SDF and their topics."""

MASS_KG = 30.0
FLOOR_CLEARANCE_M = 0.02

SHAPES = {
    'person': {'kind': 'cylinder', 'radius': 0.20, 'height': 1.70, 'colour': (0.85, 0.35, 0.10)},
    'cart': {'kind': 'box', 'size': (0.60, 0.40, 0.90), 'colour': (0.15, 0.45, 0.80)},
}


def shape_height(shape):
    """Return the height of a named shape in metres."""
    spec = SHAPES[shape]
    if spec['kind'] == 'cylinder':
        return spec['height']
    return spec['size'][2]


def spawn_height(shape):
    """Return the z of the model origin: half the height plus a small gap above the floor."""
    return shape_height(shape) / 2.0 + FLOOR_CLEARANCE_M


def obstacle_topics(name):
    """Return the Gazebo topics (velocity command, odometry) of the obstacle called name."""
    return '/model/{}/cmd_vel'.format(name), '/model/{}/odometry'.format(name)


def inertia(shape, mass=MASS_KG):
    """Return (ixx, iyy, izz) of the solid shape, from its centre of mass."""
    spec = SHAPES[shape]
    if spec['kind'] == 'cylinder':
        r, h = spec['radius'], spec['height']
        side = mass * (3.0 * r * r + h * h) / 12.0
        return side, side, mass * r * r / 2.0
    sx, sy, sz = spec['size']
    return (mass * (sy * sy + sz * sz) / 12.0,
            mass * (sx * sx + sz * sz) / 12.0,
            mass * (sx * sx + sy * sy) / 12.0)


def _geometry(spec):
    if spec['kind'] == 'cylinder':
        return '<cylinder><radius>{}</radius><length>{}</length></cylinder>'.format(
            spec['radius'], spec['height'])
    return '<box><size>{} {} {}</size></box>'.format(*spec['size'])


def obstacle_sdf(name, shape, gravity=False):
    """Return the SDF of a movable obstacle: a body that follows a velocity command."""
    spec = SHAPES[shape]
    cmd_topic, odom_topic = obstacle_topics(name)
    ixx, iyy, izz = inertia(shape)
    colour = '{} {} {} 1'.format(*spec['colour'])
    geometry = _geometry(spec)
    return (
        '<sdf version="1.9"><model name="{name}">'
        '<link name="body">'
        '<gravity>{gravity}</gravity>'
        '<inertial><mass>{mass}</mass><inertia>'
        '<ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>'
        '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
        '<collision name="collision"><geometry>{geometry}</geometry></collision>'
        '<visual name="visual"><geometry>{geometry}</geometry>'
        '<material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>'
        '</link>'
        '<plugin filename="gz-sim-velocity-control-system" '
        'name="gz::sim::systems::VelocityControl">'
        '<topic>{cmd}</topic></plugin>'
        '<plugin filename="gz-sim-odometry-publisher-system" '
        'name="gz::sim::systems::OdometryPublisher">'
        '<odom_frame>world</odom_frame>'
        '<robot_base_frame>{name}</robot_base_frame>'
        '<odom_publish_frequency>30</odom_publish_frequency>'
        '<odom_topic>{odom}</odom_topic>'
        '<tf_topic>/model/{name}/tf</tf_topic>'
        '<dimensions>3</dimensions></plugin>'
        '</model></sdf>'
    ).format(name=name, gravity='true' if gravity else 'false', mass=MASS_KG,
             ixx=ixx, iyy=iyy, izz=izz, geometry=geometry, colour=colour,
             cmd=cmd_topic, odom=odom_topic)
