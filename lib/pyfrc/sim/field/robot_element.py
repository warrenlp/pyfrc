from .elements import CompositeElement, DrawableElement

import math


class RobotElement(CompositeElement):
    """
        TODO: allow user customization
    """

    def __init__(self, controller, config_obj):

        super().__init__()

        self.sim_type = config_obj["pyfrc"]["sim_type"]

        # Load params from the user's sim/config.json
        px_per_ft = config_obj["pyfrc"][self.sim_type]["field"]["px_per_ft"]
        self.field_height = config_obj["pyfrc"][self.sim_type]["field"]["h"]
        self.field_drawing_margin = config_obj["field_drawing_margin"]

        robot_w = config_obj["pyfrc"][self.sim_type]["robot"]["w"]
        robot_l = config_obj["pyfrc"][self.sim_type]["robot"]["l"]
        center_x = config_obj["pyfrc"][self.sim_type]["robot"]["starting_x"]
        center_y = config_obj["pyfrc"][self.sim_type]["robot"]["starting_y"]
        # For the profile simulation the y-axis points up which is opposite of the tkinter y-axis.
        if self.sim_type == "profile":
            center_y = self.field_height - center_y
        angle = math.radians(config_obj["pyfrc"][self.sim_type]["robot"]["starting_angle"])

        self.controller = controller
        self.controller.robot_face = 0
        self.px_per_ft = px_per_ft

        robot_w *= px_per_ft
        robot_l *= px_per_ft
        center_x *= px_per_ft
        center_y *= px_per_ft

        # drawing hack
        self._vector = (center_x, center_y, angle)

        # create a bunch of drawable objects that represent the robot
        center = (center_x, center_y)
        pts = [
            (center_x - robot_w / 2, center_y - robot_l / 2),
            (center_x + robot_w / 2, center_y - robot_l / 2),
            (center_x + robot_w / 2, center_y + robot_l / 2),
            (center_x - robot_w / 2, center_y + robot_l / 2),
        ]

        pts = [(pt[0] + self.field_drawing_margin, pt[1] + self.field_drawing_margin) for pt in pts]

        robot = DrawableElement(pts, center, 0, "red")
        self.elements.append(robot)

        pts = [
            (center_x - robot_w / 2, center_y - robot_l / 2),
            (center_x + robot_w / 2, center_y),
            (center_x - robot_w / 2, center_y + robot_l / 2),
        ]

        pts = [(pt[0] + self.field_drawing_margin, pt[1] + self.field_drawing_margin) for pt in pts]

        robot_pt = DrawableElement(pts, center, 0, "green")
        self.elements.append(robot_pt)

        if angle != 0:
            self.rotate(angle)

        # Add peripherals to robot in profile simulation if they are included with robot
        if self.sim_type == "profile":
            objects = config_obj["pyfrc"][self.sim_type]["robot"].get("objects")

            if objects:
                self.peripherals = {}

            for obj in objects:
                name = obj["name"]
                color = obj.get("color", "gray")
                elem_center = obj["center"]
                pts = [[pt_x * self.px_per_ft, pt_y * self.px_per_ft] for pt_x, pt_y in obj["points"]]
                elem = DrawableElement(pts, elem_center, 0, color)
                self.peripherals[name] = [elem, (0.0, 0.0, 0.0)]

    @property
    def angle(self):
        return self._vector[2]

    @property
    def front_center(self):
        x, y = self.elements[1].pts[1]
        return x, y

    @property
    def center(self):
        return self.elements[1].center

    def initialize(self, canvas):
        super().initialize(canvas)
        if hasattr(self, 'peripherals'):
            for name in self.peripherals:
                e, starting_vector = self.peripherals[name]
                e.initialize(canvas)
                self.controller.register_element(name, starting_vector)

    def perform_move(self):

        if not self.controller.is_alive():
            self.elements[1].set_color("gray")

        # query the controller for move information
        self.move_robot()

        # Call the superclass to actually do the drawing
        self.update_coordinates()

        if hasattr(self, 'peripherals'):
            self.move_peripherals()
            self.update_peripheral_coordinates()

    def move_robot(self):

        vx, vy, a = self.controller._get_vector()  # units: ft
        ox, oy, oa = self._vector  # units: px

        if self.sim_type == "profile":
            vy = self.field_height - vy

        vx *= self.px_per_ft
        vy *= self.px_per_ft

        dx = vx - ox
        dy = vy - oy
        da = a - oa

        if da != 0:
            self.rotate(da)

        self.move((dx, dy))

        self._vector = vx, vy, a

    def move_peripherals(self):

        for name in self.peripherals:
            e, position_vector = self.peripherals[name]  # Element position, units: px
            ox, oy, oa = position_vector
            x, y, a = self.controller._get_vector(name)  # Robot/PhysicsController position, units: ft

            x *= self.px_per_ft
            y *= self.px_per_ft

            dx = x - ox
            dy = y - oy
            da = a - oa

            if da != 0:
                e.rotate(da)

            e.move((dx, dy))

            self.peripherals[name][1] = x, y, a

    def update_peripheral_coordinates(self):
        for name in self.peripherals:
            e, _ = self.peripherals[name]
            e.update_coordinates()
