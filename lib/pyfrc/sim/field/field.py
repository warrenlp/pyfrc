from os.path import exists
from tkinter import PhotoImage

import tkinter as tk

from .elements import DrawableElement


class RobotField(object):
    def __init__(self, root, manager, config_obj):
        """
            initializes all default values and creates
            a board, waits for run() to be called
            to start the board

            manager - sim manager class instance
            board_size - a tuple with values (rows, cols)
        """

        # TODO: support drawing an actual field?

        self.manager = manager
        self.elements = []  # robots, walls, missles, etc

        sim_type = config_obj["pyfrc"]["sim_type"]

        field_size = (
            config_obj["pyfrc"][sim_type]["field"]["w"],
            config_obj["pyfrc"][sim_type]["field"]["h"],
        )
        px_per_ft = config_obj["pyfrc"][sim_type]["field"]["px_per_ft"]

        # setup board characteristics -- cell size is 1ft
        self.cols, self.rows = field_size
        self.margin = config_obj["field_drawing_margin"]
        self.objects = config_obj["pyfrc"][sim_type]["field"]["objects"]
        self.cellSize = px_per_ft
        self.px_per_ft = px_per_ft

        self.canvasWidth = 2 * self.margin + self.cellSize * self.cols
        self.canvasHeight = 2 * self.margin + self.cellSize * self.rows

        self.canvas = tk.Canvas(root, width=self.canvasWidth, height=self.canvasHeight)
        self.canvas.bind("<Key>", self.on_key_pressed)
        self.canvas.bind("<Button-1>", self.on_click)

        self.text_id = None

        # Draw the field initially
        self.draw_field()

        # Load elements from the config
        # -> This probably belongs somewhere else
        self._load_field_elements(px_per_ft, config_obj)

    def _load_field_elements(self, px_per_ft, config_obj):

        sim_type = config_obj["pyfrc"]["sim_type"]

        # custom image
        image_path = config_obj["pyfrc"][sim_type]["field"]["image"]

        if image_path and exists(image_path):
            self.photo = PhotoImage(file=image_path)
            self.canvas.create_image(
                (self.canvasWidth / 2, self.canvasHeight / 2), image=self.photo
            )

        if self.objects:
            for obj in self.objects:
                color = obj["color"]
                rect = obj.get("rect")
                if rect:
                    x, y, w, h = rect
                    obj["points"] = [(x, h + y), (x + w, h + y), (x + w, y), (x, y)]
                pts = [
                    (
                        self.margin + int(pt[0] * px_per_ft),
                        self.margin + int((self.rows - pt[1]) * px_per_ft),
                    )
                    for pt in obj["points"]
                ]

                element = DrawableElement(pts, None, None, color)
                self.add_moving_element(element)

    def add_moving_element(self, element):
        """Add elements to the board"""

        element.initialize(self.canvas)
        self.elements.append(element)

    def grid(self, *args, **kwargs):
        self.canvas.grid(*args, **kwargs)

    def on_key_pressed(self, event):
        """
            likely to take in a set of parameters to treat as up, down, left,
            right, likely to actually be based on a joystick event... not sure
            yet
        """

        return

        # TODO

        if event.keysym == "Up":
            self.manager.set_joystick(0.0, -1.0, 0)
        elif event.keysym == "Down":
            self.manager.set_joystick(0.0, 1.0, 0)
        elif event.keysym == "Left":
            self.manager.set_joystick(-1.0, 0.0, 0)
        elif event.keysym == "Right":
            self.manager.set_joystick(1.0, 0.0, 0)

        elif event.char == " ":
            mode = self.manager.get_mode()
            if mode == self.manager.MODE_DISABLED:
                self.manager.set_mode(self.manager.MODE_OPERATOR_CONTROL)
            else:
                self.manager.set_mode(self.manager.MODE_DISABLED)

    def on_click(self, event):
        self.canvas.focus_set()

    def update_widgets(self):

        # TODO: process collisions and such too

        for element in self.elements:
            element.perform_move()

    def draw_field(self):
        for row in range(self.rows):
            for col in range(self.cols):
                self.draw_board_cell(row, col)

    def draw_board_cell(self, row, col):
        left = self.margin + col * self.cellSize
        right = left + self.cellSize
        top = self.margin + row * self.cellSize
        bottom = top + self.cellSize
        self.canvas.create_rectangle(
            left, top, right, bottom, outline="#ccc", fill="#fff"
        )
