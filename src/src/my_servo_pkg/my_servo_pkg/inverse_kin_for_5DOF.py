import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import time
import math
import ipywidgets as widgets
import serial
import ipympl, ipywidgets
import sys


my_chain = ikpy.chain.Chain.from_urdf_file(
    "/home/dmitriyb51/Downloads/Untitled_description/urdf/Untitled.urdf",
    active_links_mask=[False, True, True, True, True, False]
)

POSITION_TOLERANCE = 0.04


ORIENTATIONS = [
    [0, 0, -1],   
    [1, 0, 0],    
    [-1, 0, 0],   
    [0, 1, 0],    
    [0, -1, 0],   
]

while True:
    A = float(input("X coordinate: "))
    B = float(input("Y coordinate: "))
    C = float(input("Z coordinate: "))

    target_position = [A, B, C]

    success = False

    for target_orientation in ORIENTATIONS:
        print(f"\nTrying orientation: {target_orientation}")

        ik = my_chain.inverse_kinematics(
            target_position,
            target_orientation,
            orientation_mode="Y"
        )

        computed_position = my_chain.forward_kinematics(ik)
        ee_pos = computed_position[:3, 3]

        print(
            "Computed position:",
            [f"{x:.3f}" for x in ee_pos]
        )

        if (
            abs(ee_pos[0] - A) <= POSITION_TOLERANCE and
            abs(ee_pos[1] - B) <= POSITION_TOLERANCE and
            abs(ee_pos[2] - C) <= POSITION_TOLERANCE
        ):
            print("Position reached successfully!")
            print(
                "Joint angles (deg):",
                list(map(lambda r: round(math.degrees(r), 2), ik.tolist()))
            )
            success = True
            break 

    if not success:
        print("position is not reachable")