from dataclasses import dataclass

import taichi as ti


@ti.dataclass
class SlmBlocker:
    """
    A data structure to represent a blocker in the Social Locomotion Model. Used under taichi framework.
    :param is_human: 1 if the blocker is a human, 0 if it is an obstacle @param position: the position of the blocker.
                       Constructed using ti.Vector([x, y])
    :param a: parameter a in the SIFM model @param b: parameter b in the SIFM model
    :param c: parameter c in the SIFM model Note that if the blocker is human, it should be set to SIFM.c,
               otherwise it should be set to 6.76
    :param orientation_rad: the orientation of the blocker in radians. Only used when the blocker is a human.
    """
    is_human: ti.i32
    position: ti.math.vec2
    a: ti.f32
    b: ti.f32
    c: ti.f32
    orientation_rad: ti.f32


@dataclass
class SlmSimMetadata:
    """
    A data structure to represent the metadata of a simulation in the Social Locomotion Model.
    """
    level: int = 0
    trial_index: int = 0
    first_gen_time: float = 0.0
    has_collision: bool = False
    second_gen_time: float = 0.0
    final_path_length: float = 0.0
