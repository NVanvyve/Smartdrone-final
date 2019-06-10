from __future__ import print_function

import lib_trilateration_py
from enum import IntEnum, unique

@unique
class TrilaterationOutput(IntEnum):
    # Integer values for the output of the trilateration function
    TRIL_3SPHERES = 3
    TRIL_4SPHERES = 4
    # Errors < 0
    ERR_TRIL_CONCENTRIC = -1
    ERR_TRIL_COLINEAR_2SOLUTIONS = -2
    ERR_TRIL_SQRTNEGNUMB = -3


def calculate_tag_location(count, ranges, anchor_positions):
    result, best_solution = lib_trilateration_py.get_location(count >= 4, anchor_positions, ranges)
    return TrilaterationOutput(result), best_solution


def compute(report, anchor_positions):
    """
    Tries to update the best solution
    :param report: tag range report object
    :param anchor_positions:
    :return: best solution. Throw ValueError exception if cannot find one
    """

    ranges = [0] * 4

    for (unit, distance) in report.unit_and_ranges:
        ranges[unit] = distance

    if len(report.units) >= 3:
        result, best_solution = calculate_tag_location(len(report.units), ranges, anchor_positions)
        if result == TrilaterationOutput.TRIL_3SPHERES:
            return best_solution
        else:
            raise ValueError(result)
    else:
        raise ValueError("Not enough anchors detected!")
