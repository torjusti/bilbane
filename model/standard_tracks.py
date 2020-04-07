import model.model as model

import numpy as np

# Dimensions
# https://lh3.googleusercontent.com/pVPVstEa1A2aRGYhXy0n7XdTSklC340KJW0KCjGxp4Rdy0fx3THsYeeK9t4nP-BrsWsZk5aIyoGvs-9odcRnZq50DGvW7zCRTZOpXgZ5G2gkMLnIxQqC3VVdPcz39cDnSlg9XSBt
RAIL_WIDTH = model.StraightRail.RAIL_WIDTH
# RADIUS_EXTRA plus half the RAIL_WIDTH is the radius of a radius-1 turn.
RADIUS_EXTRA = 0.214 - RAIL_WIDTH

class Straight(model.StraightRail):
    length = 0.350  # meter

    names = {"std": 1, "half": 0.5, "quarter": 0.25, "short": RADIUS_EXTRA / length}

    def __init__(self, fraction=1):
        if type(fraction) is str:
            fraction = self.names[fraction]
        super().__init__(self.length * fraction)


class Curve(model.TurnRail):
    def __init__(self, curve=2, angle=45, direction=model.TurnRail.Left):
        """
        :param curve: integer from 1-4.
        :param angle: in degrees
        :param direction: TurnRail.Left or Right
        """
        assert curve in range(1, 5)
        super().__init__(RADIUS_EXTRA + (curve - .5) * RAIL_WIDTH, angle / 180 * np.pi, direction)
