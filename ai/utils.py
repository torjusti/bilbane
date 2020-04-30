from model import standard_tracks as st
from model.car import Car
from model import model


def get_training_track(version='sqircle'):
    """ Creates a simple track for training on. """
    if version == 'sqircle':
        rails = [
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45),
            st.Curve(2, 45),
            st.Curve(2, 45),
            st.Curve(2, 45),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45),
            st.Curve(2, 45),
            st.Curve(2, 45),
            st.Curve(2, 45),
        ]
    elif version == 'double-ellipse':
        rails = [
            # Top line of bottom part of E.
            st.Straight(),
            st.Straight(),

            # Line between E and I.
            st.Straight(),
            st.Straight(),

            # I.
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),

            # Line between I and T.
            st.Straight(),
            st.Straight(),
            st.Straight(),

            # T.
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Straight(),
            st.Straight(),
            st.Straight(),

            # Transfer back to the beginning.
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),



            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Left),

            # E.
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Straight(),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Straight(),
            st.Straight(),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),

            # Patching.
            st.Straight(),
            st.Straight('half'),
        ]

    track = model.Track(rails, None)
    car = Car(model.Rail.Lane2, track)
    track.cars = [car]

    return track, car
