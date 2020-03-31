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
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Right),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
            st.Curve(2, 45, direction=st.Curve.Left),
        ]

    track = model.Track(rails, None)
    car = Car(model.Rail.Lane2, track)
    track.cars = [car]

    return track, car
