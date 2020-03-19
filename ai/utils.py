from model import standard_tracks as st
from model.car import Car
from model import model


def get_training_track():
    """ Creates a simple track for training on. """
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

    track = model.Track(rails, None)
    car = Car(model.Rail.Lane2, track)
    track.cars = [car]

    return track, car
