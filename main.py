from model.car import Car
from model import model
from model import standard_tracks as st
from visualization import game


def main():

    rails = [
        st.Straight(),
        st.Straight(),
        st.Curve(1, 45, direction=st.Curve.Left),
        st.Curve(1, 45, direction=st.Curve.Left),
        st.Straight("short"),
        st.Curve(1, 45, direction=st.Curve.Left),
        st.Curve(1, 45, direction=st.Curve.Left),
        st.Straight(),
        st.Straight(),
        st.Curve(1, 45, direction=st.Curve.Left),
        st.Curve(1, 45, direction=st.Curve.Left),
        st.Straight("short"),
        st.Curve(1, 45, direction=st.Curve.Left),
        st.Curve(1, 45, direction=st.Curve.Left),
    ]

    track = model.Track(rails, None)

    cars = [
        Car(model.Rail.Lane1, track, key_control=True),
        Car(model.Rail.Lane2, track),
    ]

    track.cars = cars

    game.start_game(track)


if __name__ == '__main__':
    main()
