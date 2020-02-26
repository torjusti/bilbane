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

    cars = [
        model.Car(model.Rail.Lane1, speed=1, key_control=True),
        model.Car(model.Rail.Lane2, speed=1),
    ]

    track = model.Track(rails, cars)

    game.start_game(track)


if __name__ == '__main__':
    main()
