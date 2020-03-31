from model.car import Car
from model import model
from model import standard_tracks as st
from visualization import game
from ai.controller import get_controller


def main():
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

    cars = [
        Car(model.Rail.Lane1, track, key_control=True),
        Car(model.Rail.Lane2, track),
    ]

    # TODO: Fix this hack.
    track.cars = cars

    controller = get_controller(track, cars[1])
    cars[1].controller = controller
    game.start_game(track)


if __name__ == '__main__':
    main()
