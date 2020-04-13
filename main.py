from model.car import Car
from model import model
from model import standard_tracks as st
from visualization import game
from ai.controller import train


def main():
    rails = [
        st.Straight('half'),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Straight('half'),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
    ]

    track = model.Track(rails, None)

    cars = [
        Car(model.Rail.Lane1, track, key_control=True),
        #Car(model.Rail.Lane2, track),
    ]

    track.cars = cars

    #controller = train(track, cars[1])
    #cars[1].controller = controller
    game.start_game(track)


if __name__ == '__main__':
    main()
