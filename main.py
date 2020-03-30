from model.point_mass_car import PointMassCar
from model.rigid_body_car import RigidBodyCar
from model import model
from model import standard_tracks as st
from visualization import game
from ai.controller import train


def main():
    """
    rails = [
        st.Straight("std"),
        st.Straight("std"),
        st.Straight("std"),
        st.Straight("std"),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Right),
        st.Curve(2, 45, direction=st.Curve.Right),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Straight("std"),
        st.Straight("std"),
        st.Straight("std"),
        st.Straight("std"),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Right),
        st.Curve(2, 45, direction=st.Curve.Right),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
    ]
    """
    rails = [
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
        st.Curve(2, 45, direction=st.Curve.Left),
    ]

    track = model.Track(rails, None)

    cars = [
        RigidBodyCar(model.Rail.Lane1, track, key_control=True),
        #PointMassCar(model.Rail.Lane2, track),
    ]

    track.cars = cars

    #controller = train(track, cars[1])
    #cars[1].controller = controller
    game.start_game(track)


if __name__ == '__main__':
    main()
