from model.car import Car
from model import model
from model import standard_tracks as st
from visualization import game
from parameters import parameterestimation as pe
from ai.controller import get_controller


def main():
    rails_eit = [
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
        Car(model.Rail.Lane2, track, key_control=True),
        Car(model.Rail.Lane2, track),
    ]

    # TODO: Fix this hack.
    track.cars = cars

    controller = get_controller(track, cars[1])
    cars[1].controller = controller
    game.start_game(track)

    #test = pe.DataGathering(track, "parameters/parameter_estimation5.txt")
    #test.run()

    #Find best fit of parameters
    #best_fit = pe.ParameterEstimation("parameters/quick_test5.txt")
    #best_fit.run()
    #best_fit = pe.ParameterEstimation("parameters/parameter_estimation5.txt")
    #best_fit.run()
    #cent_froce_est = pe.EstimateMaxCentrifugalForce(track)
    #cent_froce_est.run()

if __name__ == '__main__':
    main()
