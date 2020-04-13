from model.car import Car
from model import model
from model import standard_tracks as st
from visualization import game
from ai.controller import train
from parameters import parameterestimation as pe

def main():
    rails = [
        st.Straight("half"),
        st.Curve(),
        st.Curve(),
        st.Curve(),
        st.Curve(),
        st.Straight("half"),
        st.Curve(),
        st.Curve(),
        st.Curve(),
        st.Curve(),
    ]

    track = model.Track(rails, None)

    cars = [
        Car(model.Rail.Lane1, track, key_control=False),
        #Car(model.Rail.Lane2, track),
    ]
    
    track.cars = cars
    
    #test = pe.DataGathering(track, "parameters/parameter_estimation5.txt")
    #test.run()

    # controller = train(track, cars[1])
    # cars[1].controller = controller
    #Find best fit of parameters
    #best_fit = pe.ParameterEstimation("parameters/parameter_estimation5.txt")
    #best_fit.run()
    cent_froce_est = pe.EstimateMaxCentrifugalForce(track)
    cent_froce_est.run()


if __name__ == '__main__':
    main()
