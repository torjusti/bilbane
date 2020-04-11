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
    test = pe.ParameterEstimation(track, "test_2")
    test.run()
    # controller = train(track, cars[1])
    # cars[1].controller = controller
    


if __name__ == '__main__':
    main()
