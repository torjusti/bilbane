if __name__ == "__main__":
    # Doing some testing here

    # ----------------------------------------------
    # Test set-up

    # 1st track

    # Define rails of 1st test track (starts with straight)
    test_rails = [
        model.StraightRail(200),
        model.TurnRail(100, np.pi, model.TurnRail.Left),
        model.TurnRail(25, 2 * np.pi, model.TurnRail.Right),
        model.StraightRail(200),
        model.TurnRail(100, np.pi, model.TurnRail.Left)
    ]

    # Define test track, without cars
    test_track = model.Track(test_rails, None)

    # Define test cars
    test_cars = [
        Car(model.Rail.Lane1, test_track),
        Car(model.Rail.Lane2, test_track)
    ]

    # Put test cars on test track
    test_track.cars = test_cars

    # 2nd track

    # Define rails of test track
    test_rails_2 = [
        model.TurnRail(2, 2 * np.pi, model.TurnRail.Right)
    ]

    # Define test track, without cars
    test_track_2 = model.Track(test_rails_2, None)

    # Define test cars
    test_cars_2 = [
        Car(model.Rail.Lane1, test_track_2),
        Car(model.Rail.Lane2, test_track_2)
    ]

    # Put test cars on test track
    test_track_2.cars = test_cars_2

    # 3nd track

    # Define rails of test track
    test_rails_3 = [
        model.TurnRail(2, 2 * np.pi, model.TurnRail.Left)
    ]

    # Define test track, without cars
    test_track_3 = model.Track(test_rails_3, None)

    # Define test cars
    test_cars_3 = [
        Car(model.Rail.Lane1, test_track_3),
        Car(model.Rail.Lane2, test_track_3)
    ]

    # Put test cars on test track
    test_track_3.cars = test_cars_3

    # ----------------------------------------------------------
    # Testing force calculations

    print("PRINTING PROPERTIES OF CAR 1")
    print("Mass:   ", test_track.cars[0].mass)
    print("Area:   ", test_track.cars[0].area)
    print("C_drag: ", test_track.cars[0].drag_coeff)
    print("C_mag:  ", test_track.cars[0].mag_coeff)
    print("eta:    ", test_track.cars[0].motor_eta)
    print("mu_tire:", test_track.cars[0].mu_tire)
    print("mu_pin: ", test_track.cars[0].mu_pin)
    print("mu_roll:", test_track.cars[0].mu_roll)
    print("")

    print("RESISTANCE TEST")
    print("Resistances should be roughly [39.1 39.1]")
    print(test_track.resistances)
    print("Okay!\n")

    print("MAGNET FORCE TEST")
    print("Magnet force should be roughly [0 0 -1]")
    print(test_track.cars[0].get_magnet_force())
    print("Okay!\n")

    print("GRAVITATIONAL FORCE TEST")
    print("Gravitational force should be roughly [0 0 -.7848]")
    print(test_track.cars[0].get_gravity_force())
    print("Okay!\n")

    print("NORMAL FORCE TEST")
    print("Normal force should be roughly [0 0 1.7848]")
    print(test_track.cars[0].get_normal_force())
    print("Okay!\n")

    print("ROLLING RESISTANCE TEST")
    print("Rolling resistance should be roughly [-0.017848 0 0]")
    print(test_track.cars[0].get_rolling_resistance())
    print("Okay!\n")

    print("LATERAL FRICTION TEST 1 -- STRAIGHT")
    print("Lateral friction force on a straight should be [0 0 0]")
    print(test_track.cars[0].get_lateral_friction())
    print("Okay!\n")

    print("LATERAL FRICTION TEST 2 -- RIGHT TURN")
    print("Lateral friction force on a straight should be [0 0 0]")
    print(test_track_2.cars[0].get_lateral_friction())
    print("Okay!\n")

    print("LATERAL FRICTION TEST 3 -- LEFT TURN")
    print("Lateral friction force on a straight should be [0 0 0]")
    print(test_track_3.cars[0].get_lateral_friction())
    print("Okay!\n")

    test_track_2.cars[0].vel_vec = np.asarray([10, 2, 3])
    test_track_3.cars[0].vel_vec = np.asarray([10, 2, 3])

    print("LATERAL FRICTION TEST 4 -- RIGHT TURN")
    print("Lateral friction force on a straight should be [0 -1.606 0]")
    print(test_track_2.cars[0].get_lateral_friction())
    print("Okay!\n")

    print("LATERAL FRICTION TEST 5 -- LEFT TURN")
    print("Lateral friction force on a straight should be [0 1.606 0]")
    print(test_track_3.cars[0].get_lateral_friction())
    print("Okay!\n")

    print("THRUST FORCE TEST")
    test_track.cars[0].controller_input = 0.25
    print("Thrust force should be roughly [0.39 0 0]")
    print(test_track.cars[0].get_thrust_force())
    print("Okay!\n")

    print("DRAG FORCE TEST")
    test_track.cars[0].vel_vec = np.asarray([0.2, -.5, .7])
    print("Drag force should be roughly [-0.00037 0 0]")
    print(test_track.cars[0].get_drag_force())
    print("Okay!\n")

    print("LATERAL PIN FORCE TEST 1 -- STRAIGHT")
    print("Lateral pin force should be [0 0 0]")
    print(test_track.cars[0].get_lateral_pin_force())
    print("Okay!\n")

    test_track_2.cars[0].vel_vec = np.asarray([1, 2, 3])
    test_track_3.cars[0].vel_vec = np.asarray([1, 2, 3])

    print("LATERAL PIN FORCE TEST 2 -- RIGHT TURN")
    print("Lateral pin force should be [0 0 0]")
    print(test_track_2.cars[0].get_lateral_pin_force())
    print("Okay!\n")

    print("LATERAL PIN FORCE TEST 3 -- LEFT TURN")
    print("Lateral pin force should be [0 0 0]")
    print(test_track_3.cars[0].get_lateral_pin_force())
    print("Okay!\n")

    test_track_2.cars[0].vel_vec = np.asarray([10, 2, 3])
    test_track_3.cars[0].vel_vec = np.asarray([10, 2, 3])

    print("LATERAL PIN FORCE TEST 4 -- RIGHT TURN")
    print("Lateral pin force should be roughly [0 -2.83 0]")
    print(test_track_2.cars[0].get_lateral_pin_force())
    print("Okay!\n")

    print("LATERAL PIN FORCE TEST 5 -- LEFT TURN")
    print("Lateral pin force should be roughly [0 3.00 0]")
    print(test_track_3.cars[0].get_lateral_pin_force())
    print("Okay!\n")

    print("PIN FRICTION FORCE TEST")
    print("Pin friction force should be roughly [-0.12 0 0]")
    print(test_track_3.cars[0].get_pin_friction())
    print("Okay\n")

    print("TOTAL FORCE TEST")
    print("Total force should be roughly [-0.19 4.6 0]")
    #print("Rolling resistance:", test_track_3.cars[0].get_rolling_resistance())
    #print("Pin friction:",       test_track_3.cars[0].get_pin_friction())
    #print("Lateral friction:",   test_track_3.cars[0].get_lateral_friction())
    #print("Magnet force:",       test_track_3.cars[0].get_magnet_force())
    #print("Gravity force:",      test_track_3.cars[0].get_gravity_force())
    #print("Normal force:",       test_track_3.cars[0].get_normal_force())
    #print("Thrust force:",       test_track_3.cars[0].get_thrust_force())
    #print("Drag force:",         test_track_3.cars[0].get_drag_force())
    #print("Lateral pin force:",  test_track_3.cars[0].get_lateral_pin_force())
    print("Total force:",        test_track_3.cars[0].get_total_force())
    print("Okay\n")

    # ----------------------------------------------------------------------------
    # Testing state update functions

    test_track_3.cars[0].vel_vec = np.asarray([10, 0, 0])
    print(test_track_3.cars[0].pos_vec)
    for i in range(1001):
        test_track_3.cars[0].pos_vec, test_track_3.cars[0].vel_vec, angle_vec= test_track_3.cars[0].get_new_state(0.001)
        if (i%100 == 0):
            print("i=",i)
            print("New position: ", test_track_3.cars[0].pos_vec)
            print("New velocity: ", test_track_3.cars[0].vel_vec)