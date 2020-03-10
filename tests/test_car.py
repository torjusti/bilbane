import unittest

import numpy as np

from model import model
from model.car import Car, G_ACC
from tests.utils import TestCaseExtra


class CarTest(TestCaseExtra):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.test_track_1 = self.test_track1()
        self.test_track_2 = self.test_track2()
        self.test_track_3 = self.test_track3()
        self.tracks = [self.test_track_1, self.test_track_2, self.test_track_3]

    def test_track1(self):
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
        return test_track

    def test_track2(self):
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
        return test_track_2

    def test_track3(self):
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
        return test_track_3

    def test_resistance_force(self):
        # RESISTANCE TEST
        self.assertAlmostEqualVector([39.1, 39.1], self.test_track_1.resistances, 1, "Test resistances.")

    def test_magnet_force(self):
        # MAGNET FORCE TEST
        self.assertAlmostEqualVector([0, 0, -1], self.test_track_1.cars[0].get_magnet_force(), 1, "Test magnet force.")

    def test_gravity_force(self):
        # GRAVITATIONAL FORCE TEST
        for track in self.tracks:
            for car in track.cars:
                gravity_force = self.test_track_1.cars[0].mass * G_ACC
                self.assertAlmostEqualVector([0, 0, -gravity_force], car.get_gravity_force(), 4,
                                             "Test gravity force.")

    def test_normal_force(self):
        # NORMAL FORCE TEST
        for track in self.tracks:
            for car in track.cars:
                normal_force = car.mag_coeff + car.mass * G_ACC
                self.assertAlmostEqualVector([0, 0, normal_force], car.get_normal_force(),
                                             4, "Test normal force.")

    def test_rolling_resistance(self):
        # ROLLING RESISTANCE TEST
        for track in self.tracks:
            for car in track.cars:
                # car: Car = self.test_track1().cars[0]
                N = car.mag_coeff + car.mass * G_ACC
                rolling_rest = - car.mu_roll * N
                velocity = np.array([1, 2, 3])
                car.vel_vec = velocity
                self.assertAlmostEqualVector([rolling_rest, 0, 0], car.get_rolling_resistance(), 6,
                                             "Test rolling resistance.")

    def test_lateral_friction_force(self):
        # LATERAL FRICTION TEST 1 -- STRAIGHT
        self.assertAlmostEqualVector([0, 0, 0], self.test_track_1.cars[0].get_lateral_friction(), 4,
                                     "Test lateral friction.")

        # LATERAL FRICTION TEST 2 -- RIGHT TURN
        self.assertAlmostEqualVector([0, 0, 0], self.test_track_2.cars[0].get_lateral_friction(), 4,
                                     "Test lateral friction.")

        # LATERAL FRICTION TEST 3 -- LEFT TURN
        self.assertAlmostEqualVector([0, 0, 0], self.test_track_3.cars[0].get_lateral_friction(), 4,
                                     "Test lateral friction.")

        self.test_track_2.cars[0].vel_vec = np.asarray([10, 2, 3])
        self.test_track_3.cars[0].vel_vec = np.asarray([10, 2, 3])

        # LATERAL FRICTION TEST 4 -- RIGHT TURN
        self.assertAlmostEqualVector([0, -1.606, 0], self.test_track_2.cars[0].get_lateral_friction(), 3,
                                     "Test lateral friction.")

        # LATERAL FRICTION TEST 5 -- LEFT TURN
        self.assertAlmostEqualVector([0, 1.606, 0], self.test_track_3.cars[0].get_lateral_friction(), 3,
                                     "Test lateral friction.")

    def test_thrust_force(self):
        # THRUST FORCE TEST
        for track in self.tracks:
            for car in track.cars:
                car.controller_input = 0.25
                velocity = np.array([1, 2, 3])
                car.vel_vec = velocity
                thrust = car.controller_input * car.max_power / np.linalg.norm(velocity)
                self.assertAlmostEqualVector([thrust, 0, 0], car.get_thrust_force(), 4, "Test thrust force.")

    def test_drag_force(self):
        # DRAG FORCE TEST
        test_track = self.test_track_1
        test_track.cars[0].vel_vec = np.asarray([0.2, -.5, .7])
        self.assertAlmostEqualVector([-0.00037, 0, 0], test_track.cars[0].get_drag_force(), 5, "Test drag force.")

    def test_lateral_pin_force(self):
        # LATERAL PIN FORCE TEST 1 -- STRAIGHT
        self.assertAlmostEqualVector([0, 0, 0], self.test_track_1.cars[0].get_lateral_pin_force(), 4,
                                     "Test lateral pin force.")

        self.test_track_2.cars[0].vel_vec = np.asarray([1, 2, 3])
        self.test_track_3.cars[0].vel_vec = np.asarray([1, 2, 3])

        # LATERAL PIN FORCE TEST 2 -- RIGHT TURN
        self.assertAlmostEqualVector([0, 0, 0], self.test_track_2.cars[0].get_lateral_pin_force(), 4,
                                     "Test lateral pin force.")

        # LATERAL PIN FORCE TEST 3 -- LEFT TURN
        self.assertAlmostEqualVector([0, 0, 0], self.test_track_3.cars[0].get_lateral_pin_force(), 4,
                                     "Test lateral pin force.")

        self.test_track_2.cars[0].vel_vec = np.asarray([10, 2, 3])
        self.test_track_3.cars[0].vel_vec = np.asarray([10, 2, 3])

        # LATERAL PIN FORCE TEST 4 -- RIGHT TURN
        self.assertAlmostEqualVector([0, -2.83, 0], self.test_track_2.cars[0].get_lateral_pin_force(), 2,
                                     "Test lateral pin force.")

        # LATERAL PIN FORCE TEST 5 -- LEFT TURN

        self.assertAlmostEqualVector([0, 3.00, 0], self.test_track_3.cars[0].get_lateral_pin_force(), 2,
                                     "Test lateral pin force.")

    def test_pin_friction_force(self):
        # PIN FRICTION FORCE TEST
        self.test_track_3.cars[0].vel_vec = np.asarray([10, 2, 3])
        self.assertAlmostEqualVector([-0.12, 0, 0], self.test_track_3.cars[0].get_pin_friction(), 2,
                                     "Test lateral pin force.")

    def test_state_update_functions(self):
        """
        :return:
        # Testing state update functions
        test_track_3.cars[0].vel_vec = np.asarray([10, 0, 0])
        print(test_track_3.cars[0].pos_vec)
        for i in range(1001):
            test_track_3.cars[0].pos_vec, test_track_3.cars[0].vel_vec, angle_vec= test_track_3.cars[0].get_new_state(0.001)
            if (i%100 == 0):
                print("i=",i)
                print("New position: ", test_track_3.cars[0].pos_vec)
                print("New velocity: ", test_track_3.cars[0].vel_vec)
        """
        pass


if __name__ == '__main__':
    unittest.main()
