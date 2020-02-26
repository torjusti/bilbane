from model.car import Car
from model import model
from visualization import game
import math
from ai.controller import train


def main():
    rails = [
        model.StraightRail(300),
        model.TurnRail(200, math.pi / 2, model.TurnRail.Left),
        model.TurnRail(100, math.pi, model.TurnRail.Left),
        model.StraightRail(300),
        model.TurnRail(100, math.pi, model.TurnRail.Right),
        model.StraightRail(200),
        model.TurnRail(100, math.pi * 3/2, model.TurnRail.Left),
    ]

    track = model.Track(rails, None)

    cars = [
        Car(model.Rail.Lane1, track, key_control=True),
        Car(model.Rail.Lane2, track),
    ]

    track.cars = cars

    controller = train(track, cars[1])
    cars[1].controller = controller
    game.start_game(track)

if __name__ == '__main__':
    main()
