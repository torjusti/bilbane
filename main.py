import arcade
from model import model
from visualization import game
import math


def main():
    rails = [
        model.StraightRail(3),
        model.TurnRail(2, math.pi / 2, model.TurnRail.Left),
        model.TurnRail(1, math.pi, model.TurnRail.Left),
        model.StraightRail(3),
        model.TurnRail(1, math.pi, model.TurnRail.Right),
        model.StraightRail(2),
        model.TurnRail(1, math.pi * 3/2, model.TurnRail.Left),
    ]

    cars = [
        model.Car(model.Rail.Lane1, speed=5),
        model.Car(model.Rail.Lane2, speed=1),
    ]

    track = model.Track(rails, cars)

    game.start_game(track)


if __name__ == '__main__':
    main()
