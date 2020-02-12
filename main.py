import arcade
from model import model
from visualization import game
import math


def main():
    rails = [
        model.StraightRail(100),
        model.TurnRail(100, math.pi/2, model.TurnRail.Left),
        model.StraightRail(100),
        model.TurnRail(100, math.pi/2, model.TurnRail.Left),
        model.StraightRail(100),
        model.TurnRail(100, math.pi/2, model.TurnRail.Left),
        model.StraightRail(100),
        model.TurnRail(100, math.pi/2, model.TurnRail.Left)
        
    ]

    cars = [
        model.Car(model.Rail.Lane1, speed=500),
        model.Car(model.Rail.Lane2, speed=100),
    ]

    track = model.Track(rails, cars)

    game.start_game(track)


if __name__ == '__main__':
    main()
