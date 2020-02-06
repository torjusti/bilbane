import arcade
from model import model
from visualization import game
import math

def main():
    rails = [
        model.StraightRail(200),
        model.TurnRail(100, math.pi),
        model.StraightRail(200),
        model.TurnRail(100, math.pi),
    ]

    cars = [
        model.Car(),
    ]

    track = model.Track(rails, cars)

    game.start_game(track)


if __name__ == '__main__':
    main()
