from util.orientation import Orientation, relative_location
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.ball_prediction_analysis import find_slice_at_time
from util.boost_pad_tracker import BoostPadTracker
from util.drive import steer_toward_target
from util.sequence import Sequence, ControlStep
from util.vec import Vec3

from collections import deque
import math


spawn_locations = [
    (-2047, -2559),
    (2047, -2559),
    (-2047, 2559),
    (2047, 2559),
    (-255, -3840),
    (255, -3840),
    (0, -4608),
    (-255, 3840),
    (255, 3840),
    (0, 4608)
]

left_orange_post = Vec3(800, 5213, 228.5)
right_orange_post = Vec3(-800, 5123, 228.5)
right_blue_post = Vec3(800, -5213, 228.5)
left_blue_post = Vec3(-800, -5123, 228.5)


class MyBot(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.active_sequence: Sequence = None
        self.boost_pad_tracker = BoostPadTracker()
        self.mechanic_queue: deque = deque()
        self.kickoff_mechanic_loaded: bool = False
        self.demoing: bool = False

    def initialize_agent(self):
        # Set up information about the boost pads now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """

        # Keep our boost pad info updated with which pads are currently active
        self.boost_pad_tracker.update_boost_status(packet)

        # This is good to keep at the beginning of get_output. It will allow you to continue
        # any sequences that you may have started during a previous call to get_output.
        if self.active_sequence and not self.active_sequence.done:
            controls = self.active_sequence.tick(packet)
            if controls is not None:
                return controls
        elif len(self.mechanic_queue) > 0:
            return self.next_in_mechanic(packet)

        # Gather some information about our car and the ball
        my_car = packet.game_cars[self.index]
        car_location = Vec3(my_car.physics.location)
        car_velocity = Vec3(my_car.physics.velocity)
        car_orientation = Orientation(my_car.physics.rotation)
        car_path = car_velocity*.1 + car_location
        ball_location = Vec3(packet.game_ball.physics.location)
        enemy_car = packet.game_cars[1-self.index]
        enemy_location = Vec3(enemy_car.physics.location)
        enemy_path = Vec3(enemy_car.physics.velocity)*.1 + \
            Vec3(enemy_car.physics.location)
        enemy_velocity = Vec3(enemy_car.physics.velocity)

        bot_to_ball = relative_location(
            car_location, car_orientation, ball_location)
        bot_to_enemy = relative_location(
            car_location, car_orientation, enemy_location)

        # Shooting info
        car_to_ball_direction = (ball_location - car_location).normalized()
        ball_to_left_post_direction = ()

        # Kickoff mechanic
        if packet.game_info.is_kickoff_pause and len(self.mechanic_queue) == 0 and not self.kickoff_mechanic_loaded:
            return self.kickoff(packet)

        if ball_location.x != 0 or ball_location.y != 0:
            self.kickoff_mechanic_loaded = False

        if bot_to_ball.length() < bot_to_enemy.length() - 1500:
            controls = SimpleControllerState()
            controls.steer = steer_toward_target(my_car, ball_location)
            controls.throttle = 1.0

            return controls

        # get boost if low
        if my_car.boost < 75 and not self.demoing:
            nearest_boost_location = self.get_nearest_boost(
                packet=packet, car_location=car_location)
            controls = SimpleControllerState()
            controls.steer = steer_toward_target(
                my_car, Vec3(nearest_boost_location))
            controls.throttle = 1.0
            return controls

        # start demoing
        if my_car.boost > 75:
            self.demoing = True

        # demo logic
        if self.demoing:
            if not my_car.is_super_sonic and my_car.boost < 5:
                self.demoing = False
            elif not my_car.is_super_sonic:
                self.renderer.draw_string_3d(
                    car_location, 1, 1, "Demoing", self.renderer.red())

                controls = SimpleControllerState()

                if bot_to_enemy.length() > 500:
                    controls.steer = steer_toward_target(
                        my_car, enemy_location+enemy_velocity*.2)
                else:
                    controls.steer = steer_toward_target(
                        my_car, enemy_location)

                controls.throttle = 1.0
                controls.boost = True

                # jump if enemy jumps
                if (enemy_location - car_location).length() < 900 and Vec3(enemy_car.physics.location).z > car_location.z:
                    print('jump')
                    controls.jump = True

                if (enemy_location - car_location).length() < 800:
                    car_path = car_velocity*.1 + car_location

                    # if relative_location(car_path, car_orientation, enemy_path).y > 50:
                    #     return self.side_dodge(packet=packet, enemy_location=enemy_location, direction=False)
                    # if relative_location(car_path, car_orientation, enemy_path).y < -50:
                    #     return self.side_dodge(packet=packet, enemy_location=enemy_location, direction=True)

                return controls
            else:
                self.renderer.draw_string_3d(
                    car_location, 1, 1, "Demoing", self.renderer.red())

                controls = SimpleControllerState()

                if bot_to_enemy.length() > 800:
                    controls.steer = steer_toward_target(
                        my_car, enemy_location+enemy_velocity*.2)
                else:
                    controls.steer = steer_toward_target(
                        my_car, enemy_location)

                controls.throttle = 1.0
                controls.boost = True

                # jump if enemy jumps
                if (enemy_location - car_location).length() < 900 and Vec3(enemy_car.physics.location).z > car_location.z:
                    print('jump')
                    controls.jump = True

                if (enemy_location - car_location).length() < 800:

                    if relative_location(car_path, car_orientation, enemy_path).y > 50:
                        return self.side_dodge(packet=packet, enemy_location=enemy_location, direction=False)
                    if relative_location(car_path, car_orientation, enemy_path).y < -50:
                        return self.side_dodge(packet=packet, enemy_location=enemy_location, direction=True)

                return controls

    def get_nearest_boost(self, packet: GameTickPacket, car_location):
        info = self.get_field_info()
        nearest_boost_loc = None

        # loop over all the boosts
        for i, boost in enumerate(info.boost_pads):
            # only want large boosts that haven't been taken
            if boost.is_full_boost and packet.game_boosts[i].is_active:
                # if we haven't found any boosts yet, use this one
                if not nearest_boost_loc:
                    nearest_boost_loc = boost.location
                else:
                    # if this boost is closer, save that
                    if car_location.dist(Vec3(boost.location)) < car_location.dist(Vec3(nearest_boost_loc)):
                        nearest_boost_loc = boost.location

        # if no large boosts are found, find the nearest small boost
        # CODE SMELL: very similar duplicate code, looping over boost list twice
        if nearest_boost_loc is None:
            for i, boost in enumerate(info.boost_pads):
                # only want large boosts that haven't been taken
                if packet.game_boosts[i].is_active:
                    # if we haven't found any boosts yet, use this one
                    if not nearest_boost_loc:
                        nearest_boost_loc = boost.location
                    else:
                        # if this boost is closer, save that
                        if car_location.dist(Vec3(boost.location)) < car_location.dist(Vec3(nearest_boost_loc)):
                            nearest_boost_loc = boost.location

        # if no boosts are available, target the center of the field
        if nearest_boost_loc is None:
            nearest_boost_loc = Vec3(0, 0, 0)

        # a different possible optimization we could make would be to look at the
        # packet.game_boosts[i].timer to find boosts that will respawn before our car arrives there

        return Vec3(nearest_boost_loc)

    def begin_front_flip(self, packet):
        # Send some quickchat just for fun
        self.send_quick_chat(
            team_only=False, quick_chat=QuickChatSelection.Information_IGotIt)

        # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
        # logic during that time because we are setting the active_sequence.
        self.active_sequence = Sequence([
            ControlStep(duration=0.05,
                        controls=SimpleControllerState(jump=True)),
            ControlStep(duration=0.05,
                        controls=SimpleControllerState(jump=False)),
            ControlStep(duration=0.2, controls=SimpleControllerState(
                jump=True, pitch=-1)),
            ControlStep(duration=0.8, controls=SimpleControllerState()),
        ])

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)

    def kickoff(self, packet):
        my_car = packet.game_cars[self.index]
        packet.game_cars
        car_location = Vec3(my_car.physics.location)

        kickoff_location = Vec3(0, 0, 0)

        self.kickoff_mechanic_loaded = True

        controls = SimpleControllerState()
        controls.steer = steer_toward_target(my_car, kickoff_location)
        controls.throttle = 1.0
        controls.boost = True

        print('Kickoff has been called')

        self.mechanic_queue = deque()

        is_diagonal = False

        for location in spawn_locations[:4]:
            if int(location[0]) == int(car_location.x) and int(location[1]) == int(car_location.y):
                is_diagonal = True

        if is_diagonal:
            print('This is a diagonal kickoff location')

            self.active_sequence = Sequence([
                ControlStep(duration=.1,
                            controls=controls),
            ])

            self.mechanic_queue.append(Sequence([
                ControlStep(duration=0.02,
                            controls=SimpleControllerState(jump=True, boost=False)),
                ControlStep(duration=0.02,
                            controls=SimpleControllerState(jump=False)),
                ControlStep(duration=0.2, controls=SimpleControllerState(
                    jump=True, pitch=-1)),
                ControlStep(
                    duration=0.8, controls=SimpleControllerState(throttle=1)),
            ]))

            self.mechanic_queue.append(Sequence([
                ControlStep(duration=.01,
                            controls=controls),
            ]))

            self.mechanic_queue.append(Sequence([
                ControlStep(duration=1,
                            controls=SimpleControllerState(boost=True, throttle=1)),
            ]))

            self.mechanic_queue.append(Sequence([
                ControlStep(duration=0.02,
                            controls=SimpleControllerState(jump=True, boost=False)),
                ControlStep(duration=0.02,
                            controls=SimpleControllerState(jump=False)),
                ControlStep(duration=0.2, controls=SimpleControllerState(
                    jump=True, pitch=-1)),
                ControlStep(
                    duration=0.8, controls=SimpleControllerState(throttle=1)),
            ]))

        else:
            self.active_sequence = Sequence([
                ControlStep(duration=.1,
                            controls=controls),
            ])
            self.mechanic_queue.append(Sequence([
                ControlStep(duration=.1,
                            controls=SimpleControllerState(boost=True, throttle=1)),
            ]))

            self.mechanic_queue.append(Sequence([
                ControlStep(duration=0.01,
                            controls=SimpleControllerState(jump=True, boost=False)),
                ControlStep(duration=0.02,
                            controls=SimpleControllerState(jump=False)),
                ControlStep(duration=0.2, controls=SimpleControllerState(
                    jump=True, pitch=-1)),
                ControlStep(
                    duration=0.8, controls=SimpleControllerState(throttle=1)),

            ]))

            self.mechanic_queue.append(Sequence([
                ControlStep(duration=.02,
                            controls=controls),
            ]))

            self.mechanic_queue.append(Sequence([
                ControlStep(duration=1.4,
                            controls=SimpleControllerState(boost=True, throttle=1)),
            ]))

            self.mechanic_queue.append(Sequence([
                ControlStep(duration=0.02,
                            controls=SimpleControllerState(jump=True, boost=False)),
                ControlStep(duration=0.02,
                            controls=SimpleControllerState(jump=False)),
                ControlStep(duration=0.2, controls=SimpleControllerState(
                    jump=True, pitch=-1)),
                ControlStep(
                    duration=0.8, controls=SimpleControllerState(throttle=1)),
            ]))

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)

    def next_in_mechanic(self, packet):
        self.active_sequence = self.mechanic_queue.popleft()
        return self.active_sequence.tick(packet)

    # returns the amount of radians to turn from source to target
    def side_dodge(self, packet: GameTickPacket, enemy_location: Vec3, direction: bool):
        my_car = packet.game_cars[self.index]
        packet.game_cars
        car_location = Vec3(my_car.physics.location)

        controls = SimpleControllerState()
        controls.steer = steer_toward_target(my_car, enemy_location)
        controls.throttle = 1.0
        controls.boost = True

        # True means left
        if direction:
            self.active_sequence = Sequence([
                ControlStep(duration=0.02,
                            controls=SimpleControllerState(jump=True, boost=True)),
                ControlStep(duration=0.02,
                            controls=SimpleControllerState(jump=False, boost=True)),
                ControlStep(duration=0.2, controls=SimpleControllerState(
                    jump=True, roll=-1, boost=True)),
                ControlStep(
                    duration=0.8, controls=SimpleControllerState(throttle=1, boost=True)),
            ])
        else:
            self.active_sequence = Sequence([
                ControlStep(duration=0.02,
                            controls=SimpleControllerState(jump=True, boost=True)),
                ControlStep(duration=0.02,
                            controls=SimpleControllerState(jump=False, boost=True)),
                ControlStep(duration=0.2, controls=SimpleControllerState(
                    jump=True, roll=1, boost=True)),
                ControlStep(
                    duration=0.8, controls=SimpleControllerState(throttle=1, boost=True)),
            ])

        return self.active_sequence.tick(packet)
