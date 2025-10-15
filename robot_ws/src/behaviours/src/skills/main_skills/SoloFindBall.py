import math
import time
import random
from enum import Enum

from src.skills.Skill import Skill
from src.skills.leaf_skills.Stand import Stand
from src.skills.leaf_skills.Walk import Walk
from src.skills.main_skills.WalkInCircle import WalkInCircle
from src.utils.Timer import Timer


class State(Enum):
    SPIN_1 = 1
    STAND_1 = 2
    SPIN_2 = 3
    STAND_2 = 4
    SPIN_3 = 5
    STAND_3 = 6


class SoloFindBall(Skill):
    """
    Description:
    Skill to find the ball. It will rotate 120 degrees, pause, then do a head scan.
    After doing three cycles of this (a full 360 degree rotation), it will walk to
    a new position before repeating the scans.
    """

    SPIN_TURN_RATE = 1.5
    WALK_TURN_RATE = 0.2
    HEAD_TURN_RATE = 0.1
    MAX_HEAD_YAW = math.radians(70)
    HEAD_SCAN_PITCH = 0.4

    def _initialise_sub_skills(self):
        self._sub_skills = {
            "Spin": Walk(self),
            "Stand": Stand(self)
        }

    def _reset(self):
        self._current_sub_skill = "Stand"
        self._turn_direction = random.choice([-1, 1])
        self._initial_heading = self.ctx.blackboard.robot_world.heading
        self._phase = State.SPIN_1
        self._input = None
        self._head_hit_left = False

    def _transition(self):
        if self._phase == State.SPIN_1:
            self._current_sub_skill = "Spin"
            self._centre_head()
            if abs(self.ctx.blackboard.robot_world.heading - self._initial_heading) > math.radians(120):
                self._phase = State.STAND_1

        elif self._phase == State.STAND_1:
            self._current_sub_skill = "Stand"
            if self._head_scan():
                self._phase = State.SPIN_2
                self._initial_heading = self.ctx.blackboard.robot_world.heading

        elif self._phase == State.SPIN_2:
            self._current_sub_skill = "Spin"
            self._centre_head()
            if abs(self.ctx.blackboard.robot_world.heading - self._initial_heading) > math.radians(120):
                self._phase = State.STAND_2

        elif self._phase == State.STAND_2:
            self._current_sub_skill = "Stand"
            if self._head_scan():
                self._phase = State.SPIN_3
                self._initial_heading = self.ctx.blackboard.robot_world.heading

        elif self._phase == State.SPIN_3:
            self._current_sub_skill = "Spin"
            self._centre_head()
            if abs(self.ctx.blackboard.robot_world.heading - self._initial_heading) > math.radians(120):
                self._phase = State.STAND_3

        elif self._phase == State.STAND_3:
            self._current_sub_skill = "Stand"
            if self._head_scan():
                self._phase = State.SPIN_1
                self._initial_heading = self.ctx.blackboard.robot_world.heading

    def _tick(self):
        if self._current_sub_skill == "Spin":
            self._tick_sub_skill(turn=self.SPIN_TURN_RATE * self._turn_direction)
        else:
            self._tick_sub_skill()

    def _centre_head(self):
        self.ctx.blackboard.motion_command.head_command.is_relative = False
        self.ctx.blackboard.motion_command.head_command.yaw_speed = 0.25
        self.ctx.blackboard.motion_command.head_command.yaw = 0.0

    def _head_scan(self):
        self.ctx.blackboard.motion_command.head_command.is_relative = False
        yaw = self.ctx.blackboard.motion_command.head_command.yaw
        if yaw > 0 and self._head_hit_left:
            self._head_hit_left = False
            return True
        if yaw > self.MAX_HEAD_YAW:
            self.ctx.blackboard.head_yaw_increment = -self.HEAD_TURN_RATE
        elif yaw < -self.MAX_HEAD_YAW:
            self.ctx.blackboard.head_yaw_increment = self.HEAD_TURN_RATE
            self._head_hit_left = True
        self.ctx.blackboard.motion_command.head_command.pitch = self.HEAD_SCAN_PITCH
        self.ctx.blackboard.motion_command.head_command.yaw += self.ctx.blackboard.head_yaw_increment
        self.ctx.blackboard.motion_command.head_command.yaw_speed = 0.1
        self.ctx.blackboard.motion_command.head_command.pitch_speed = 0.05
        return False
