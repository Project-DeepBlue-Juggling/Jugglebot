# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

'''
This module has been updated from the base YASMIN library to:
- Include access to the ROS2 node
- Include a default error handling method
'''

from typing import List
from abc import ABC, abstractmethod
from yasmin.blackboard import Blackboard
from threading import Event

# Imports to allow the State to access the ROS node
from rclpy.node import Node
from yasmin_ros.yasmin_node import YasminNode 

class State(ABC):

    _node: Node

    def __init__(self, outcomes: List[str], node: Node = None) -> None:
        self._outcomes = []
        self._canceled = False
        self._error_event = Event()

        # Initialize self._node
        if node is None:
            self._node = YasminNode.get_instance()
        else:
            self._node = node

        if outcomes:
            # Ensure 'error' outcome is always present
            self._outcomes = list(set(outcomes + ['error']))
        else:
            raise Exception("There must be at least one outcome")

    def __call__(self, blackboard: Blackboard = None) -> str:
        self._canceled = False
        self._error_event.clear() # Reset the error event at the beginning of the state

        if blackboard is None:
            blackboard = Blackboard()

        outcome = self.execute(blackboard)

        if outcome not in self._outcomes:
            raise Exception(
                f"Outcome '{outcome}' does not belong to the outcomes of the state '{self}'. The possible outcomes are: {self._outcomes}")

        return outcome

    @abstractmethod
    def execute(self, blackboard: Blackboard) -> str:
        raise NotImplementedError(
            "Subclasses must implement the execute method")

    def __str__(self) -> str:
        return self.__class__.__name__

    def cancel_state(self) -> None:
        self._canceled = True
        self._error_event.set() # Set the error event when the state is canceled

    def is_canceled(self) -> bool:
        return self._canceled

    def get_outcomes(self) -> List[str]:
        return self._outcomes

    def on_error(self, blackboard: Blackboard) -> str:
        """
        Default error handling method. Subclasses can override this method to implement custom error handling.
        """
        return 'error' # Return generic error outcome