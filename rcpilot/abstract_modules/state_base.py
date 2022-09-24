from abc import ABC, abstractmethod
from rcpilot.abstract_modules.singleton_meta import SingletonMeta

class StateBase(ABC, SingletonMeta):
    """
    The base State class declares methods that all Concrete State should
    implement and also provides a backreference to the Context object,
    associated with the State. This backreference can be used by States to
    transition the Context to another State.
    """

    @property
    def context(self):
        return self._context

    @context.setter
    def context(self, context) -> None:
        self._context = context

    @abstractmethod
    def execute(self) -> None:
        pass