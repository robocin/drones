"""Created by felipe-nunes on 22/09/2022
"""

class MessageType:
    """Message types for debugger
    """
    WARNING = 0
    ERROR = 1
    INFO = 2

    @staticmethod
    def as_string(message):
        match message:
            case 0:
                return "WARNING"
            case 1:
                return "ERROR"
            case 2:
                return "INFO"
            case _:
                return message
     