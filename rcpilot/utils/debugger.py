from rcpilot.environment.environment import MessageType
from colorama import Fore, Back

class Debug:
    def __init__(self, message_type) -> None:
        self.message_type = message_type

    def __call__(self, *args, **kwargs):
        background, foreground = self.__color_select(self.message_type)

        header = background + foreground + '[' + MessageType.as_string(self.message_type) + ']' + Back.RESET + Fore.RESET
        debug = "{} {}".format(header, *args, **kwargs)
        print(debug)

    def __color_select(self, context):
        match(context):
            case "ROBOCIN_PILOT":
                return Back.LIGHTGREEN_EX, Fore.BLACK
            case "ROBOCIN_VISION":
                return Back.LIGHTBLUE_EX, Fore.BLACK
            case "ROBOCIN_DECISION":
                return Back.LIGHTCYAN_EX, Fore.BLACK
            case "ROBOCIN_NAVIGATION":
                return Back.LIGHTMAGENTA_EX, Fore.BLACK
            case "TELEMETRY":
                return Back.LIGHTWHITE_EX, Fore.BLACK
            case MessageType.WARNING:
                return Back.LIGHTYELLOW_EX, Fore.BLACK
            case MessageType.ERROR:
                return Back.LIGHTRED_EX, Fore.BLACK

