"""
    Class for runtime debugging at stdout.
    Main module for printing information on console. 
"""

from autopilot.constants import MessageType
from colorama import Fore, Back


class STDOUT:
    @staticmethod
    def debug(context: str, message: str):
        background, foreground = STDOUT.color_select(context)

        header = background + foreground + \
            '[' + MessageType.as_string(context) + ']' + \
            Back.RESET + Fore.RESET
        debug = "{} {}".format(header, message)
        print(debug)

    @staticmethod
    def color_select(context):
        match(context):
            case "ROBOCIN_PILOT":
                return Back.LIGHTGREEN_EX, Fore.BLACK
            case "ROBOCIN_VISION":
                return Back.LIGHTBLUE_EX, Fore.BLACK
            case "ROBOCIN_DECISION":
                return Back.LIGHTCYAN_EX, Fore.BLACK
            case "ROBOCIN_NAVIGATION":
                return Back.LIGHTMAGENTA_EX, Fore.BLACK
            case MessageType.WARNING:
                return Back.LIGHTYELLOW_EX, Fore.BLACK
            case MessageType.ERROR:
                return Back.LIGHTRED_EX, Fore.BLACK
