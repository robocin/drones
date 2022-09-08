class STDOUT:
    @staticmethod
    def debug(time, context: str, message: str):
        debug = "{}s: [{}] {}".format(time, context.upper(), message)
        print(debug)