import select
import sys
import termios
import tty


class KeyboardInterface:
    def __init__(self):
        self.enabled = sys.stdin.isatty()
        self.fd = sys.stdin.fileno() if self.enabled else None
        self.old_settings = None

    def __enter__(self):
        if self.fd is not None:
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.restore()

    def restore(self):
        if self.fd is not None and self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.old_settings)
            self.old_settings = None

    def get_key(self):
        if not self.enabled:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if readable:
            return sys.stdin.read(1)
        return None
