from collections.abc import Callable, Mapping

from prompt_toolkit import PromptSession
from prompt_toolkit.completion import Completer, Completion
from prompt_toolkit.document import Document
from prompt_toolkit.formatted_text import FormattedText
from prompt_toolkit.history import InMemoryHistory
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.patch_stdout import patch_stdout
from prompt_toolkit.styles import Style


class CommandCompleter(Completer):
    def __init__(self, commands: Mapping[str, str]):
        self.commands = commands

    def get_completions(self, document: Document, _complete_event):
        prefix = document.text_before_cursor.lower()
        for command, description in self.commands.items():
            if command.startswith(prefix):
                yield Completion(
                    command,
                    start_position=-len(prefix),
                    display_meta=description,
                )


def run_command_shell(
    commands: Mapping[str, str],
    dispatch: Callable[[str], bool],
    status: Callable[[], str],
    *,
    title: str,
    emergency_command: str,
):
    bindings = KeyBindings()

    @bindings.add("c-g")
    def stop(event):
        event.app.exit(result=emergency_command)

    session = PromptSession(
        completer=CommandCompleter(commands),
        complete_while_typing=True,
        history=InMemoryHistory(),
        key_bindings=bindings,
        style=Style.from_dict(
            {
                "prompt": "bold #00aaaa",
                "status": "#ffffff bg:#303030",
                "shortcut": "bold #ffff00 bg:#303030",
            }
        ),
    )

    def toolbar():
        return FormattedText(
            [
                ("class:status", f" {status()}  "),
                ("class:shortcut", "Ctrl+G STOP RECORDING  Ctrl+C EXIT  Tab COMPLETE "),
            ]
        )

    print(title, flush=True)
    with patch_stdout(raw=True):
        while True:
            try:
                line = session.prompt(
                    FormattedText([("class:prompt", "teleop> ")]),
                    bottom_toolbar=toolbar,
                    refresh_interval=0.5,
                )
            except (EOFError, KeyboardInterrupt):
                return
            command = line.strip().lower()
            if not command:
                continue
            try:
                if not dispatch(command):
                    return
            except Exception as exception:
                print(f"Error: {exception}", flush=True)
