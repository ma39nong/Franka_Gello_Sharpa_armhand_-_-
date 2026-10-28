import hashlib

from linker_hand_ros2_sdk.o30i_transport import bundled_libcanbus_path


def test_bundled_x86_64_libcanbus_runtime_is_present_and_unmodified():
    library = bundled_libcanbus_path()
    assert library.name == "libcanbus.so"
    assert hashlib.sha256(library.read_bytes()).hexdigest() == (
        "6c7100a10415cbbde08d42d895ccdf4c9b680badf1f04e3322458037d320c543"
    )
