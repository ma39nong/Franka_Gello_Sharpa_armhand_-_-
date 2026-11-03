"""Refuse camera recovery whenever collector ownership is uncertain."""
import errno
import os
import socket
import sys


def main():
    try:
        port = int(os.environ.get("COLLECTION_CONTROL_PORT", "5592"))
        with socket.create_connection(("127.0.0.1", port), timeout=1):
            pass
    except OSError as error:
        if error.errno == errno.ECONNREFUSED:
            return 0
        print("相机自动恢复已阻止：无法确认采集控制端口状态：" + str(error), file=sys.stderr)
        return 1
    except ValueError:
        print("相机自动恢复已阻止：采集控制端口配置无效。", file=sys.stderr)
        return 1
    print("相机自动恢复已阻止：采集控制端口仍被占用，请先退出采集终端。", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
