import socket
from config import GCS_NOTIFICATION_PORT


def send_buzzer_alert(message: str) -> bool:
    """
    Send a buzzer alert to the GCS and block until the operator dismisses it.

    Returns True if the GCS acknowledged (operator clicked OK).
    Returns False if the GCS is unreachable (e.g. not running) — caller should
    still stop the buzzer after a timeout in that case.

    Usage:
        if send_buzzer_alert("Low Battery!"):
            stop_buzzer()
    """
    try:
        with socket.create_connection(('127.0.0.1', GCS_NOTIFICATION_PORT), timeout=5) as s:
            s.sendall(f"BUZZER_ALERT:{message}\n".encode())
            ack = s.makefile().readline().strip()
            return ack == "BUZZER_ACK"
    except Exception as e:
        print(f"[Notifications] GCS unreachable: {e}")
        return False
