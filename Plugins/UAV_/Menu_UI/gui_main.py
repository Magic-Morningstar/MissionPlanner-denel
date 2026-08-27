import sys

# Assumes gui_main.py sits alongside the API/ package (same relative
# layout as main.py importing API.panel_sync) — adjust this import if
# gui_main.py ends up somewhere else relative to API/.
from API import api

from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QFrame,
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont


class ActionBox(QFrame):
    """One UP/DOWN action inside a physical button group."""

    def __init__(self, arrow, label):
        super().__init__()

        self.setObjectName("actionBox")

        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.arrow_label = QLabel(arrow)
        self.arrow_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.arrow_label.setFont(QFont("Arial", 24, QFont.Weight.Bold))

        self.text_label = QLabel(label or "UNASSIGNED")
        self.text_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.text_label.setWordWrap(True)
        self.text_label.setFont(QFont("Arial", 12, QFont.Weight.Bold))

        layout.addWidget(self.arrow_label)
        layout.addWidget(self.text_label)

    def set_active(self, active):
        """Set the visual state to match the hardware active state."""
        self.setObjectName(
            "actionBoxActive" if active else "actionBox"
        )

        self.style().unpolish(self)
        self.style().polish(self)
        self.update()


class ButtonGroup(QFrame):
    """Represents one physical hardware button with UP and DOWN."""

    def __init__(self, number, up_action, down_action):
        super().__init__()

        self.setObjectName("buttonGroup")

        layout = QVBoxLayout(self)

        title = QLabel(f"BUTTON {number}")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 12, QFont.Weight.Bold))

        self.up_box = ActionBox("▲", up_action)
        self.down_box = ActionBox("▼", down_action)

        layout.addWidget(title)
        layout.addWidget(self.up_box)
        layout.addWidget(self.down_box)

    def set_up_active(self, active):
        self.up_box.set_active(active)

    def set_down_active(self, active):
        self.down_box.set_active(active)


class MenuItem(QFrame):

    def __init__(self, short_title):
        super().__init__()

        self.setObjectName("menuItem")

        layout = QHBoxLayout(self)

        self.label = QLabel(short_title.upper())
        self.label.setFont(QFont("Arial", 14, QFont.Weight.Bold))

        arrow = QLabel("›")
        arrow.setFont(QFont("Arial", 24))

        layout.addWidget(self.label)
        layout.addStretch()
        layout.addWidget(arrow)

    def set_selected(self, selected):

        if selected:
            self.setObjectName("menuItemSelected")
        else:
            self.setObjectName("menuItem")

        self.style().unpolish(self)
        self.style().polish(self)


class MenuUI(QWidget):

    def __init__(self):
        super().__init__()

        # FIXED: was self.load_config() reading menu_config.json — now
        # reads api.menus() directly, in-process. Shape is different
        # from the old JSON schema: no "bit" key, no "bindings" key —
        # api.controls(menu_id) is the replacement for the latter (see
        # update_display/update_hardware_states below), and "selected"
        # already tells us which menu is current, so there's no need to
        # separately look up a bit index and match it (see below).
        self.menus = [
            menu
            for menu in api.menus()
            if menu["id"] != "system"
        ]

        # FIXED: old code read menu_register.get("selected menu", 0) and
        # matched it against each menu's "bit" field. api.menus() already
        # tells us which menu is selected directly — no bit arithmetic
        # needed on this side at all.
        self.current_index = 0
        for i, menu in enumerate(self.menus):
            if menu["selected"]:
                self.current_index = i
                break

        self.menu_widgets = []
        self.button_groups = []

        self.setup_window()
        self.setup_ui()
        self.update_display()

        # =================================================
        # HARDWARE STATE MONITOR
        # =================================================
        # FIXED: previously re-read menu_config.json from disk every
        # tick. Now calls api.controls()/api.menus() directly — same
        # polling cadence, no file I/O, no JSON parsing, no
        # FileNotFoundError/JSONDecodeError handling needed (api.py's
        # calls can't fail that way).
        self.hardware_timer = QTimer(self)
        self.hardware_timer.timeout.connect(
            self.update_hardware_states
        )
        self.hardware_timer.start(50)  # 20 checks per second

        # Store the last states so the status text only
        # changes when a real hardware state transition occurs.
        self.last_active_states = [False] * 6

    def setup_window(self):

        self.setWindowTitle("Ground Station HUD")

        self.resize(1500, 850)

        # Transparent overlay
        self.setAttribute(
            Qt.WidgetAttribute.WA_TranslucentBackground
        )

        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint
            | Qt.WindowType.WindowStaysOnTopHint
        )

    def setup_ui(self):

        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(30, 30, 30, 30)

        # =================================================
        # LEFT MENU
        # =================================================

        self.menu_panel = QFrame()
        self.menu_panel.setObjectName("menuPanel")

        menu_layout = QVBoxLayout(self.menu_panel)

        menu_header = QHBoxLayout()

        menu_title = QLabel("▦  MENU")
        menu_title.setFont(
            QFont("Arial", 18, QFont.Weight.Bold)
        )

        self.menu_count = QLabel()

        menu_header.addWidget(menu_title)
        menu_header.addStretch()
        menu_header.addWidget(self.menu_count)

        menu_layout.addLayout(menu_header)

        for menu in self.menus:

            item = MenuItem(menu["short_title"])

            self.menu_widgets.append(item)

            menu_layout.addWidget(item)

        menu_layout.addStretch()

        main_layout.addWidget(
            self.menu_panel,
            alignment=Qt.AlignmentFlag.AlignTop
        )

        # =================================================
        # RIGHT SIDE
        # =================================================

        right_layout = QVBoxLayout()

        right_layout.addStretch()

        # Main control panel
        self.control_panel = QFrame()
        self.control_panel.setObjectName("controlPanel")

        control_layout = QVBoxLayout(self.control_panel)

        # Header
        header_layout = QHBoxLayout()

        self.menu_title = QLabel()
        self.menu_title.setFont(
            QFont("Arial", 20, QFont.Weight.Bold)
        )

        self.short_title = QLabel()
        self.short_title.setFont(
            QFont("Arial", 14, QFont.Weight.Bold)
        )

        header_layout.addWidget(self.menu_title)

        header_layout.addStretch()

        header_layout.addWidget(self.short_title)

        control_layout.addLayout(header_layout)

        # Button groups
        self.buttons_layout = QHBoxLayout()

        for i in range(3):

            group = ButtonGroup(
                i + 1,
                "",
                ""
            )

            self.button_groups.append(group)

            self.buttons_layout.addWidget(group)

        control_layout.addLayout(
            self.buttons_layout
        )

        right_layout.addWidget(
            self.control_panel
        )

        # Bottom status bar
        self.status_panel = QFrame()
        self.status_panel.setObjectName("statusPanel")

        status_layout = QHBoxLayout(
            self.status_panel
        )

        status_layout.addWidget(
            QLabel("▲ ▼   Change Menu")
        )

        status_layout.addStretch()

        self.action_status = QLabel(
            "READY"
        )

        self.action_status.setFont(
            QFont("Arial", 10, QFont.Weight.Bold)
        )

        status_layout.addWidget(
            self.action_status
        )

        right_layout.addWidget(
            self.status_panel
        )

        main_layout.addLayout(right_layout)

        # =================================================
        # STYLE
        # =================================================

        self.setStyleSheet("""
            QFrame#menuPanel,
            QFrame#controlPanel,
            QFrame#statusPanel {
                background-color: rgba(0, 0, 0, 80);
                border: 1px solid rgba(0, 217, 255, 120);
                border-radius: 12px;
                color: #e0f7fc;
            }

            QFrame#menuItem {
                background-color: rgba(0, 0, 0, 40);
                border: 1px solid rgba(0, 217, 255, 70);
                border-radius: 8px;
                padding: 8px;
                color: #e0f7fc;
            }

            QFrame#menuItemSelected {
                background-color: rgba(0, 180, 255, 60);
                border: 2px solid #00d9ff;
                border-radius: 8px;
                padding: 8px;
                color: #ffffff;
            }

            QFrame#buttonGroup {
                background-color: rgba(0, 0, 0, 50);
                border: 1px solid rgba(0, 217, 255, 90);
                border-radius: 10px;
                padding: 8px;
                color: #e0f7fc;
            }

            QFrame#actionBox {
                background-color: rgba(5, 15, 25, 50);
                border: 1px solid rgba(0, 217, 255, 90);
                border-radius: 10px;
                padding: 8px;
                color: #e0f7fc;
            }

            QFrame#actionBoxActive {
                background-color: rgba(0, 150, 230, 80);
                border: 2px solid #00e5ff;
                border-radius: 10px;
                padding: 8px;
                color: #ffffff;
            }

            QLabel {
                color: #e0f7fc;
            }
        """)

    def get_binding_label(self, bindings, index):

        if index >= len(bindings):
            return "UNASSIGNED"

        return (
            bindings[index].get("label")
            or "UNASSIGNED"
        )

    def update_display(self):

        menu = self.menus[self.current_index]

        self.menu_title.setText(
            menu["title"].upper()
        )

        self.short_title.setText(
            menu["short_title"].upper()
        )

        self.menu_count.setText(
            f"{self.current_index + 1} / {len(self.menus)}"
        )

        # Highlight selected menu
        for i, widget in enumerate(
            self.menu_widgets
        ):
            widget.set_selected(
                i == self.current_index
            )

        # FIXED: was menu.get("bindings", []) reading straight off the
        # JSON dict — api.menus() doesn't carry bindings at all, so this
        # is now a direct call for this menu's live controls instead.
        bindings = api.controls(menu["id"])

        # Update 3 button groups
        for i, group in enumerate(
            self.button_groups
        ):

            up_index = i * 2
            down_index = i * 2 + 1

            up_label = self.get_binding_label(
                bindings,
                up_index
            )

            down_label = self.get_binding_label(
                bindings,
                down_index
            )

            group.up_box.text_label.setText(
                up_label
            )

            group.down_box.text_label.setText(
                down_label
            )

            # Immediately reflect the current live state
            # when changing menus.
            up_active = (
                up_index < len(bindings)
                and bindings[up_index].get("active", False)
            )

            down_active = (
                down_index < len(bindings)
                and bindings[down_index].get("active", False)
            )

            group.set_up_active(up_active)
            group.set_down_active(down_active)

        self.action_status.setText("READY")

    def update_hardware_states(self):
        """
        Read live active states from api.py for the menu currently
        displayed in the UI.

        TRUE  -> corresponding action box stays highlighted.
        FALSE -> corresponding action box returns to normal.

        Menu selection is NOT changed from here — same as before,
        browsing with the keyboard is local to this window and doesn't
        push a menu change back to the backend (see change_menu below).
        """

        # FIXED: was a try/except around open()+json.load() with
        # FileNotFoundError/JSONDecodeError/OSError handling — none of
        # that applies anymore, api.controls() can't fail that way.
        current_menu_id = self.menus[self.current_index]["id"]
        bindings = api.controls(current_menu_id)

        current_states = [False] * 6

        for i, group in enumerate(self.button_groups):

            up_index = i * 2
            down_index = i * 2 + 1

            up_active = (
                up_index < len(bindings)
                and bindings[up_index].get("active", False) is True
            )

            down_active = (
                down_index < len(bindings)
                and bindings[down_index].get("active", False) is True
            )

            current_states[up_index] = up_active
            current_states[down_index] = down_active

            group.set_up_active(up_active)
            group.set_down_active(down_active)

            if up_active and not self.last_active_states[up_index]:
                label = self.get_binding_label(bindings, up_index)
                self.action_status.setText(
                    f"ACTIVE: {label.upper()}"
                )

            if down_active and not self.last_active_states[down_index]:
                label = self.get_binding_label(bindings, down_index)
                self.action_status.setText(
                    f"ACTIVE: {label.upper()}"
                )

        # If nothing is active anymore, return status to READY.
        if not any(current_states):
            self.action_status.setText("READY")

        self.last_active_states = current_states

    def change_menu(self, direction):

        self.current_index = (
            self.current_index + direction
        ) % len(self.menus)

        self.update_display()

    def keyPressEvent(self, event):

        key = event.key()

        # MENU NAVIGATION ONLY
        if key == Qt.Key.Key_Up:
            self.change_menu(-1)

        elif key == Qt.Key.Key_Down:
            self.change_menu(1)

        else:
            super().keyPressEvent(event)


def create_gui():
    """
    Builds the QApplication + window but does NOT run the event loop —
    the caller runs app.exec() itself. This is what lets main.py launch
    the GUI: Qt's event loop has to run on the main thread, which is
    exactly the thread main.py's Controller.start() needs to block on
    anyway once its background threads (SerialHandler, Mavlink_controller,
    PanelStateSync) are already running independently.
    """
    app = QApplication(sys.argv)
    window = MenuUI()
    window.show()
    return app, window


if __name__ == "__main__":
    app, window = create_gui()
    sys.exit(app.exec())