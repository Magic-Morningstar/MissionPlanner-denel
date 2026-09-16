import sys
import math

from API import api

from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QFrame,
    QGraphicsDropShadowEffect,
    QGraphicsOpacityEffect,
)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont, QColor


EXPANDED_TIMEOUT_MS = 4000
BUTTON_FADE_TIMEOUT_MS = 1800
BUTTON_IDLE_OPACITY = 0.18
EDGE_MARGIN_PX = 4


class ActionBox(QFrame):
    """One UP/DOWN action inside a physical button group."""

    def __init__(self, arrow, label):
        super().__init__()
        self.setObjectName("actionBox")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(7, 5, 7, 5)
        layout.setSpacing(2)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.arrow_label = QLabel(arrow)
        self.arrow_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.arrow_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))

        self.text_label = QLabel(label or "UNASSIGNED")
        self.text_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.text_label.setWordWrap(True)
        self.text_label.setFont(QFont("Arial", 9, QFont.Weight.Bold))

        layout.addWidget(self.arrow_label)
        layout.addWidget(self.text_label)

    def set_active(self, active):
        self.setObjectName("actionBoxActive" if active else "actionBox")
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()


class ButtonGroup(QFrame):
    """One physical rocker/button pair shown in the bottom-right HUD."""

    def __init__(self, number, up_action="", down_action=""):
        super().__init__()
        self.setObjectName("buttonGroup")
        self.setFixedWidth(138)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(4)

        title = QLabel(f"BUTTON {number}")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 8, QFont.Weight.Bold))

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
    """One menu row with hover-preview support and active-elsewhere dot."""

    hovered = Signal(str)
    hover_left = Signal()

    def __init__(self, menu_id, short_title):
        super().__init__()

        self.menu_id = menu_id
        self.setObjectName("menuItem")
        self.setFixedHeight(32)
        self.setMouseTracking(True)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(7, 2, 7, 2)
        layout.setSpacing(5)

        self.active_dot = QLabel("●")
        self.active_dot.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.active_dot.setObjectName("menuActiveDot")
        self.active_dot.setFixedWidth(12)
        self.active_dot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.active_dot.hide()

        glow = QGraphicsDropShadowEffect(self.active_dot)
        glow.setBlurRadius(12)
        glow.setOffset(0, 0)
        glow.setColor(QColor("#ff9d2e"))
        self.active_dot.setGraphicsEffect(glow)

        self.label = QLabel(short_title.upper())
        self.label.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.label.setFont(QFont("Arial", 9, QFont.Weight.Bold))

        arrow = QLabel("›")
        arrow.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        arrow.setFont(QFont("Arial", 15, QFont.Weight.Bold))

        layout.addWidget(self.active_dot)
        layout.addWidget(self.label)
        layout.addStretch()
        layout.addWidget(arrow)

    def enterEvent(self, event):
        self.hovered.emit(self.menu_id)
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.hover_left.emit()
        super().leaveEvent(event)

    def set_selected(self, selected):
        self.setObjectName("menuItemSelected" if selected else "menuItem")
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()

    def set_background_active(self, active):
        self.active_dot.setVisible(active)


class HoverFrame(QFrame):
    """Frame that tells the HUD when the mouse enters/leaves it."""

    entered = Signal()
    left = Signal()

    def enterEvent(self, event):
        self.entered.emit()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.left.emit()
        super().leaveEvent(event)


class MenuUI(QWidget):
    def __init__(self):
        super().__init__()

        # IMPORTANT:
        # Nothing about the menu names/count is hard coded here.
        # Whatever API.config exposes through api.menus() is what the GUI shows.
        # Exact backend menu list. No GUI-side menu names are added here.
        self.menus = list(api.menus())

        self.current_index = self._index_for_menu(api.current_menu())
        if self.current_index is None:
            self.current_index = 0

        self.preview_menu_id = None
        self.menu_widgets = []
        self.button_groups = []
        self.last_backend_menu_id = api.current_menu()
        self.last_active_by_menu = {}
        self.expanded = False

        self.setup_window()
        self.setup_ui()
        self.refresh_from_api(rebuild_menus=True)
        self.set_expanded(False)

        self.collapse_timer = QTimer(self)
        self.collapse_timer.setSingleShot(True)
        self.collapse_timer.timeout.connect(lambda: self.set_expanded(False))

        self.button_fade_timer = QTimer(self)
        self.button_fade_timer.setSingleShot(True)
        self.button_fade_timer.timeout.connect(self.fade_controls)

        self.hardware_timer = QTimer(self)
        self.hardware_timer.timeout.connect(self.update_hardware_states)
        self.hardware_timer.start(50)

    # ------------------------------------------------------------------
    # Window / geometry
    # ------------------------------------------------------------------

    def setup_window(self):
        self.setWindowTitle("Ground Station HUD")
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)

        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint
            | Qt.WindowType.WindowStaysOnTopHint
        )

    def showEvent(self, event):
        """
        Use the actual monitor geometry instead of a fixed 1200x680 window.
        This puts the menu at the real top-left edge and controls at the
        real bottom-right edge on 1080p, 1440p, portable monitors, etc.
        """
        screen = self.screen() or QApplication.primaryScreen()
        if screen is not None:
            self.setGeometry(screen.availableGeometry())
        super().showEvent(event)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def setup_ui(self):
        self.root = QGridLayout(self)
        self.root.setContentsMargins(
            EDGE_MARGIN_PX,
            EDGE_MARGIN_PX,
            EDGE_MARGIN_PX,
            EDGE_MARGIN_PX,
        )
        self.root.setHorizontalSpacing(6)
        self.root.setVerticalSpacing(6)

        # Small resting tab: true top-left corner.
        self.compact_tab = HoverFrame()
        self.compact_tab.setObjectName("compactTab")
        self.compact_tab.setFixedSize(170, 38)

        compact_layout = QHBoxLayout(self.compact_tab)
        compact_layout.setContentsMargins(9, 3, 9, 3)

        self.compact_dot = QLabel("●")
        self.compact_dot.setObjectName("compactDot")
        self.compact_dot.setFixedWidth(12)

        self.compact_title = QLabel("MENU")
        self.compact_title.setFont(QFont("Arial", 9, QFont.Weight.Bold))

        self.compact_menu = QLabel()
        self.compact_menu.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        self.compact_menu.setAlignment(Qt.AlignmentFlag.AlignRight)

        compact_layout.addWidget(self.compact_dot)
        compact_layout.addWidget(self.compact_title)
        compact_layout.addStretch()
        compact_layout.addWidget(self.compact_menu)

        # Hovering the small resting tab opens the menu.
        self.compact_tab.entered.connect(self.open_from_hover)

        self.root.addWidget(
            self.compact_tab,
            0,
            0,
            alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft,
        )

        # Expanded menu: true top-left.
        self.menu_panel = QFrame()
        self.menu_panel.setObjectName("menuPanel")
        self.menu_panel.setFixedWidth(190)

        self.menu_layout = QVBoxLayout(self.menu_panel)
        self.menu_layout.setContentsMargins(7, 7, 7, 7)
        self.menu_layout.setSpacing(3)

        menu_header = QHBoxLayout()

        menu_title = QLabel("▦ MENU")
        menu_title.setFont(QFont("Arial", 10, QFont.Weight.Bold))

        self.menu_count = QLabel()
        self.menu_count.setFont(QFont("Arial", 8))

        menu_header.addWidget(menu_title)
        menu_header.addStretch()
        menu_header.addWidget(self.menu_count)
        self.menu_layout.addLayout(menu_header)

        self.root.addWidget(
            self.menu_panel,
            0,
            0,
            alignment=Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft,
        )

        # Expanded action controls: true bottom-right.
        self.control_panel = QFrame()
        self.control_panel.setObjectName("controlPanel")

        # Keep the button HUD visible at all times. It sits very faint when
        # idle and returns to full brightness when a real control changes.
        self.control_opacity = QGraphicsOpacityEffect(self.control_panel)
        self.control_panel.setGraphicsEffect(self.control_opacity)
        self.control_opacity.setOpacity(BUTTON_IDLE_OPACITY)

        self.control_layout = QVBoxLayout(self.control_panel)
        self.control_layout.setContentsMargins(7, 6, 7, 7)
        self.control_layout.setSpacing(4)

        header_layout = QHBoxLayout()

        self.menu_title = QLabel()
        self.menu_title.setFont(QFont("Arial", 10, QFont.Weight.Bold))

        self.short_title = QLabel()
        self.short_title.setFont(QFont("Arial", 8, QFont.Weight.Bold))

        header_layout.addWidget(self.menu_title)
        header_layout.addStretch()
        header_layout.addWidget(self.short_title)
        self.control_layout.addLayout(header_layout)

        self.buttons_layout = QHBoxLayout()
        self.buttons_layout.setSpacing(5)
        self.control_layout.addLayout(self.buttons_layout)

        self.action_status = QLabel("READY")
        self.action_status.setObjectName("actionStatus")
        self.action_status.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.action_status.setFont(QFont("Arial", 8, QFont.Weight.Bold))
        self.control_layout.addWidget(self.action_status)

        self.root.addWidget(
            self.control_panel,
            1,
            1,
            alignment=Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignRight,
        )

        # The empty centre absorbs all remaining video area.
        self.root.setColumnStretch(1, 1)
        self.root.setRowStretch(1, 1)

        self.setStyleSheet("""
            QFrame#compactTab,
            QFrame#menuPanel,
            QFrame#controlPanel {
                background-color: rgba(0, 0, 0, 140);
                border: 1px solid rgba(0, 229, 255, 195);
                border-radius: 9px;
                color: #e9fbff;
            }

            QFrame#compactTab {
                background-color: rgba(0, 0, 0, 165);
                border: 1px solid #00e5ff;
            }

            QLabel#compactDot {
                color: #00e5ff;
                background: transparent;
                font-size: 10px;
            }

            QFrame#menuItem {
                background-color: rgba(0, 0, 0, 60);
                border: 1px solid rgba(0, 217, 255, 75);
                border-radius: 6px;
                color: #dff9ff;
            }

            QFrame#menuItem:hover {
                background-color: rgba(0, 130, 195, 80);
                border: 1px solid rgba(0, 229, 255, 190);
            }

            QFrame#menuItemSelected {
                background-color: rgba(0, 170, 255, 85);
                border: 1px solid #00e5ff;
                border-radius: 6px;
                color: #ffffff;
            }

            QLabel#menuActiveDot {
                color: #ff9d2e;
                background: transparent;
                font-size: 12px;
                font-weight: bold;
            }

            QFrame#buttonGroup {
                background-color: rgba(0, 0, 0, 70);
                border: 1px solid rgba(0, 217, 255, 105);
                border-radius: 7px;
                color: #e0f7fc;
            }

            QFrame#actionBox {
                background-color: rgba(5, 15, 25, 70);
                border: 1px solid rgba(0, 217, 255, 105);
                border-radius: 6px;
                color: #e0f7fc;
            }

            QFrame#actionBoxActive {
                background-color: rgba(0, 170, 255, 125);
                border: 2px solid #00e5ff;
                border-radius: 6px;
                color: #ffffff;
            }

            QLabel#actionStatus {
                color: #00e5ff;
            }

            QLabel {
                color: #e0f7fc;
                background: transparent;
            }
        """)

    # ------------------------------------------------------------------
    # Dynamic menu/control rebuilding
    # ------------------------------------------------------------------

    def _clear_layout_widgets(self, layout):
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            child_layout = item.layout()

            if widget is not None:
                widget.deleteLater()
            elif child_layout is not None:
                self._clear_layout_widgets(child_layout)

    def rebuild_menu_rows(self):
        # Preserve the header at index 0; remove all menu row widgets after it.
        while self.menu_layout.count() > 1:
            item = self.menu_layout.takeAt(1)
            if item.widget():
                item.widget().deleteLater()

        self.menu_widgets = []

        for menu in self.menus:
            item = MenuItem(menu["id"], menu["short_title"])
            item.hovered.connect(self.preview_menu)
            item.hover_left.connect(self.end_preview)
            self.menu_widgets.append(item)
            self.menu_layout.addWidget(item)

    def rebuild_button_groups(self, bindings):
        self._clear_layout_widgets(self.buttons_layout)
        self.button_groups = []

        # Pair controls as UP/DOWN. If the config contains 4, 6, 8... controls,
        # the GUI automatically makes 2, 3, 4... rocker groups.
        pair_count = max(1, math.ceil(len(bindings) / 2))

        for i in range(pair_count):
            up_index = i * 2
            down_index = up_index + 1

            up_label = self.get_binding_label(bindings, up_index)
            down_label = self.get_binding_label(bindings, down_index)

            group = ButtonGroup(i + 1, up_label, down_label)
            self.button_groups.append(group)
            self.buttons_layout.addWidget(group)

    def refresh_from_api(self, rebuild_menus=False):
        latest_menus = list(api.menus())

        # If someone adds/removes/renames/reorders a menu in config.py while
        # this code is restarted, the GUI simply reflects api.menus().
        # No GUI menu list needs editing.
        old_ids = [m["id"] for m in self.menus]
        new_ids = [m["id"] for m in latest_menus]

        self.menus = latest_menus

        if rebuild_menus or old_ids != new_ids:
            self.rebuild_menu_rows()

        backend_menu_id = api.current_menu()
        idx = self._index_for_menu(backend_menu_id)
        if idx is not None:
            self.current_index = idx

        self.update_display()
        self.update_menu_activity_indicators()

    # ------------------------------------------------------------------
    # Display state
    # ------------------------------------------------------------------

    def _index_for_menu(self, menu_id):
        for i, menu in enumerate(self.menus):
            if menu["id"] == menu_id:
                return i
        return None

    def get_binding_label(self, bindings, index):
        if index >= len(bindings):
            return "UNASSIGNED"
        return bindings[index].get("label") or "UNASSIGNED"

    def displayed_menu_id(self):
        return self.preview_menu_id or self.menus[self.current_index]["id"]

    def update_display(self):
        if not self.menus:
            return

        display_id = self.displayed_menu_id()
        display_index = self._index_for_menu(display_id)

        if display_index is None:
            display_index = self.current_index
            display_id = self.menus[display_index]["id"]

        menu = self.menus[display_index]
        bindings = api.controls(display_id)

        # Compact tab always reflects the ACTUAL selected menu, not hover preview.
        actual_menu = self.menus[self.current_index]
        self.compact_menu.setText(actual_menu["short_title"].upper())

        self.menu_title.setText(menu["title"].upper())
        self.short_title.setText(menu["short_title"].upper())
        self.menu_count.setText(
            f"{self.current_index + 1}/{len(self.menus)}"
        )

        for i, widget in enumerate(self.menu_widgets):
            widget.set_selected(i == self.current_index)

        self.rebuild_button_groups(bindings)

        for i, group in enumerate(self.button_groups):
            up_index = i * 2
            down_index = up_index + 1

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

    def update_menu_activity_indicators(self):
        any_background_active = False

        for i, (menu, widget) in enumerate(zip(self.menus, self.menu_widgets)):
            bindings = api.controls(menu["id"])

            has_active_feature = any(
                control.get("assigned", True)
                and control.get("active", False)
                for control in bindings
            )

            background_active = (
                has_active_feature
                and i != self.current_index
            )

            widget.set_background_active(background_active)
            any_background_active = (
                any_background_active or background_active
            )

        self.compact_dot.setStyleSheet(
            "color: #ff9d2e;"
            if any_background_active
            else "color: #00e5ff;"
        )

    # ------------------------------------------------------------------
    # Hover preview
    # ------------------------------------------------------------------

    def open_from_hover(self):
        """Hovering the collapsed tab exposes the menu without changing selection."""
        self.set_expanded(True)
        self.activate_controls()
        if hasattr(self, "collapse_timer"):
            self.collapse_timer.stop()

    def preview_menu(self, menu_id):
        """
        Hovering a menu previews its controls only.
        It does NOT send api.select() and therefore does NOT change the
        actual hardware/backend menu.
        """
        self.preview_menu_id = menu_id
        self.set_expanded(True)
        self.activate_controls()
        if hasattr(self, "collapse_timer"):
            self.collapse_timer.stop()
        self.update_display()

    def end_preview(self):
        self.preview_menu_id = None
        self.update_display()
        if hasattr(self, "collapse_timer"):
            self.collapse_timer.start(EXPANDED_TIMEOUT_MS)

    # ------------------------------------------------------------------
    # Compact / expanded behaviour
    # ------------------------------------------------------------------

    def set_expanded(self, expanded):
        self.expanded = expanded

        self.compact_tab.setVisible(not expanded)
        self.menu_panel.setVisible(expanded)

        # Only the menu collapses. The button HUD remains in the bottom-right.
        self.control_panel.setVisible(True)

        if expanded:
            self.raise_()

    def wake_ui(self):
        if not self.expanded:
            self.set_expanded(True)

        # Any real UI interaction/menu activity should make the button HUD
        # return to its normal full/dark appearance.
        self.activate_controls()

        if hasattr(self, "collapse_timer"):
            self.collapse_timer.start(EXPANDED_TIMEOUT_MS)

    def activate_controls(self):
        """Make the bottom-right button HUD fully visible for a short time."""
        self.control_opacity.setOpacity(1.0)
        self.button_fade_timer.start(BUTTON_FADE_TIMEOUT_MS)

    def fade_controls(self):
        """Return the bottom-right button HUD to the faint idle state."""
        self.control_opacity.setOpacity(BUTTON_IDLE_OPACITY)

    # ------------------------------------------------------------------
    # Backend / hardware sync
    # ------------------------------------------------------------------

    def snapshot_active_states(self, menu_id):
        return tuple(
            bool(c.get("active", False))
            for c in api.controls(menu_id)
        )

    def update_hardware_states(self):
        # Refresh menu definitions too, so GUI structure is always sourced
        # from api.menus(), never duplicated here.
        latest = list(api.menus())
        latest_ids = [m["id"] for m in latest]
        current_ids = [m["id"] for m in self.menus]

        if latest_ids != current_ids:
            self.menus = latest
            self.rebuild_menu_rows()

        backend_menu_id = api.current_menu()

        if backend_menu_id != self.last_backend_menu_id:
            idx = self._index_for_menu(backend_menu_id)

            if idx is not None:
                self.current_index = idx

            self.last_backend_menu_id = backend_menu_id
            self.preview_menu_id = None
            self.update_display()
            self.action_status.setText("MENU CHANGED")
            self.wake_ui()

        # Detect state transitions across ALL menus, not just the displayed one.
        # This keeps orange dots correct when something is left ON elsewhere.
        for menu in self.menus:
            menu_id = menu["id"]
            states = self.snapshot_active_states(menu_id)
            previous = self.last_active_by_menu.get(menu_id)

            if previous is not None and states != previous:
                self.wake_ui()
                self.activate_controls()

                # Only put action text in the status area if this is the
                # currently displayed/selected menu.
                if menu_id == self.displayed_menu_id():
                    controls = api.controls(menu_id)
                    for i, (old, new) in enumerate(zip(previous, states)):
                        if old != new and i < len(controls):
                            label = controls[i].get("label") or "UNASSIGNED"
                            self.action_status.setText(
                                f"{label.upper()}: {'ON' if new else 'OFF'}"
                            )
                            break

            self.last_active_by_menu[menu_id] = states

        self.update_display()
        self.update_menu_activity_indicators()

    # ------------------------------------------------------------------
    # Temporary keyboard navigation
    # ------------------------------------------------------------------

    def change_menu(self, direction):
        if not self.menus:
            return

        new_index = (
            self.current_index + direction
        ) % len(self.menus)

        new_menu_id = self.menus[new_index]["id"]

        api.select(new_menu_id)

        self.current_index = new_index
        self.last_backend_menu_id = new_menu_id
        self.preview_menu_id = None

        self.update_display()
        self.action_status.setText("MENU CHANGED")
        self.wake_ui()

    def keyPressEvent(self, event):
        key = event.key()

        # Temporary simulator for the dedicated physical menu control.
        if key == Qt.Key.Key_Up:
            self.change_menu(-1)

        elif key == Qt.Key.Key_Down:
            self.change_menu(1)

        else:
            super().keyPressEvent(event)


def create_gui():
    app = QApplication.instance() or QApplication(sys.argv)
    window = MenuUI()
    window.show()
    return app, window


if __name__ == "__main__":
    app, window = create_gui()
    sys.exit(app.exec())





# BUTTON_IDLE_OPACITY = 0.18       # faint/idle
# # activate_controls:
# self.control_opacity.setOpacity(1.0)  # dark/active
# BUTTON_FADE_TIMEOUT_MS = 1800    # how long before going faint
# find these value to change the opacity and fade timeout for the button HUD in the MenuUI class.