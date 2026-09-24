import pyray as rl
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget


class RecordButton(Widget):
  """Compact on-road screen recording toggle."""
  SIZE = 86

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._font = gui_app.font(FontWeight.BOLD)

  def _render(self, rect: rl.Rectangle) -> None:
    recording = gui_app.is_recording()
    cx, cy = int(rect.x + rect.width / 2), int(rect.y + rect.height / 2)
    bg = rl.Color(110, 0, 0, 220) if recording else rl.Color(0, 0, 0, 175)
    border = rl.Color(255, 70, 70, 255) if recording else rl.Color(255, 255, 255, 190)
    rl.draw_circle(cx, cy, rect.width / 2, bg)
    rl.draw_circle_lines(cx, cy, rect.width / 2 - 2, border)
    rl.draw_circle(cx, cy - 8, 13, rl.RED if recording else rl.Color(255, 90, 90, 255))
    label = "停止" if recording else "录像"
    size = 22
    tw = rl.measure_text_ex(self._font, label, size, 0).x
    rl.draw_text_ex(self._font, label, rl.Vector2(cx - tw / 2, cy + 11), size, 0, rl.WHITE)

  def _handle_mouse_release(self, mouse_pos):
    super()._handle_mouse_release(mouse_pos)
    gui_app.toggle_recording()
    self._params.put_bool("ScreenRecord", gui_app.is_recording())
