import tkinter as tk

from suspension_kinematics.gui.steering.widgets import HardpointEditor
from suspension_kinematics.steering.workbench import default_hardpoint_rows


def test_set_rows_rebuilds_sheet_row_positions_for_initial_load() -> None:
    root = tk.Tk()
    root.withdraw()
    try:
        editor = HardpointEditor(root, lambda: None)
        editor.pack(fill=tk.BOTH, expand=True)

        editor.set_rows(default_hardpoint_rows())
        root.update_idletasks()
        root.update()

        assert len(editor.sheet.get_sheet_data()) == 6
        assert len(editor.sheet.MT.row_positions) == 7
        assert len(editor.sheet.MT.find_all()) > 1
    finally:
        root.destroy()
