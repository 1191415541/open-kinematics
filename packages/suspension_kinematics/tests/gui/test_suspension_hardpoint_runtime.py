import tkinter as tk

from suspension_kinematics.gui.suspension.widgets import HardpointTable
from suspension_kinematics.gui.suspension.workbench import (
    create_default_suspension_project,
)


def test_set_hardpoints_rebuilds_sheet_row_positions_for_initial_load() -> None:
    root = tk.Tk()
    root.withdraw()
    try:
        table = HardpointTable(root, lambda: None)
        table.pack(fill=tk.BOTH, expand=True)

        project = create_default_suspension_project()
        table.set_hardpoints(project.hardpoints)
        root.update_idletasks()
        root.update()

        assert len(table.sheet.get_sheet_data()) == len(project.hardpoints)
        assert len(table.sheet.MT.row_positions) == len(project.hardpoints) + 1
        assert len(table.sheet.MT.find_all()) > 1
    finally:
        root.destroy()
