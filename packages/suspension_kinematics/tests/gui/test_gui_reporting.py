from pathlib import Path
from zipfile import ZipFile

import pytest

from suspension_kinematics.gui.reporting import (
    ReportCurveSelection,
    ReportExportOptions,
    export_gui_report_docx,
)
from suspension_kinematics.gui.suspension.workbench import (
    SuspensionCurve,
    SuspensionSweepSettings,
    load_suspension_project,
)
from suspension_kinematics.steering.workbench import (
    SteeringCurve,
    default_steering_project,
)


def test_export_gui_report_docx_writes_chapters_toc_and_summary_table(
    double_wishbone_geometry_file: Path,
    tmp_path: Path,
) -> None:
    pytest.importorskip("docx")
    suspension_project = load_suspension_project(double_wishbone_geometry_file)
    suspension_project.settings = SuspensionSweepSettings(
        start=-10.0,
        stop=20.0,
        steps=4,
    )
    suspension_project.curves = [
        SuspensionCurve(
            x_output="wheel_travel_mm",
            y_output="camber_deg",
            label="Camber vs Travel",
        )
    ]
    steering_project = default_steering_project()
    steering_project.curves = [
        SteeringCurve(
            x_output="input_value",
            y_output="left_wheel_angle_deg",
            label="Left Wheel vs Input",
        )
    ]
    report_path = tmp_path / "gui-report.docx"

    export_gui_report_docx(
        report_path,
        options=ReportExportOptions(
            scope="combined",
            include_images=(
                "suspension_preview",
                "steering_preview",
            ),
        ),
        suspension_project=suspension_project,
        steering_project=steering_project,
        suspension_source_path=double_wishbone_geometry_file,
    )

    assert report_path.exists()
    assert report_path.stat().st_size > 0
    with ZipFile(report_path) as archive:
        document_xml = archive.read("word/document.xml").decode("utf-8")
    assert "Kinematics GUI Report" in document_xml
    assert "Table Of Contents" in document_xml
    assert "Suspension" in document_xml
    assert "Steering" in document_xml
    assert "Kinematic Parameter Summary" in document_xml
    assert "Camber vs Travel" in document_xml
    assert "Left Wheel vs Input" in document_xml


def test_export_gui_report_docx_only_includes_selected_curves(
    double_wishbone_geometry_file: Path,
    tmp_path: Path,
) -> None:
    docx = pytest.importorskip("docx")
    suspension_project = load_suspension_project(double_wishbone_geometry_file)
    suspension_project.settings = SuspensionSweepSettings(
        start=-10.0,
        stop=20.0,
        steps=4,
    )
    suspension_project.curves = [
        SuspensionCurve(
            x_output="wheel_travel_mm",
            y_output="camber_deg",
            label="Camber vs Travel",
        ),
        SuspensionCurve(
            x_output="wheel_travel_mm",
            y_output="toe_deg",
            label="Toe vs Travel",
        ),
    ]
    steering_project = default_steering_project()
    steering_project.curves = [
        SteeringCurve(
            x_output="input_value",
            y_output="left_wheel_angle_deg",
            label="Left Wheel vs Input",
        ),
        SteeringCurve(
            x_output="input_value",
            y_output="right_wheel_angle_deg",
            label="Right Wheel vs Input",
        ),
    ]
    report_path = tmp_path / "gui-selected-curves-report.docx"

    export_gui_report_docx(
        report_path,
        options=ReportExportOptions(
            scope="combined",
            include_images=(),
            suspension_curves=(
                ReportCurveSelection(
                    x_output="wheel_travel_mm",
                    y_output="toe_deg",
                    label="Suspension Toe Export",
                ),
                ReportCurveSelection(
                    x_output="wheel_travel_mm",
                    y_output="camber_deg",
                    label="Suspension Camber Export",
                ),
            ),
            steering_curves=(
                ReportCurveSelection(
                    x_output="input_value",
                    y_output="left_wheel_angle_deg",
                    label="Steering Left Export",
                ),
            ),
        ),
        suspension_project=suspension_project,
        steering_project=steering_project,
        suspension_source_path=double_wishbone_geometry_file,
    )

    document = docx.Document(report_path)
    paragraphs = [paragraph.text for paragraph in document.paragraphs if paragraph.text]
    joined_text = "\n".join(paragraphs)
    assert len(document.inline_shapes) == 3
    assert "Curve Results" in joined_text
    assert "Suspension Toe Export" in joined_text
    assert "Suspension Camber Export" in joined_text
    assert "Steering Left Export" in joined_text
    assert "Right Wheel vs Input" not in joined_text
    assert joined_text.index("Suspension Toe Export") < joined_text.index(
        "Suspension Camber Export"
    )
