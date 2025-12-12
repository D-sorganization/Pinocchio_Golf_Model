"""Pinocchio GUI Wrapper (PyQt6 + meshcat)."""

import logging
import sys

import meshcat.geometry as g
import meshcat.visualizer as viz
import numpy as np  # noqa: TID253
import pinocchio as pin
from PyQt6 import QtCore, QtWidgets

# Set up logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class LogPanel(QtWidgets.QTextEdit):
    """Log panel widget for displaying messages."""

    def __init__(self) -> None:
        """Initialize the log panel."""
        super().__init__()
        self.setReadOnly(True)
        self.setStyleSheet(
            "background:#111; color:#0F0; font-family:Consolas; font-size:12px;"
        )


class PinocchioGUI(QtWidgets.QWidget):
    """Main GUI widget for Pinocchio robot visualization and computation."""

    def __init__(self) -> None:
        """Initialize the Pinocchio GUI."""
        super().__init__()
        self.setWindowTitle("Pinocchio UI: Robot Math With No Floor")

        # Internal state
        self.model = None
        self.data = None
        self.joint_sliders: list[QtWidgets.QSlider] = []
        self.joint_rows: list[QtWidgets.QWidget] = []  # Track row widgets for cleanup
        self.joint_names: list[str] = []

        # Meshcat viewer
        self.viewer = viz.Visualizer(zmq_url="tcp://127.0.0.1:6000")
        self.viewer.open()

        # Layout
        layout = QtWidgets.QVBoxLayout()

        # URDF Loader Button
        self.load_btn = QtWidgets.QPushButton("Load URDF")
        self.load_btn.setToolTip("Load a URDF file from the filesystem")
        self.load_btn.clicked.connect(self.load_urdf)
        layout.addWidget(self.load_btn)

        # Scroll Area for Sliders
        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.slider_container = QtWidgets.QWidget()
        self.slider_layout = QtWidgets.QVBoxLayout()
        self.slider_container.setLayout(self.slider_layout)
        self.scroll.setWidget(self.slider_container)
        layout.addWidget(self.scroll, stretch=3)

        # Robot Computation Panel
        compute_layout = QtWidgets.QGridLayout()

        self.fk_btn = QtWidgets.QPushButton("Forward Kinematics")
        self.fk_btn.setToolTip("Compute end-effector position based on current joint angles")
        self.fk_btn.clicked.connect(self.compute_fk)
        compute_layout.addWidget(self.fk_btn, 0, 0)

        self.mass_btn = QtWidgets.QPushButton("Mass Matrix")
        self.mass_btn.setToolTip("Compute the joint space inertia matrix M(q)")
        self.mass_btn.clicked.connect(self.compute_mass_matrix)
        compute_layout.addWidget(self.mass_btn, 0, 1)

        self.bias_btn = QtWidgets.QPushButton("Bias Forces")
        self.bias_btn.setToolTip("Compute Coriolis, centrifugal, and gravity forces")
        self.bias_btn.clicked.connect(self.compute_bias_forces)
        compute_layout.addWidget(self.bias_btn, 0, 2)

        compute_frame_layout = QtWidgets.QHBoxLayout()
        self.frame_box = QtWidgets.QComboBox()
        self.frame_box.setToolTip("Select a frame to compute Jacobian")
        self.jac_btn = QtWidgets.QPushButton("Compute Jacobian")
        self.jac_btn.setToolTip("Compute the geometric Jacobian for the selected frame")
        self.jac_btn.clicked.connect(self.compute_jacobian)
        compute_frame_layout.addWidget(self.frame_box)
        compute_frame_layout.addWidget(self.jac_btn)
        compute_layout.addLayout(compute_frame_layout, 1, 0, 1, 3)

        layout.addLayout(compute_layout, stretch=1)

        # Log Output
        self.log = LogPanel()
        layout.addWidget(self.log, stretch=2)

        # 3D Viewer joint pos overlay
        self.viewer["overlay/joint_axes"].delete()

        self.setLayout(layout)
        self.resize(500, 700)

    def log_write(self, text: str) -> None:
        """Write text to log panel.

        Args:
            text: Text to write
        """
        self.log.append(text)
        logger.info("%s", text)

    def load_urdf(self) -> None:
        """Load URDF file and initialize model."""
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select URDF File", "", "URDF Files (*.urdf *.xml)"
        )
        if not fname:
            return

        try:
            self.model = pin.buildModelFromUrdf(fname)
            if self.model is None:
                msg = "Failed to build model from URDF"
                raise RuntimeError(msg)  # noqa: TRY301
            self.data = self.model.createData()
            if self.data is None:
                msg = "Failed to create model data"
                raise RuntimeError(msg)  # noqa: TRY301
            # Type narrowing: at this point, self.model and self.data are guaranteed to be non-None
            self.frame_box.clear()

            # Setup frames for Jacobian query
            for _i, frame in enumerate(self.model.frames):
                self.frame_box.addItem(frame.name)

            # Create sliders
            self.create_sliders()

            # Viewer reset and load model visual geometry
            self.viewer["robot"].delete()
            self.viewer["robot"].set_object(g.URDFLoader().load(fname))
            self.log_write(f"✅ URDF loaded: {fname}")
            msg = (
                f"Model has {self.model.nq} generalized coordinates "
                f"and {len(self.model.frames)} frames."
            )
            self.log_write(msg)

        except (OSError, ValueError, RuntimeError) as e:
            self.log_write(f"❌ Failed to load URDF: {e}")

    def create_sliders(self) -> None:
        """Create sliders for joint control."""
        if self.model is None:
            return

        # Clear old rows (sliders + labels)
        for row in self.joint_rows:
            row.deleteLater()

        self.joint_rows = []
        self.joint_sliders = []
        self.joint_names = []

        # Note: We don't need to recreate self.slider_layout, just clear it
        # But since we delete widgets, they remove themselves from layout.

        for jn in self.model.names[1:]:
            self.joint_names.append(jn)

            # Container for the row
            row_widget = QtWidgets.QWidget()
            row_layout = QtWidgets.QHBoxLayout(row_widget)
            row_layout.setContentsMargins(0, 0, 0, 0)

            # Label Name
            label = QtWidgets.QLabel(jn)
            label.setFixedWidth(120)  # Give enough space for long joint names

            # Slider
            slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
            slider.setMinimum(-314)
            slider.setMaximum(314)
            slider.setValue(0)
            slider.setSingleStep(1)

            # Value Display
            val_label = QtWidgets.QLabel("0.00")
            val_label.setFixedWidth(50)
            val_label.setAlignment(
                QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter
            )

            # Connect slider
            # Use default argument to capture specific label instance
            slider.valueChanged.connect(
                lambda val, vl=val_label: self.on_slider_change(val, vl)
            )

            row_layout.addWidget(label)
            row_layout.addWidget(slider)
            row_layout.addWidget(val_label)

            self.slider_layout.addWidget(row_widget)

            self.joint_sliders.append(slider)
            self.joint_rows.append(row_widget)

    def on_slider_change(self, val: int, label: QtWidgets.QLabel) -> None:
        """Handle slider value change.

        Args:
            val: The integer value from slider (-314 to 314)
            label: The label widget to update
        """
        # Update text
        rad = val / 100.0
        label.setText(f"{rad:.2f}")

        # Update viewer
        self.update_viewer_joints()

    def update_viewer_joints(self) -> None:
        """Update viewer with current joint positions."""
        if self.model is None or self.data is None:
            return
        q = self.get_joint_state()
        self.viewer["robot"].set_joint_positions(q)
        self.viewer["overlay/joint_axes"].delete()

        # Draw local axes for each joint
        for jid, name in enumerate(self.joint_names):
            o_mf = self.data.oMf[jid + 1]
            self.viewer[f"overlay/joint_axes/{name}"].set_object(
                g.Triad(scale=0.1), o_mf
            )

    def get_joint_state(self) -> list[float]:
        """Get current joint state from sliders.

        Returns:
            List of joint positions in radians
        """
        if self.model is None:
            return []
        return [
            s.value() / 100.0 for s in self.joint_sliders
        ]  # Using radian approx mapping

    def compute_fk(self) -> None:
        """Compute forward kinematics."""
        if self.model is None or self.data is None:
            self.log_write("Please load a URDF model first.")
            return
        q = self.get_joint_state()
        pin.forwardKinematics(self.model, self.data, np.array(q))
        pin.updateFramePlacements(self.model, self.data)

        pos = self.data.oMf[-1].translation
        self.log_write(f"FK for end frame '{self.model.frames[-1].name}':")
        self.log_write(f"Position = {np.round(pos, 4)}")
        self.viewer["robot"].set_joint_positions(q)

    def compute_mass_matrix(self) -> None:
        """Compute mass matrix."""
        if self.model is None or self.data is None:
            self.log_write("Please load a URDF model first.")
            return
        q = self.get_joint_state()
        m_matrix = pin.crba(self.model, self.data, np.array(q))
        self.log_write("Mass matrix M(q):")
        self.log_write(str(np.round(m_matrix, 4)))

    def compute_bias_forces(self) -> None:
        """Compute bias forces (gravity + coriolis + centrifugal)."""
        if self.model is None or self.data is None:
            return
        q = self.get_joint_state()
        pin.computeGeneralizedGravity(self.model, self.data, np.array(q))
        pin.rnea(
            self.model,
            self.data,
            np.array(q),
            np.zeros(self.model.nv),
            np.zeros(self.model.nv),
        )
        b = self.data.nle  # nonlinear effects vector

        self.log_write("Bias terms (gravity + coriolis + centrifugal):")
        self.log_write(str(np.round(b, 4)))

    def compute_jacobian(self) -> None:
        """Compute Jacobian for selected frame."""
        if self.model is None or self.data is None:
            self.log_write("Please load a URDF model first.")
            return
        frame_name = self.frame_box.currentText()
        frame_id = self.model.getFrameId(frame_name)
        q = self.get_joint_state()

        pin.forwardKinematics(self.model, self.data, np.array(q))
        pin.updateFramePlacements(self.model, self.data)
        j_matrix = pin.computeFrameJacobian(
            self.model,
            self.data,
            np.array(q),
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

        self.log_write(f"Jacobian for frame '{frame_name}' (LOCAL_WORLD_ALIGNED):")
        self.log_write(str(np.round(j_matrix, 4)))


def main() -> None:
    """Main entry point for the GUI application."""
    app = QtWidgets.QApplication(sys.argv)
    gui = PinocchioGUI()
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
