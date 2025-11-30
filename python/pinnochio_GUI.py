# r1
# Pinocchio GUI Wrapper (PyQt6 + meshcat)
# Total lines: 138

import sys
from PyQt6 import QtWidgets, QtCore
import pinocchio as pin
import numpy as np
import meshcat.geometry as g
import meshcat.visualizer as viz


class LogPanel(QtWidgets.QTextEdit):
    def __init__(self):
        super().__init__()
        self.setReadOnly(True)
        self.setStyleSheet("background:#111; color:#0F0; font-family:Consolas; font-size:12px;")


class PinocchioGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pinocchio UI: Robot Math With No Floor")

        # Internal state
        self.model = None
        self.data = None
        self.joint_sliders = []
        self.joint_names = []

        # Meshcat viewer
        self.viewer = viz.Visualizer(zmq_url="tcp://127.0.0.1:6000")
        self.viewer.open()

        # Layout
        layout = QtWidgets.QVBoxLayout()

        # URDF Loader Button
        self.load_btn = QtWidgets.QPushButton("Load URDF")
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
        self.fk_btn.clicked.connect(self.compute_fk)
        compute_layout.addWidget(self.fk_btn, 0, 0)

        self.mass_btn = QtWidgets.QPushButton("Mass Matrix")
        self.mass_btn.clicked.connect(self.compute_mass_matrix)
        compute_layout.addWidget(self.mass_btn, 0, 1)

        self.bias_btn = QtWidgets.QPushButton("Bias Forces")
        self.bias_btn.clicked.connect(self.compute_bias_forces)
        compute_layout.addWidget(self.bias_btn, 0, 2)

        compute_frame_layout = QtWidgets.QHBoxLayout()
        self.frame_box = QtWidgets.QComboBox()
        self.jac_btn = QtWidgets.QPushButton("Compute Jacobian")
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

    def log_write(self, text):
        self.log.append(text)
        print(text)

    def load_urdf(self):
        fname, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select URDF File", "", "URDF Files (*.urdf *.xml)"
        )
        if not fname:
            return

        try:
            self.model = pin.buildModelFromUrdf(fname)
            self.data = self.model.createData()
            self.frame_box.clear()

            # Setup frames for Jacobian query
            for i, frame in enumerate(self.model.frames):
                self.frame_box.addItem(frame.name)

            # Create sliders
            self.create_sliders()

            # Viewer reset and load model visual geometry
            self.viewer["robot"].delete()
            self.viewer["robot"].set_object(g.URDFLoader().load(fname))
            self.log_write(f"✅ URDF loaded: {fname}")
            self.log_write(f"Model has {self.model.nq} generalized coordinates and {len(self.model.frames)} frames.")

        except Exception as e:
            self.log_write(f"❌ Failed to load URDF: {e}")

    def create_sliders(self):
        # Clear old sliders
        for s in self.joint_sliders:
            s.deleteLater()

        self.joint_sliders = []
        self.joint_names = []
        self.slider_layout.setParent(None)
        self.slider_layout = QtWidgets.QVBoxLayout()
        self.slider_container.setLayout(self.slider_layout)

        for jn in self.model.names[1:]:
            self.joint_names.append(jn)
            slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
            slider.setMinimum(-314)
            slider.setMaximum(314)
            slider.setValue(0)
            slider.setSingleStep(1)

            label = QtWidgets.QLabel(jn)
            slider.valueChanged.connect(self.update_viewer_joints)

            self.slider_layout.addWidget(label)
            self.slider_layout.addWidget(slider)
            self.joint_sliders.append(slider)

        self.joint_sliders = self.joint_sliders

    def update_viewer_joints(self):
        if not self.model:
            return
        q = self.get_joint_state()
        self.viewer["robot"].set_joint_positions(q)
        self.viewer["overlay/joint_axes"].delete()

        # Draw local axes for each joint
        for jid, name in enumerate(self.joint_names):
            frame = self.model.frames[jid+1]
            oMf = self.data.oMf[jid+1]
            self.viewer[f"overlay/joint_axes/{name}"].set_object(
                g.Triad(scale=0.1), oMf
            )

    def update_viewer_joints(self):
        self.update_viewer_joints()

    def get_joint_state(self):
        if not self.model:
            return []
        q = []
        for s in self.joint_sliders:
            q.append(s.value() / 100.0)  # Using radian approx mapping
        return q

    def compute_fk(self):
        if not self.model:
            self.log_write("Load a URDF first, Einstein.")
            return
        q = self.get_joint_state()
        pin.forwardKinematics(self.model, self.data, np.array(q))
        pin.updateFramePlacements(self.model, self.data)

        pos = self.data.oMf[-1].translation
        self.log_write(f"FK for end frame '{self.model.frames[-1].name}':")
        self.log_write(f"Position = {np.round(pos, 4)}")
        self.viewer["robot"].set_joint_positions(q)

    def compute_mass_matrix(self):
        if not self.model:
            self.log_write("URDF first, floor later.")
            return
        q = self.get_joint_state()
        M = pin.crba(self.model, self.data, np.array(q))
        self.log_write("Mass matrix M(q):")
        self.log_write(str(np.round(M, 4)))

    def compute_bias_forces(self):
        if not self.model:
            return
        q = self.get_joint_state()
        pin.computeGeneralizedGravity(self.model, self.data, np.array(q))
        pin.rnea(self.model, self.data, np.array(q), np.zeros(self.model.nv), np.zeros(self.model.nv))
        b = self.data.nle  # nonlinear effects vector

        self.log_write("Bias terms (gravity + coriolis + centrifugal):")
        self.log_write(str(np.round(b, 4)))

    def compute_jacobian(self):
        if not self.model:
            self.log_write("No model, no Jacobian, only sadness.")
            return
        frame_name = self.frame_box.currentText()
        frame_id = self.model.getFrameId(frame_name)
        q = self.get_joint_state()

        pin.forwardKinematics(self.model, self.data, np.array(q))
        pin.updateFramePlacements(self.model, self.data)
        J = pin.computeFrameJacobian(self.model, self.data, np.array(q), frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        self.log_write(f"Jacobian for frame '{frame_name}' (LOCAL_WORLD_ALIGNED):")
        self.log_write(str(np.round(J, 4)))

    def compute_mass_matrix(self):
        if not self.state:
            self.log.append("Load a URDF first.")
            return
        M = pin.crba(self.model, self.data, self.state.q)
        self.log.append("Mass matrix M(q):")
        self.log.append(str(np.round(M, 4)))

    def get_joint_state(self):
        if not self.model:
            return None
        q = np.zeros(self.model.nq)
        idx = 0
        for i in range(len(self.joint_sliders)):
            q[idx] = self.joint_sliders[i].value() / 100.0
            idx += 1
        return q


def main():
    app = QtWidgets.QApplication(sys.argv)
    gui = PinocchioGUI()
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
