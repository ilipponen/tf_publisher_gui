import sys
from datetime import datetime
from threading import Thread

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QHBoxLayout, QLabel, QLineEdit, QDoubleSpinBox, QGridLayout, QMenuBar,
    QFileDialog, QMessageBox)
from PyQt5.QtCore import Qt

import yaml
import numpy as np
from scipy.spatial import transform as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from PyQt5.QtWidgets import QComboBox

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher_gui')
        self.publisher = TransformBroadcaster(self)
        self.transform = TransformStamped()

        # Parameters for the transform
        self.declare_parameter('parent_frame', 'parent_link')
        self.declare_parameter('child_frame', 'child_link')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('hz', 10)


        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.init_x = self.get_parameter('x').value
        self.init_y = self.get_parameter('y').value
        self.init_z = self.get_parameter('z').value
        self.init_roll = self.get_parameter('roll').value
        self.init_pitch = self.get_parameter('pitch').value
        self.init_yaw = self.get_parameter('yaw').value

        self.hz = self.get_parameter('hz').value


        self.transform.header.frame_id = self.parent_frame
        self.transform.child_frame_id = self.child_frame
        
        self.update_transform(
            self.init_x, self.init_y, self.init_z,
            self.init_roll, self.init_pitch, self.init_yaw
        )


        self.timer = self.create_timer(1/self.hz, self.publish_transform)

    def update_transform(self, x, y, z, roll, pitch, yaw):
        self.transform.transform.translation.x = x
        self.transform.transform.translation.y = y
        self.transform.transform.translation.z = z

        # Convert the roll, pitch, yaw to quaternion
        rot = R.Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        quat = rot.as_quat()

        self.transform.transform.rotation.x = quat[0]
        self.transform.transform.rotation.y = quat[1]
        self.transform.transform.rotation.z = quat[2]
        self.transform.transform.rotation.w = quat[3]

    def publish_transform(self):
        # self.get_logger().info(f"Publishing transform: {self.transform}")
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.publisher.sendTransform(self.transform)

    def get_transform_dict(self) -> dict[str, float]:

        roll, pitch, yaw = R.Rotation.from_quat([
            self.transform.transform.rotation.x,
            self.transform.transform.rotation.y,
            self.transform.transform.rotation.z,
            self.transform.transform.rotation.w
        ]).as_euler('xyz', degrees=False)

        return {
            'x': self.transform.transform.translation.x,
            'y': self.transform.transform.translation.y,
            'z': self.transform.transform.translation.z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }
    
    def update_frame(self, parent_frame, child_frame):
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.transform.header.frame_id = self.parent_frame
        self.transform.child_frame_id = self.child_frame

    def transform_string(self) -> str:
        x = self.transform.transform.translation.x
        y = self.transform.transform.translation.y
        z = self.transform.transform.translation.z
        roll, pitch, yaw = R.Rotation.from_quat([
            self.transform.transform.rotation.x,
            self.transform.transform.rotation.y,
            self.transform.transform.rotation.z,
            self.transform.transform.rotation.w
        ]).as_euler('xyz', degrees=False)
        frame_str = f"Parent Frame: {self.parent_frame}, Child Frame: {self.child_frame}"
        transltion_str = f"x: {x}, y: {y}, z: {z}"
        rotation_str = f"roll: {roll}, pitch: {pitch}, yaw: {yaw}"

        return f"{frame_str} {transltion_str} {rotation_str}"


class TFPublisherApp(QWidget):
    def __init__(self, node : TFPublisher) -> None:
        super().__init__()
        self.node : TFPublisher = node
        self.parent_frame = node.parent_frame
        self.child_frame = node.child_frame
        self.initUI()

    def initUI(self) -> None:

        layout = QVBoxLayout()
        self.setLayout(layout)
        # add a menu bar
        menu_bar = QMenuBar()
        menu = menu_bar.addMenu("File")
        save_action = menu.addAction("Save")
        save_action.triggered.connect(self.save_to_yaml)
        print_action = menu.addAction("Print")
        print_action.triggered.connect(self.print_transform)
        # add a separator
        menu.addSeparator()
        about_action = menu.addAction("About")
        about_action.triggered.connect(self.show_about_dialog)
        exit_action = menu.addAction("Exit")
        exit_action.triggered.connect(self.close)
        layout.setMenuBar(menu_bar)

        self.frame_display = QLabel()
        frame_name_str = f"{self.parent_frame} → {self.child_frame}"
        self.frame_display.setText(frame_name_str)
        font = self.frame_display.font()
        font.setBold(True)
        font.setPointSize(14)
        self.frame_display.setFont(font)
        self.frame_display.setAlignment(Qt.AlignCenter)

        layout.addWidget(self.frame_display)
        self.frames: dict[str, QComboBox] = {}

        self.frame_layout = QGridLayout()
        for i, frame in enumerate(['parent', 'child']):
            label = QLabel(f'{frame.capitalize()} Frame:')
            combo_box = QComboBox()
            combo_box.setEditable(True)
            if frame == 'parent':
                combo_box.setCurrentText(self.node.parent_frame)
            else:
                combo_box.setCurrentText(self.node.child_frame)
            combo_box.lineEdit().textChanged.connect(self.frame_update(frame))
            self.frame_layout.addWidget(label, i, 0)
            self.frame_layout.addWidget(combo_box, i, 1)
            self.frames[frame] = combo_box
        layout.addLayout(self.frame_layout)

        self.update_tf_tree()

        set_frames_button = QPushButton("Set Frames")
        set_frames_button.clicked.connect(self.update_frames)
        layout.addWidget(set_frames_button)


        init_values = self.node.get_transform_dict()

        # X, Y, Z inputs
        self.translation_inputs = {}
        for axis in ['x', 'y', 'z']:
            h_layout = QHBoxLayout()
            label = QLabel(f'Translation {axis}:')
            input_field = QDoubleSpinBox()
            input_field.setRange(-20.0, 20.0)
            input_field.setSingleStep(0.01)
            input_field.setDecimals(3)
            input_field.setValue(init_values[axis])
            input_field.valueChanged.connect(self.update_transform)
            h_layout.addWidget(label)
            h_layout.addWidget(input_field)
            layout.addLayout(h_layout)
            self.translation_inputs[axis] = input_field

        # Roll, Pitch, Yaw inputs
        self.rotation_inputs : dict[str, QDoubleSpinBox] = {}
        for axis in ['roll', 'pitch', 'yaw']:
            h_layout = QHBoxLayout()
            label = QLabel(f'{axis.capitalize()}:')
            input_field = QDoubleSpinBox()
            input_field.setRange(-3.14159 * 2, 3.14159 * 2)
            input_field.setSingleStep(0.0001)
            input_field.setDecimals(6)
            input_field.setValue(init_values[axis])
            input_field.valueChanged.connect(self.update_transform)
            h_layout.addWidget(label)
            h_layout.addWidget(input_field)
            layout.addLayout(h_layout)
            self.rotation_inputs[axis] = input_field


        self.rad_deg_switch = QPushButton("Switch to Degrees")
        self.rad_deg_switch.setCheckable(True)
        self.rad_deg_switch.toggled.connect(self.toggle_rad_deg)
        layout.addWidget(self.rad_deg_switch)
        # add separator to layout
        line = QLabel()
        line.setFrameShape(QLabel.HLine)
        line.setFrameShadow(QLabel.Sunken)
        layout.addWidget(line)
        # add save and print buttons
        save_button = QPushButton("Save")
        save_button.clicked.connect(self.save_to_yaml)
        layout.addWidget(save_button)

        print_button = QPushButton("Print")
        print_button.clicked.connect(self.print_transform)
        layout.addWidget(print_button)

        self.setLayout(layout)
        self.setWindowTitle('TF Publisher GUI')
        self.show()

    def update_tf_tree(self):
        # Fetch the current tf tree and update the combo boxes
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, self.node)

        def update_frames():
            frames = tf_buffer.all_frames_as_string()
            frame_list = frames.split('\n')
            frame_list = [frame.split(' ')[0] for frame in frame_list if frame]

            for combo_box in self.frames.values():
                current_text = combo_box.currentText()
                combo_box.clear()
                combo_box.addItems(frame_list)
                combo_box.setCurrentText(current_text)

        QTimer.singleShot(1000, update_frames)


    def toggle_rad_deg(self):
        if self.rad_deg_switch.isChecked():
            self.rad_deg_switch.setText("Switch to Radians")
            roll = np.rad2deg(self.rotation_inputs['roll'].value())
            pitch = np.rad2deg(self.rotation_inputs['pitch'].value())
            yaw = np.rad2deg(self.rotation_inputs['yaw'].value())
            # Modify the Spinbox settings to be in degrees
            for sb in self.rotation_inputs.values():
                sb.setRange(-360, 360)
                sb.setSingleStep(0.1)
                sb.setDecimals(3)
        else:
            self.rad_deg_switch.setText("Switch to Degrees")
            roll = np.deg2rad(self.rotation_inputs['roll'].value())
            pitch = np.deg2rad(self.rotation_inputs['pitch'].value())
            yaw = np.deg2rad(self.rotation_inputs['yaw'].value())
            # Modify the Spinbox settings to be in radians
            for sb in self.rotation_inputs.values():
                sb.setRange(-3.14159 * 2, 3.14159 * 2)
                sb.setSingleStep(0.0001)
                sb.setDecimals(6)

        self.rotation_inputs['roll'].setValue(roll)
        self.rotation_inputs['pitch'].setValue(pitch)
        self.rotation_inputs['yaw'].setValue(yaw)

    def show_about_dialog(self):
        about_widget = QWidget()
        about_layout = QVBoxLayout()
        about_widget.setLayout(about_layout)

        about_text = QLabel("""
        This is a simple tool for publishing a transform between two frames using a GUI.

        Author:
        Ilkka Lipponen, 2025
        """)
        about_text.setWordWrap(True)
        about_text.setAlignment(Qt.AlignCenter)
        about_layout.addWidget(about_text)

        close_button = QPushButton("Close")
        close_button.clicked.connect(about_widget.close)
        about_layout.addWidget(close_button)

        about_widget.setWindowTitle("About")
        about_widget.setGeometry(100, 100, 200, 300)
        about_widget.show()
        self.about_widget = about_widget  # Keep a reference to prevent garbage collection



    def save_to_yaml(self):
        # Prompt the user for a filename
        suggested_filename = f"{self.parent_frame}-{self.child_frame}-{datetime.now().strftime('%Y-%m-%d-%H-%M-%S')}.yaml"
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save transformation", suggested_filename, "YAML Files (*.yaml)")
        if not filename:
            return
        transforms_as_dict = self.node.get_transform_dict()
        transforms_as_dict['parent_frame'] = self.parent_frame
        transforms_as_dict['child_frame'] = self.child_frame

        tf_dict = {}
        for key, value in transforms_as_dict.items():
            if key in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
                fvalue = round(float(value), 12)
                tf_dict[key] = fvalue
            else:
                tf_dict[key] = value

        # put dict in the order we want
        tf_dict = dict([
            ('date', datetime.now().strftime('%Y-%m-%d')),
            ('time', datetime.now().strftime('%H:%M:%S')),
            ('parent_frame', tf_dict['parent_frame']),
            ('child_frame', tf_dict['child_frame']),
            ('x', tf_dict['x']),
            ('y', tf_dict['y']),
            ('z', tf_dict['z']),
            ('roll', tf_dict['roll']),
            ('pitch', tf_dict['pitch']),
            ('yaw', tf_dict['yaw']),
        ])
        
        with open(filename, 'w') as f:
            yaml.dump(tf_dict, f, default_flow_style=False, sort_keys=False)

    def print_transform(self):
        transform_str = self.node.transform_string()
        self.node.get_logger().info(transform_str)

    def update_frames(self):
        self.parent_frame = self.frames['parent'].currentText()
        self.child_frame = self.frames['child'].currentText()
        self.node.update_frame(self.parent_frame, self.child_frame)

    def update_transform(self) -> None:
        x = self.translation_inputs['x'].value()
        y = self.translation_inputs['y'].value()
        z = self.translation_inputs['z'].value()


        roll = self.rotation_inputs['roll'].value()
        pitch = self.rotation_inputs['pitch'].value()
        yaw = self.rotation_inputs['yaw'].value()

        # Convert to radians if the switch is toggled
        if self.rad_deg_switch.isChecked():
            roll = np.deg2rad(roll)
            pitch = np.deg2rad(pitch)
            yaw = np.deg2rad(yaw)

        self.node.update_transform(x, y, z, roll, pitch, yaw)

    def frame_update(self, frame_name) -> callable:
        def handler():

            if (frame_name == 'parent' and 
                self.frames[frame_name].currentText() == self.node.parent_frame):
                self.frames[frame_name].setStyleSheet(
                    "background-color: #8FFF98;")
        
                return
            elif (frame_name == 'child' and 
                  self.frames[frame_name].currentText() == self.node.child_frame):
                self.frames[frame_name].setStyleSheet(
                    "background-color: #8FFF98;")
        
                return
    
            self.frames[frame_name].setStyleSheet("background-color: #fc994e;")
        return handler

    def update_frames(self):
        self.parent_frame = self.frames['parent'].currentText()
        self.child_frame = self.frames['child'].currentText()
        self.node.update_frame(self.parent_frame, self.child_frame)

        self.frame_display.setText(f"{self.parent_frame} → {self.child_frame}")

        for frame in self.frames.values():
            frame.setStyleSheet("background-color: #8FFF98;")

def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()

    def ros_spin():
        rclpy.spin(node)

    ros_thread = Thread(target=ros_spin)
    ros_thread.start()

    app = QApplication(sys.argv)
    ex : TFPublisherApp = TFPublisherApp(node)
    
    # Update the node's transform with the latest values from the GUI
    def update_node_transform():
        if rclpy.ok():
            ex.update_transform()
        else:
            timer.stop()
            app.quit()

    timer = QTimer()
    timer.timeout.connect(update_node_transform)
    timer.start(100)
    
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
