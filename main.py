import sys
import time
import os
import math
import pickle

import numpy as np
import numpy as nmp
import cv2

from PySide6 import QtCore, QtGui
from PySide6.QtGui import QPainter
from PySide6.QtWidgets import (
    QApplication, QLabel, QPushButton, QWidget, QHBoxLayout, QVBoxLayout,
    QProgressBar, QGridLayout, QLineEdit, QPlainTextEdit
)

import xarm

# ----------------------------------------------------------------------
# Paths and storage
# ----------------------------------------------------------------------

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROGRAM_DIR = os.path.join(BASE_DIR, "programs")
IMAGES_DIR = os.path.join(BASE_DIR, "images")

os.makedirs(PROGRAM_DIR, exist_ok=True)
os.makedirs(IMAGES_DIR, exist_ok=True)

current_program_name = "unnamed"  # used for .pr and images subfolder
capture_events = []  # list of {"step": int, "cam_index": int, "shot_index": int}

# ----------------------------------------------------------------------
# Qt app / robot / globals
# ----------------------------------------------------------------------

app = QApplication(sys.argv)
bar = QProgressBar()

arm = xarm.Controller("USB")

globalx: float = 100.0
globaly: float = 100.0
globalz: float = 100.0
globalclaw: float = 0.0
globalduck: float = 0.0
globalwrist: float = 0.0
globalrotate: float = -90.0

# index 0..6, we really use 1..6 for servos
globaltime = [0.0] * 7

# sequence: list of [motid, angle]
commandsequence = [[0, 0.0]]

# During capture we may want to temporarily ignore user moves
movement_lock_until: float = 0.0

# ----------------------------------------------------------------------
# Kinematics helpers
# ----------------------------------------------------------------------


def rotate(coords, angle_deg, origin):
    ncoords = np.array([coords[0] - origin[0], coords[1] - origin[1]])
    angle = angle_deg / 57.29
    rotmat = np.array(
        [
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)],
        ]
    )
    newcoords = rotmat.dot(ncoords)
    newcoords[0] += 300
    newcoords[1] += 300
    return newcoords


def determineAnglesFormXYZ(nx, ny, nz):
    lenght = math.sqrt(nx * nx + ny * ny + nz * nz)
    nominallenght = 300
    corrangle = [0.0, 0.0, 0.0, 0.0]
    maincorangle = 0
    angle = [0.0, 0.0, 0.0, 0.0]
    if lenght != 0:
        angle[1] = 57.29 * math.acos(nz / lenght)
    if nx > 0:
        angle[0] = math.atan(ny / nx) * 57.29
    if nx < 0 and ny >= 0:
        angle[0] = (math.atan(ny / nx) + math.pi) * 57.29
    if nx < 0 and ny < 0:
        angle[0] = (math.atan(ny / nx) - math.pi) * 57.29
    if nx == 0 and ny > 0:
        angle[0] = (math.pi / 2) * 57.29
    if nx == 0 and ny < 0:
        angle[0] = -(math.pi / 2) * 57.29
    if nx == 0 and ny < 0:
        angle[0] = 0

    if lenght < nominallenght:
        deltalenght = nominallenght - lenght
        lenghtshort = deltalenght / 195
        lenghtshort = max(-1.0, min(1.0, lenghtshort))
        maincorangle = 2 * 57.29 * math.asin(lenghtshort)
        if maincorangle > 124:
            maincorangle = 124
        corrangle = [0.0, maincorangle / 2, -maincorangle, -maincorangle / 2]

    angle = nmp.subtract(angle, corrangle)
    return angle


def determineXYZfromAngles(angle):
    mindiff = 10000
    nx = ny = nz = 0
    for fx in range(-300, 300, 10):
        for fy in range(-300, 300, 10):
            for fz in range(0, 300, 10):
                fangle = determineAnglesFormXYZ(fx, fy, fz)
                diff = np.sum(np.abs(np.subtract(angle, fangle)))
                if diff < mindiff:
                    mindiff = diff
                    nx, ny, nz = fx, fy, fz
    return [nx, ny, nz]


fangle = [0.0, 0.0, 0.0, 0.0]
rescoord = determineXYZfromAngles(fangle)
globalx, globaly, globalz = rescoord[0], rescoord[1], rescoord[2]

# ----------------------------------------------------------------------
# Image / camera helpers
# ----------------------------------------------------------------------

MAX_CAMERAS = 4
cameras = []  # list of dicts: {"index", "cap", "label", "last_frame"}
camera_timer: QtCore.QTimer | None = None


def get_sequence_image_dir():
    seq_dir = os.path.join(IMAGES_DIR, current_program_name)
    os.makedirs(seq_dir, exist_ok=True)
    return seq_dir


def capture_image_from_camera(cam_index, shot_index):
    """Save a fresh frame from camera cam_index as camX_shotXXXX.png."""
    for cam in cameras:
        if cam["index"] == cam_index:
            cap = cam["cap"]
            ret, frame_bgr = cap.read()
            if not ret:
                print(f"Cannot read from camera {cam_index} at capture time")
                return
            # update preview cache
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            cam["last_frame"] = frame_rgb

            seq_dir = get_sequence_image_dir()
            filename = os.path.join(seq_dir, f"cam{cam_index}_shot{shot_index:04d}.png")
            cv2.imwrite(filename, frame_bgr)
            print(f"Saved image {filename}")
            return
    print(f"Camera {cam_index} not found for capture")


def record_capture_event(cam_index):
    """User pressed 'Capture cam N' while recording."""
    global capture_events, movement_lock_until

    if len(commandsequence) <= 1:
        step_index = 0
    else:
        step_index = len(commandsequence) - 1

    existing_for_cam = [e for e in capture_events if e["cam_index"] == cam_index]
    shot_index = len(existing_for_cam)

    # Capture immediately at this moment
    capture_image_from_camera(cam_index, shot_index)

    capture_events.append(
        {"step": step_index, "cam_index": cam_index, "shot_index": shot_index}
    )

    # Don't allow recorded moves for 1 second after capture
    movement_lock_until = max(movement_lock_until, time.time() + 1.0)


def update_camera_frames():
    """Grab frames from all cameras and update preview labels."""
    for cam in cameras:
        ret, frame_bgr = cam["cap"].read()
        if not ret:
            continue
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        cam["last_frame"] = frame_rgb
        h, w, ch = frame_rgb.shape
        bytes_per_line = ch * w
        qimg = QtGui.QImage(
            frame_rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888
        )
        pix = QtGui.QPixmap.fromImage(qimg)
        pix = pix.scaled(400, 300, QtCore.Qt.KeepAspectRatio)
        cam["label"].setPixmap(pix)


def set_max_resolution(cap):
    """
    Try to set the camera to a high resolution.
    We go through a list of common resolutions from highest to lower
    and keep the first one that the camera actually accepts.
    """
    preferred_resolutions = [
        (1920, 1080),  # Full HD
        (1280, 720),   # HD
    ]

    for w, h in preferred_resolutions:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if actual_w == w and actual_h == h:
            print(f"Camera set to {w}x{h}")
            return

    # If none of the preferred resolutions worked, just report what we have
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera using default resolution {actual_w}x{actual_h}")



def init_cameras(parent_layout: QVBoxLayout):
    global cameras, camera_timer
    for idx in range(MAX_CAMERAS):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue

        set_max_resolution(cap)
        lbl = QLabel()
        lbl.setFixedSize(400, 300)
        lbl.setText(f"Camera {idx} preview")
        btn = QPushButton(f"Capture cam {idx}")
        btn.clicked.connect(lambda _, cam_index=idx: record_capture_event(cam_index))

        cameras.append(
            {"index": idx, "cap": cap, "label": lbl, "last_frame": None}
        )

        parent_layout.addWidget(lbl)
        parent_layout.addWidget(btn)

    if cameras:
        camera_timer = QtCore.QTimer()
        camera_timer.timeout.connect(update_camera_frames)
        camera_timer.start(100)


# ----------------------------------------------------------------------
# Robot movement helpers
# ----------------------------------------------------------------------


def armSetPosition(motid, angle, speed, wait, record=True):
    """
    Send command to the arm.
    If record=True, append to commandsequence (used for manual control).
    If record=False, just move for playback without modifying sequence.
    """
    global globaltime, globalduck, commandsequence, globalclaw, globalwrist, movement_lock_until

    now = time.time()
    if record and now < movement_lock_until:
        # During the lock period we ignore user moves to keep stillness after capture
        return

    if motid == 3:
        angle += globalduck

    if math.fabs(globaltime[motid] - now) > 0.5 or not record:
        arm.setPosition(motid, angle, speed, wait)
        print("setPosition", motid, angle, speed, wait)
        if record:
            commandsequence.append([motid, angle])
        globaltime[motid] = now


def movenormal(record=True):
    global globalclaw, globalwrist
    angles = determineAnglesFormXYZ(globalx, globaly, globalz)
    for u in [0, 1]:
        armSetPosition(6 - u, angles[u], 1500, False, record=record)
    for u in [2, 3]:
        armSetPosition(6 - u, -angles[u], 1500, False, record=record)
    armSetPosition(1, globalclaw, 1500, False, record=record)
    armSetPosition(2, globalwrist, 1500, False, record=record)


# ----------------------------------------------------------------------
# Non-blocking program player
# ----------------------------------------------------------------------


class ProgramPlayer(QtCore.QObject):
    def __init__(self):
        super().__init__()
        self.running = False
        self.speed = 1.0
        self.step_index = 0
        self.timex = [0.0] * 7

    def start(self, speed=1.0):
        if len(commandsequence) <= 1:
            print("Nothing to play")
            return
        self.running = True
        self.speed = speed
        self.step_index = 0
        self.timex = [0.0] * 7
        bar.setValue(0)
        self._schedule_next_step()

    def stop(self):
        self.running = False
        print("Playback finished")

    def _schedule_next_step(self):
        if not self.running:
            return
        if self.step_index >= len(commandsequence):
            self.stop()
            return
        mot, _ = commandsequence[self.step_index]
        now = time.time()
        last = self.timex[mot]
        if abs(now - last) > 0.5:
            delay = 0.01 / self.speed
        else:
            delay = 0.5 / self.speed
        self.timex[mot] = now
        msec = max(1, int(delay * 1000))
        QtCore.QTimer.singleShot(msec, self._run_current_step)

    def _run_current_step(self):
        if not self.running:
            return
        if self.step_index >= len(commandsequence):
            self.stop()
            return

        step_index = self.step_index
        mot, angle = commandsequence[step_index]

        bar.setValue(round(100 * (step_index + 1) / len(commandsequence)))

        # playback move (no recording)
        armSetPosition(mot, angle, round(500 / self.speed), False, record=False)

        # check if we need to capture images at this step
        events_here = [e for e in capture_events if e["step"] == step_index]
        self.step_index += 1

        if events_here:
            # 1 second with no movement before capture
            QtCore.QTimer.singleShot(
                1000, lambda eh=events_here: self._do_captures_and_continue(eh)
            )
        else:
            self._schedule_next_step()

    def _do_captures_and_continue(self, events_here):
        if not self.running:
            return
        for e in events_here:
            capture_image_from_camera(e["cam_index"], e["shot_index"])
        # 1 more second of stillness after capture
        QtCore.QTimer.singleShot(1000, self._schedule_next_step)


player = ProgramPlayer()


def start_programrun(speed=1.0):
    global globalx, globaly, globalz, globalclaw, globalduck, globalwrist
    globalx = 100
    globaly = 100
    globalz = 100
    globalclaw = 0.0
    globalduck = 0.0
    globalwrist = 0.0
    player.start(speed=speed)


def programrun(progid, speed=1):
    # kept for compatibility with old keyboard shortcut 'L'
    start_programrun(speed)


# ----------------------------------------------------------------------
# Keyboard / mouse handling
# ----------------------------------------------------------------------


def keyProcess(event):
    global globalx, globaly, globalz, globalclaw, globalwrist, globalrotate, commandsequence
    if event.key() == QtCore.Qt.Key_W:
        if globalz < 299:
            globalz += 1
    if event.key() == QtCore.Qt.Key_S:
        if globalz > -50:
            globalz -= 1
    if event.key() == QtCore.Qt.Key_A:
        if globalwrist < 124:
            globalwrist += 1
    if event.key() == QtCore.Qt.Key_D:
        if globalwrist > -124:
            globalwrist -= 1
    if event.key() == QtCore.Qt.Key_P:
        globalrotate += 2.5
    if event.key() == QtCore.Qt.Key_O:
        globalrotate -= 2.5
    if event.key() == QtCore.Qt.Key_L:
        programrun(0)
    # legacy save/load (listfile1..3)
    if event.key() == QtCore.Qt.Key_1 and event.modifiers() == QtCore.Qt.ControlModifier:
        with open("listfile1", "wb") as fp:
            pickle.dump(commandsequence, fp)
            print("программа 1 сохранена (legacy)")
    if event.key() == QtCore.Qt.Key_1 and event.modifiers() != QtCore.Qt.ControlModifier:
        with open("listfile1", "rb") as fp:
            commandsequence = pickle.load(fp)
            print("программа 1 загружена (legacy)")
    if event.key() == QtCore.Qt.Key_2 and event.modifiers() == QtCore.Qt.ControlModifier:
        with open("listfile2", "wb") as fp:
            pickle.dump(commandsequence, fp)
            print("программа 2 сохранена (legacy)")
    if event.key() == QtCore.Qt.Key_2 and event.modifiers() != QtCore.Qt.ControlModifier:
        with open("listfile2", "rb") as fp:
            commandsequence = pickle.load(fp)
            print("программа 2 загружена (legacy)")
    if event.key() == QtCore.Qt.Key_3 and event.modifiers() == QtCore.Qt.ControlModifier:
        with open("listfile3", "wb") as fp:
            pickle.dump(commandsequence, fp)
            print("программа 3 сохранена (legacy)")
    if event.key() == QtCore.Qt.Key_3 and event.modifiers() != QtCore.Qt.ControlModifier:
        with open("listfile3", "rb") as fp:
            commandsequence = pickle.load(fp)
            print("программа 3 загружена (legacy)")
    if event.key() == QtCore.Qt.Key_Escape:
        globalx, globaly, globalz, globalclaw, globalwrist = [0.0, 0.0, 290.0, 0.0, 0.0]
    if event.key() == QtCore.Qt.Key_K:
        commandsequence = [[0, 0.0]]

    movenormal(record=True)
    label.repaint()


class SquareLabel(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)

    def mouseDoubleClickEvent(self, event):
        global globalduck
        if event.buttons() == QtCore.Qt.LeftButton:
            globalduck += 5.0
        if event.buttons() == QtCore.Qt.RightButton:
            globalduck -= 5.0
        movenormal(record=True)
        label.repaint()

    def mouseMoveEvent(self, event):
        global globalx, globaly
        [nx, ny] = rotate(
            [event.position().x(), event.position().y()],
            -globalrotate,
            [300.0, 300.0],
        )
        x = nx - 300
        y = 300 - ny
        globalx = x
        globaly = y
        movenormal(record=True)
        label.repaint()

    def wheelEvent(self, event):
        global globalx, globaly, globalz, globalclaw
        [nx, ny] = rotate(
            [event.position().x(), event.position().y()],
            -globalrotate,
            [300.0, 300.0],
        )
        x = nx - 300
        y = 300 - ny
        delta = event.angleDelta().y()
        if (globalclaw > -120 and delta < 0) or (globalclaw < 120 and delta > 0):
            globalclaw += delta / 60
            print("delta=", delta, " globalclaw=", globalclaw)
        globalx = x
        globaly = y
        movenormal(record=True)
        label.repaint()

    def keyPressEvent(self, event):
        keyProcess(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        pen = QtGui.QPen()
        pen.setWidth(4)
        pen.setColor(QtGui.QColor("red"))
        painter.setPen(pen)
        point0 = [300, 300]
        point1 = [600, 300]
        point2 = [600, 300]
        point1 = rotate(point1, 125.0 + globalrotate, [300.0, 300.0])
        point2 = rotate(point2, -125.0 + globalrotate, [300.0, 300.0])
        painter.drawLine(point0[0], point0[1], point1[0], point1[1])
        painter.drawLine(point0[0], point0[1], point2[0], point2[1])
        smallr = math.sqrt(300 * 300 - globalz * globalz)
        painter.drawEllipse(300 - smallr, 300 - smallr, smallr * 2, smallr * 2)
        infotext = (
            "WS - вверх/вниз\n"
            "AD - поворот запястья\n"
            "Esc - поднять вверх\n"
            "L - запустить программу\n"
            "OP - повернуть координаты\n"
            "ctrl-1,2,3 - сохранить программу (legacy)\n"
            "1,2,3 - загрузить программу (legacy)\n"
            "K - начать запись программы заново"
        )
        painter.drawText(10, 10, 300, 300, QtCore.Qt.AlignTop, infotext)
        pointprev = rotate(
            [globalx + 300, -globaly + 300],
            globalrotate,
            [300.0, 300.0],
        )
        painter.drawPoint(pointprev[0], pointprev[1])


class MainWindow(QWidget):
    def __int__(self):
        super().__init__()

    def keyPressEvent(self, event):
        keyProcess(event)


# ----------------------------------------------------------------------
# Program save/load (.pr in programs/)
# ----------------------------------------------------------------------


def program(name, speed=1):
    """Load program from programs/<name>.pr and start playback."""
    global commandsequence, capture_events, current_program_name
    current_program_name = name
    fname = os.path.join(PROGRAM_DIR, name + ".pr")
    with open(fname, "rb") as fp:
        data = pickle.load(fp)
    if isinstance(data, dict):
        commandsequence = data.get("commandsequence", [[0, 0.0]])
        capture_events = data.get("capture_events", [])
    else:
        commandsequence = data
        capture_events = []
    print("программа " + name + " загружена из " + fname)
    start_programrun(speed)


def saveprogram(name):
    """Save commandsequence + capture_events to programs/<name>.pr."""
    global current_program_name
    current_program_name = name
    fname = os.path.join(PROGRAM_DIR, name + ".pr")
    data = {
        "commandsequence": commandsequence,
        "capture_events": capture_events,
    }
    with open(fname, "wb") as fp:
        pickle.dump(data, fp)
    print("программа " + name + " сохранена в " + fname)


programname1 = QLineEdit("program1")
programname2 = QLineEdit("program2")
programname3 = QLineEdit("program3")

code = QPlainTextEdit()
label = SquareLabel("test")
controlback = QPushButton("начать запись программы")


def program1(btn):
    name = programname1.text()
    btn.setText(name)
    program(name)


def program2(btn):
    name = programname2.text()
    btn.setText(name)
    program(name)


def program3(btn):
    name = programname3.text()
    btn.setText(name)
    program(name)


def program1save():
    name = programname1.text()
    saveprogram(name)


def program2save():
    name = programname2.text()
    saveprogram(name)


def program3save():
    name = programname3.text()
    saveprogram(name)


def runprogram(speed=1):
    text = code.toPlainText().split()
    for v in text:
        program(v, speed)


def controlbackfun():
    global commandsequence, capture_events, current_program_name
    label.setFocus()
    commandsequence = [[0, 0.0]]
    capture_events = []
    current_program_name = "unnamed"


# ----------------------------------------------------------------------
# Layout / UI
# ----------------------------------------------------------------------

window = MainWindow()
window.setWindowTitle("Bublik Control")
layout = QHBoxLayout()
buttoncolumn = QGridLayout()
rightpanel = QVBoxLayout()

canvas = QtGui.QPixmap(600, 600)

label.setFixedSize(600, 600)
label.setPixmap(canvas)

buttonprogram1 = QPushButton("program1")
buttonprogram1.clicked.connect(lambda: program1(buttonprogram1))
buttonprogram2 = QPushButton("program2")
buttonprogram2.clicked.connect(lambda: program2(buttonprogram2))
buttonprogram3 = QPushButton("program3")
buttonprogram3.clicked.connect(lambda: program3(buttonprogram3))

buttonprogram1save = QPushButton("сохранить 1")
buttonprogram1save.clicked.connect(program1save)
buttonprogram2save = QPushButton("сохранить 2")
buttonprogram2save.clicked.connect(program2save)
buttonprogram3save = QPushButton("сохранить 3")
buttonprogram3save.clicked.connect(program3save)

buttonrunprogram = QPushButton("Запустить")
buttonrunprogram.clicked.connect(lambda: runprogram(1))

buttonrunprogramfast = QPushButton("Запустить быстро")
buttonrunprogramfast.clicked.connect(lambda: runprogram(2))

controlback.clicked.connect(controlbackfun)

layout.addWidget(label)

buttoncolumn.addWidget(buttonprogram1, 0, 0)
buttoncolumn.addWidget(buttonprogram2, 1, 0)
buttoncolumn.addWidget(buttonprogram3, 2, 0)
buttoncolumn.addWidget(buttonprogram1save, 0, 1)
buttoncolumn.addWidget(buttonprogram2save, 1, 1)
buttoncolumn.addWidget(buttonprogram3save, 2, 1)
buttoncolumn.addWidget(programname1, 0, 2)
buttoncolumn.addWidget(programname2, 1, 2)
buttoncolumn.addWidget(programname3, 2, 2)

rightpanel.addLayout(buttoncolumn)
rightpanel.addWidget(controlback)
rightpanel.addWidget(bar)
rightpanel.addWidget(code)
rightpanel.addWidget(buttonrunprogram)
rightpanel.addWidget(buttonrunprogramfast)

camera_panel = QVBoxLayout()
init_cameras(camera_panel)
rightpanel.addLayout(camera_panel)

layout.addLayout(rightpanel)
window.setLayout(layout)

movenormal(record=False)  # initial pose without recording

label.show()
window.show()
label.repaint()

sys.exit(app.exec())
