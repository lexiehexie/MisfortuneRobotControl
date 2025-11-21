import sys
import time

import PySide6
import numpy as nmp
import math
import pickle

import numpy as np
from PySide6 import QtCore, QtGui
from PySide6.QtGui import QPainter
from PySide6.QtWidgets import QApplication, QLabel, QPushButton, QWidget, QHBoxLayout, QVBoxLayout, QProgressBar, \
    QGridLayout, QLineEdit, QPlainTextEdit
import xarm

app = QApplication()
bar = QProgressBar()

arm = xarm.Controller('USB')
globalx: int = 100
globaly: int = 100
globalz: int = 100
globalclaw: float = 0.0
globalduck: float = 0.0
globalwrist: float = 0.0
globaltime: float = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
globalrotate: float = -90.0
commandsequence = [[0, 0.0]]
commandstatus = "record"
comandcounter = 1


def rotate(coords, angle, origin):
    ncoords = np.array([coords[0] - origin[0], coords[1] - origin[1]])
    angle = angle / 57.29
    rotmat = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
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
    # print(nx, ny, nz)
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
    # print(angle[0], angle[1])

    if lenght < nominallenght:
        deltalenght = nominallenght - lenght
        lenghtshort = deltalenght / 195
        if lenghtshort < -1:
            lenghtshort = -1
        if lenghtshort > 1:
            lenghtshort = 1
        maincorangle = 2 * 57.29 * math.asin(lenghtshort)
        if maincorangle > 124:
            maincorangle = 124
        corrangle = [0.0, maincorangle / 2, -maincorangle, -maincorangle / 2]

    angle = nmp.subtract(angle, corrangle)
    # print(angle)

    return angle



def determineXYZfromAngles(angle):
    mindiff = 10000;
    nx = 0
    ny = 0
    nz = 0
    for fx in range(-300, 300, 10):
        for fy in range(-300, 300, 10):
            for fz in range(0, 300, 10):

                fangle = determineAnglesFormXYZ(fx, fy, fz)
                diff = np.sum(np.abs(np.subtract(angle, fangle)))
                if diff < mindiff:
                    #print (np.subtract(angle, fangle), nx, ny, nz)
                    mindiff = diff
                    nx = fx
                    ny = fy
                    nz = fz
    return [nx, ny, nz]

fangle = [0.0, 0.0, 0.0, 0.0]
#fangle[0] = arm.getPosition(6, True)
#fangle[1] = arm.getPosition(5, True)
#fangle[2] = - arm.getPosition(4, True)
#fangle[3] = - arm.getPosition(3, True)
print(fangle)
rescoord = determineXYZfromAngles(fangle)
globalx = rescoord[0]
globaly = rescoord[1]
globalz = rescoord[2]


def armSetPosition(motid, angle, speed, wait):
    global globaltime, globalduck, commandsequence, comandcounter, globalclaw, globalwrist
    if math.fabs(globaltime[motid] - time.time()) > 0.5 or commandstatus == "play":
        if motid == 3:
            angle += globalduck
        if commandstatus == "play":
            arm.setPosition(commandsequence[comandcounter][0], commandsequence[comandcounter][1], speed, wait)
            print(commandsequence[comandcounter][0], commandsequence[comandcounter][1], speed, wait)
            comandcounter += 1
            if comandcounter > len(commandsequence) - 1:
                commandstatus == "record"
                comandcounter = 0

        if commandstatus == "record":
            arm.setPosition(motid, angle, speed, wait)
            print(motid, angle, speed, wait)
            commandsequence.append([motid, angle])
        globaltime[motid] = time.time()


def movenormal():
    global globalclaw, globalwrist
    angles = determineAnglesFormXYZ(globalx, globaly, globalz)
    for u in [0, 1]:
        armSetPosition(6 - u, angles[u], 500, False)
    for u in [2, 3]:
        armSetPosition(6 - u, -angles[u], 500, False)
    armSetPosition(1, globalclaw, 500, False)
    armSetPosition(2, globalwrist, 500, False)


def programrun(progid, speed=1):
    print(speed)
    global commandstatus, globalx, globaly, globalz, globalclaw, globalduck, globalwrist
    globalx = 100
    globaly = 100
    globalz = 100
    globalclaw = 0.0
    globalduck = 0.0
    globalwrist = 0.0
    timex: float = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    commandstatus = "play"
    qi = 0
    bar.setValue(0)
    for w in commandsequence:
        qi += 1
        bar.setValue(round(100 * (qi / len(commandsequence))))
        delay = 0.5 / speed
        if math.fabs(timex[w[0]] - time.time()) > 0.5:
            delay = 0.01 / speed
        time.sleep(delay)
        timex[w[0]] = time.time()
        armSetPosition(0, 0.0, round(500 / speed), False)
    commandstatus = "record"


def keyProcess(event):
    global globalx, globaly, globalz, globalclaw, globalwrist, globalrotate, commandstatus, commandsequence
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
    if event.key() == QtCore.Qt.Key_1 and event.modifiers() == QtCore.Qt.ControlModifier:
        with open('listfile1', 'wb') as fp:
            pickle.dump(commandsequence, fp)
            print("программа 1 сохранена")
    if event.key() == QtCore.Qt.Key_1 and event.modifiers() != QtCore.Qt.ControlModifier:
        with open('listfile1', 'rb') as fp:
            commandsequence = pickle.load(fp)
            print("программа 1 загружена")
    if event.key() == QtCore.Qt.Key_2 and event.modifiers() == QtCore.Qt.ControlModifier:
        with open('listfile2', 'wb') as fp:
            pickle.dump(commandsequence, fp)
            print("программа 2 сохранена")
    if event.key() == QtCore.Qt.Key_2 and event.modifiers() != QtCore.Qt.ControlModifier:
        with open('listfile2', 'rb') as fp:
            commandsequence = pickle.load(fp)
            print("программа 2 загружена")
    if event.key() == QtCore.Qt.Key_3 and event.modifiers() == QtCore.Qt.ControlModifier:
        with open('listfile3', 'wb') as fp:
            pickle.dump(commandsequence, fp)
            print("программа 3 сохранена")
    if event.key() == QtCore.Qt.Key_3 and event.modifiers() != QtCore.Qt.ControlModifier:
        with open('listfile3', 'rb') as fp:
            commandsequence = pickle.load(fp)
            print("программа 3 загружена")
    if event.key() == QtCore.Qt.Key_Escape:
        globalx, globaly, globalz, globalclaw, globalwrist = [0.0, 0.0, 290.0, 0.0, 0.0]
    if event.key() == QtCore.Qt.Key_K:
        commandsequence = [[0, 0.0]]

    movenormal()
    label.repaint()


class SquareLabel(QLabel):
    def __init__(self, parent=None):
        super(SquareLabel, self).__init__(parent)

    def mouseDoubleClickEvent(self, event):
        global globalduck
        if event.buttons() == QtCore.Qt.LeftButton:
            globalduck += 5.0
        if event.buttons() == QtCore.Qt.RightButton:
            globalduck -= 5.0
        # print(globalduck)
        movenormal()
        label.repaint()

    def mouseMoveEvent(self, event):
        global globalx, globaly, globalz, globalclaw
        # print(event)
        [nx, ny] = rotate([event.position().x(), event.position().y()], -globalrotate, [300.0, 300.0])
        x = nx - 300
        y = 300 - ny
        globalx = x
        globaly = y

        # print(angles)
        movenormal()
        label.repaint()

    def wheelEvent(self, event):
        global globalx, globaly, globalz, globalclaw
        [nx, ny] = rotate([event.position().x(), event.position().y()], -globalrotate, [300.0, 300.0])
        x = nx - 300
        y = 300 - ny
        delta = event.angleDelta().y()
        z = globalz
        if (globalclaw > -120 and delta < 0) or (globalclaw < 120 and delta > 0):
            globalclaw += delta / 60
            print("delta=",delta," globalclaw=",globalclaw)
        angles = determineAnglesFormXYZ(x, y, z)
        globalx = x
        globaly = y
        # print(angles)
        movenormal()
        label.repaint()

    def keyPressEvent(self, event):
        keyProcess(event)

    def paintEvent(self, event):
        # print("painted")
        painter = QPainter(self)
        pen = QtGui.QPen()
        pen.setWidth(4)
        pen.setColor(QtGui.QColor('red'))
        painter.setPen(pen)
        point0 = [300, 300]
        point1 = [600, 300]
        point2 = [600, 300]
        # point0 = rotate(point0, -125.0)
        point1 = rotate(point1, 125.0 + globalrotate, [300.0, 300.0])
        point2 = rotate(point2, -125.0 + globalrotate, [300.0, 300.0])
        painter.drawLine(point0[0], point0[1], point1[0], point1[1])
        painter.drawLine(point0[0], point0[1], point2[0], point2[1])
        smallr = math.sqrt(300 * 300 - globalz * globalz)
        painter.drawEllipse(300 - smallr, 300 - smallr, smallr * 2, smallr * 2)
        infotext = "WS -вверх, вниз\n AD - поворот запястья \n esc - поднять вверх \n L - запустить программу"
        infotext += "\nOP - повернуть координаты \n ctrl-1,2,3 - сохранить программу \n 1,2,3 - загрузить программу"
        infotext += "\n K - начать заново запись программы"
        painter.drawText(10, 10, 300, 300, QtCore.Qt.AlignTop, infotext)
        pointprev = rotate([globalx + 300, -globaly + 300], globalrotate, [300.0, 300.0])
        painter.drawPoint(pointprev[0], pointprev[1])


class MainWindow(QWidget):
    def __int__(self):
        super().__init__()

    def keyPressEvent(self, event):
        keyProcess(event)


def program(name, speed=1):
    global commandsequence
    with open(name + ".pr", 'rb') as fp:
        commandsequence = pickle.load(fp)
        print("программа " + name + " загружена")
        programrun(0, speed)


def saveprogram(name):
    with open(name + ".pr", 'wb') as fp:
        pickle.dump(commandsequence, fp)
        print("программа " + name + " сохранена")


programname1 = QLineEdit("program1")
programname2 = QLineEdit("program2")
programname3 = QLineEdit("program3")

code = QPlainTextEdit()
label = SquareLabel("test")
controlback = QPushButton("начать запись программы")


def program1(self):
    name = programname1.text()
    self.setText(name)
    program(name)


def program2(self):
    name = programname2.text()
    self.setText(name)
    program(name)


def program3(self):
    name = programname3.text()
    self.setText(name)
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
    global commandsequence
    label.setFocus()
    commandsequence = [[0, 0.0]]


window = MainWindow()
window.setWindowTitle('Bublik Control')
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
rightpanel.addLayout((buttoncolumn))
rightpanel.addWidget(controlback)
rightpanel.addWidget(bar)
rightpanel.addWidget(code)
rightpanel.addWidget(buttonrunprogram)
rightpanel.addWidget(buttonrunprogramfast)

layout.addLayout(rightpanel)
window.setLayout(layout)
movenormal()

# label.setMouseTracking(True)
# button.show()
label.show()
window.show()

label.repaint()

app.exec()
