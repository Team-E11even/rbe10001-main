# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       lukem                                                        #
# 	Created:      8/27/2024, 2:35:53 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

from math import sin, cos, atan2, pi

# Brain should be defined by default
brain = Brain()

brain.screen.print("Hello V5")


# COMMAND SCHEDULER
class PIDController:
    def __init__(self, pGain: float, iGain: float, dGain: float):
        self.pGain = pGain
        self.iGain = iGain
        self.dGain = dGain

        self.integrated = 0
        self.previousError = 0

    def calculate(self, reading, target, deltaTime) -> float:
        error = target - reading
        deltaError = error - self.previousError

        self.integrated += error * deltaTime
        effort = (
            self.pGain * error
            + self.iGain * self.integrated
            + self.dGain * deltaError / deltaTime
        )

        self.previousError = error
        return effort


class Scheduler:
    subsystems = []
    scheduledCommands = []
    registeredTriggers = []

    @staticmethod
    def schedule(command):
        commandRequirements = command.requirements

        print("Scheduling", command.__class__.__name__)

        for comm in Scheduler.scheduledCommands:
            for req in commandRequirements:
                if req in comm.requirements:
                    comm.end()
                    comm.cancel()

        Scheduler.scheduledCommands.append(command)
        command.initialize()

    @staticmethod
    def cancel(command):
        print("Canceling", command.__class__.__name__)

        if command in Scheduler.scheduledCommands:
            Scheduler.scheduledCommands.remove(command)


class Subsystem:
    def __init__(self):
        Scheduler.subsystems.append(self)
        self.name = "Subsystem"

    def setName(self, name):
        self.name = name

    def initialize(self) -> None:
        pass

    def periodic(self) -> None:
        pass


class Command:
    def __init__(self) -> None:
        self.requirements = []
        self._running = False
        self.defaultCommand: Command | None = None

    def addRequirements(self, reqs: List[Subsystem]):
        self.requirements.extend(reqs)

    def running(self):
        return self._running

    def cancel(self):
        Scheduler.cancel(self)

    def initialize(self):
        pass

    def periodic(self):
        pass

    def isFinished(self) -> bool:
        return False

    def end(self):
        pass

    def schedule(self):
        Scheduler.schedule(self)


def _emptyFunc():
    pass


emptyCommand = Command()


class Trigger:
    def __init__(self, func: Callable[[], bool]) -> None:
        self.func = func
        self.onTrue = emptyCommand
        self.whileTrue = emptyCommand
        self.onFalse = emptyCommand
        self.whileFalse = emptyCommand

        self.prevState = func()

        Scheduler.registeredTriggers.append(self)

    def poll(self) -> None:
        value = self.func()

        if value:
            if self.whileFalse.running():
                self.whileFalse.cancel()
        else:
            if self.whileTrue.running():
                self.whileTrue.cancel()

        if value and not self.prevState:
            self.onTrue.schedule()
        if not value and self.prevState:
            self.onFalse.schedule()

        self.prevState = value


controller = Controller()


class ControllerTriggers:
    buttonA = Trigger(controller.buttonA.pressing)
    buttonB = Trigger(controller.buttonB.pressing)
    buttonX = Trigger(controller.buttonX.pressing)
    buttonY = Trigger(controller.buttonY.pressing)

    buttonUp = Trigger(controller.buttonUp.pressing)
    buttonDown = Trigger(controller.buttonDown.pressing)
    buttonLeft = Trigger(controller.buttonLeft.pressing)
    buttonRight = Trigger(controller.buttonRight.pressing)

    buttonR1 = Trigger(controller.buttonR1.pressing)
    buttonL1 = Trigger(controller.buttonL1.pressing)
    buttonR2 = Trigger(controller.buttonR2.pressing)
    buttonL2 = Trigger(controller.buttonL2.pressing)

    axis1 = controller.axis1.position
    axis2 = controller.axis2.position
    axis3 = controller.axis3.position
    axis4 = controller.axis4.position


class Bumpers:
    _dBump = Bumper(brain.three_wire_port.b)

    _abump = Bumper(brain.three_wire_port.a)


# COMMAND STRUCTURE BITS
class SequentialCommandGroup(Command):
    def __init__(self, commands: List[Command]) -> None:
        Command.__init__(self)
        self.commands = commands
        self.active_idx = 0

        reqs = []
        for command in self.commands:
            reqs.extend(command.requirements)
        self.addRequirements(reqs)

    def initialize(self):
        print(self.commands)
        self.active_idx = 0
        self.commands[0].initialize()

    def periodic(self):
        self.commands[self.active_idx].periodic()

        if self.commands[self.active_idx].isFinished():
            print("Moving to next command...")
            self.commands[self.active_idx].end()
            self.active_idx += 1
            if self.active_idx < len(self.commands):
                self.commands[self.active_idx].initialize()

    def isFinished(self) -> bool:
        return self.active_idx >= len(self.commands)


class ParallelCommandGroup(Command):
    def __init__(self, commands: List[Command]) -> None:
        Command.__init__(self)
        self.commands = commands

    def initialize(self):
        for command in self.commands:
            command.initialize()

    def periodic(self):
        for command in self.commands:
            command.periodic()

    def end(self):
        for command in self.commands:
            command.end()

    def isFinished(self) -> bool:
        retVal = True
        for command in self.commands:
            if not command.isFinished():
                retVal = False
        return retVal


class RunUntil(Command):
    def __init__(self, command: Command, condition: Callable[[], bool]) -> None:
        Command.__init__(self)
        self.command = command
        self.condition = condition

    def initialize(self):
        return self.command.initialize()

    def periodic(self):
        return self.command.periodic()

    def end(self):
        return self.command.end()

    def isFinished(self) -> bool:
        return self.condition()


# ---------------------
# -your code goes here-
# ---------------------


# found in wpilib ()
class Twist2d:
    dx: float
    dy: float
    dtheta: float

    def __init__(self, dx, dy, dtheta) -> None:
        self.dx = dx
        self.dy = dy
        self.dtheta = dtheta


class Transform2d:
    dx: float
    dy: float
    dtheta: float

    def __init__(self, dx, dy, dtheta) -> None:
        self.dx = dx
        self.dy = dy
        self.dtheta = dtheta


# found in wpilib
# https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/geometry/Pose2d.java
class Pose2d:
    x: float
    y: float
    rot: float

    def __init__(self, x, y, rot) -> None:
        self.x = x
        self.y = y
        self.rot = rot

    def applyTransform(self, transform: Transform2d) -> Pose2d:
        s = sin(self.rot)
        c = cos(self.rot)
        rotatedX = transform.dx * s - transform.dy * c
        rotatedY = transform.dy * s + transform.dx * c
        return Pose2d(self.x + rotatedX, self.y + rotatedY, self.rot + transform.dtheta)

    def exp(self, t: Twist2d):
        dx = t.dx
        dy = t.dy
        dtheta = t.dtheta

        st = sin(dtheta)
        ct = cos(dtheta)

        s = 0
        c = 0
        if abs(dtheta) < 1e-9:
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta
            c = 0.5 * dtheta
        else:
            s = st / ct
            c = (1 - ct) / dtheta

        changeX = dx * s - dy * c
        changeY = dx * c + dy * s
        changeRot = atan2(ct, st)
        return Pose2d(changeX, changeY, changeRot)


class DifferentialDriveKinematics:
    def __init__(self, trackWidth: float) -> None:
        self.trackWidth = trackWidth

    def toTwist2d(self, leftwheel, rightwheel) -> Twist2d:
        return Twist2d(
            (leftwheel + rightwheel) / 2, 0, (rightwheel - leftwheel) / self.trackWidth
        )


class DifferentialDriveOdometry:
    def __init__(
        self,
        kinematics: DifferentialDriveKinematics,
        starting_pos: Pose2d,
        l_encoder: float,
        r_encoder: float,
    ) -> None:
        self.pose = starting_pos
        self.l_encoder = l_encoder
        self.r_encoder = r_encoder
        self.kinematics = kinematics

    def update(self, gyro, left_encoder, right_encoder) -> None:
        dl = left_encoder - self.l_encoder
        dr = right_encoder - self.r_encoder

        twist = self.kinematics.toTwist2d(dl, dr)

        twistPose = self.pose.exp(twist)
        newPose = Pose2d(twistPose.x + self.pose.x, twistPose.y + self.pose.y, gyro)

        self.l_encoder = left_encoder
        self.r_encoder = right_encoder

        self.pose = newPose


driveGearing = 1 / 5


class DriveSubsystem(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)
        self.leftMotor = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
        self.rightMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)

        self.kinematics = DifferentialDriveKinematics(20 * 0.5)
        self.odometry = DifferentialDriveOdometry(
            self.kinematics,
            Pose2d(0, 0, 0),
            self.getLeftInches(),
            self.getRightInches(),
        )

        self.gyro = Inertial(Ports.PORT3)
        print("Calibrating gyro")
        self.gyro.calibrate()
        while self.gyro.is_calibrating():
            sleep(50, MSEC)

        print("calibrated")

    def getLeftInches(self):
        return -self.leftMotor.position(TURNS) * driveGearing * 4 * pi

    def getRightInches(self):
        return self.rightMotor.position(TURNS) * driveGearing * 4 * pi

    def setSpeed(self, l, r) -> None:
        self.leftMotor.spin(REVERSE, l, PERCENT)
        self.rightMotor.spin(FORWARD, r, PERCENT)

    def periodic(self) -> None:
        self.odometry.update(
            self.gyro.heading(RotationUnits.REV) * 2 * pi,
            self.getLeftInches(),
            self.getRightInches(),
        )
        pose = self.odometry.pose
        brain.screen.print_at(pose.x, x=160, y=100)
        brain.screen.print_at(pose.y, x=160, y=150)
        brain.screen.print_at(pose.rot, x=160, y=200)


INCHES_PER_FOOT = 12.0
CENTIMETERS_PER_INCH = 2.54
CENTIMETERS_PER_METER = 100.0

METERS_PER_INCH = CENTIMETERS_PER_INCH / CENTIMETERS_PER_METER
METERS_PER_FOOT = METERS_PER_INCH * INCHES_PER_FOOT


class ArmSubsystem(Subsystem):
    class Constants:
        l1 = 6 * METERS_PER_INCH
        l2 = 8.5 * METERS_PER_INCH
        wrist = 2 * METERS_PER_INCH

    class ArmPositions:
        def __init__(self, shoulder, elbow, wrist, l1_length, l2_length, w_length):
            self.shoulder = shoulder
            self.elbow = elbow
            self.wrist = wrist
            self.l1Length = l1_length
            self.l2Length = l2_length
            self.wLength = w_length

        def l1Location(self) -> Pose2d:
            xPose = self.l1Length * cos(self.shoulder)
            yPose = self.l1Length * sin(self.shoulder)
            return Pose2d(xPose, yPose, self.shoulder + self.elbow)

        def l2Location(self) -> Pose2d:
            l1Pose = self.l1Location()
            l2Transform = Transform2d(self.l2Length, 0, self.wrist)

            return l1Pose.applyTransform(l2Transform)

        def endEffectorPose(self) -> Pose2d:
            l2Pose = self.l2Location()
            wristTransform = Transform2d(self.wLength, 0, 0)

            return l2Pose.applyTransform(wristTransform)

    class ArmState:
        Stowed = 0
        Startup = 1
        Low = 2
        Mid = 3
        High = 4

    def __init__(self):
        Subsystem.__init__(self)
        print("starting drive")
        self.shoulderMotor = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)
        self.elbowMotor = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)
        self.wristMotor = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
        print("motors initialized")

        self.shoulderMotor.set_velocity(100, RPM)
        self.elbowMotor.set_velocity(100, RPM)
        self.wristMotor.set_velocity(100, RPM)

        self.endEffectorPose = Pose2d(0, 0, 0)
        self.target = ArmSubsystem.ArmPositions(
            0,
            0,
            0,
            ArmSubsystem.Constants.l1,
            ArmSubsystem.Constants.l2,
            ArmSubsystem.Constants.wrist,
        )

        self.flickAmount = 0

        print("target")

    def periodic(self):
        # self.shoulderMotor.spin_to_position(
        #     self.target.shoulder, RotationUnits.REV, wait=False
        # )
        # self.elbowMotor.spin_to_position(
        #     self.target.elbow, RotationUnits.REV, wait=False
        # )
        # self.wristMotor.spin_to_position(
        #     self.target.wris + self.flickAmount, RotationUnits.REV, wait=False
        # )

        shoulderPosition = self.shoulderMotor.position(RotationUnits.REV)
        elbowPosition = self.elbowMotor.position(RotationUnits.REV)
        wristPosition = self.wristMotor.position(RotationUnits.REV)

        brain.screen.print_at(shoulderPosition, x=50, y=20)
        brain.screen.print_at(elbowPosition, x=50, y=40)
        brain.screen.print_at(wristPosition, x=50, y=60)

    def setArmPositions(self, pos: ArmPositions):
        self.target = pos

    def setArmAngles(self, shoulder, elbow, wrist):
        self.target.shoulder = shoulder
        self.target.elbow = elbow
        self.target.wrist = wrist

    def getArmPositions(self):
        return self.target


# COMMANDS


class TankDrive(Command):
    def __init__(self, drive, leftFw, rightFw) -> None:
        Command.__init__(self)
        self.leftControl = leftFw
        self.rightControl = rightFw
        self.driveSubsystem = drive

        self.addRequirements([drive])

    def periodic(self):
        self.driveSubsystem.setSpeed(self.leftControl(), self.rightControl())


class DriveBack(Command):
    def __init__(self, drive: DriveSubsystem, distance: float) -> None:
        Command.__init__(self)
        self.distanceTarget = distance
        self.leftcurrentDistance = 0
        self.rightcurrentDistance = 0
        self.drive = drive

        self.done = False

        self.addRequirements([drive])

    def initialize(self):
        print("Starting")
        self.leftDistanceTarget = (
            self.drive.leftMotor.position(TURNS) - self.distanceTarget
        )
        self.drive.leftMotor.spin_to_position(
            self.leftDistanceTarget, TURNS, wait=False
        )

        self.rightDistanceTarget = (
            self.drive.rightMotor.position(TURNS) + self.distanceTarget
        )
        self.drive.rightMotor.spin_to_position(
            self.rightDistanceTarget, TURNS, wait=False
        )
        self.done = False

    def periodic(self):
        leftDist = self.drive.leftMotor.position(TURNS)
        rightDist = self.drive.rightMotor.position(TURNS)
        if (
            abs(self.leftDistanceTarget - leftDist) < 0.01
            and abs(self.rightDistanceTarget - rightDist) < 0.01
        ):
            self.drive.rightMotor.stop()
            self.drive.leftMotor.stop()
            self.done = True

    def end(self):
        print(
            self.rightDistanceTarget,
            self.leftDistanceTarget,
            self.distanceTarget,
            self.done,
        )

    def isFinished(self) -> bool:
        return self.done


class StopMotors(Command):
    def __init__(self, drive: DriveSubsystem) -> None:
        Command.__init__(self)
        self.drive = drive
        self.addRequirements([self.drive])

    def initialize(self):
        self.drive.leftMotor.stop()
        self.drive.rightMotor.stop()

    def isFinished(self) -> bool:
        return True


class DriveCamAligned(Command):
    def __init__(self, drive: DriveSubsystem, came) -> None:
        Command.__init__(self)
        self.drive = drive
        self.cam = came

        self.addRequirements([self.drive, self.cam])

    def periodic(self):
        offset = self.cam.getDetectedRelative()

        if offset is not None:
            offsetX, offsetY = offset
            turn = offset[1] * 100
            fw = 0
            # fw = (offset[1] - 4)
            self.drive.setSpeed(fw - turn, fw + turn)


# ARM COMMANDS


class SetTop(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.setArmAngles(-0.04 / 6, 0.63 / 6, 0.89 / 6)


class SetMid(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.setArmAngles(-0.31 / 6, 1.02 / 6, 0.9 / 6)


class SetLow(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.setArmAngles(-0.58 / 6, 1.12 / 6, 0.97 / 6)


class SetFlick(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.flickAmount = 0.4


class ClearFlick(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.flickAmount = 0


class CameraSubsystem(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)
        self.sig_1 = Signature(1, -7247, -5963, -6605, -2499, -705, -1602, 3.0, 0)
        self.camera = Vision(Ports.PORT16, 50, self.sig_1)

        self.detected = None

    def getDetected(self):
        if self.detected is None:
            return None
        kTheta = 30 * math.pi / 180
        kPhi = 0
        kObjectHeight = 14.5  # inches


        objectX = self.detected.centerX - (320/2)
        objectY = self.detected.centerY - (200/2)

        kDegPerPixel = 0.1875
        kRadPerPixel = kDegPerPixel * math.pi / 180

        alpha = -objectY * kRadPerPixel
        beta = -objectX * kRadPerPixel
        print(alpha, beta)

        h = 2.5  # inches
        dy = -4  # inches
        dx = -6.5  # inches
        dz = 7.75  # inches

        # Don't ask how these numbers appeared
        xFinal = (
            h * math.sin(kPhi) * math.sin(kTheta)
            - h * math.cos(kPhi) * math.sin(kTheta) * math.tan(beta) / math.tan(alpha)
            + dx
            + h * math.cos(kTheta) / math.tan(alpha)
        )
        yFinal = (
            -h * math.cos(kTheta) * math.sin(kPhi)
            + h * math.cos(kPhi) * math.cos(kTheta) * math.tan(beta) / math.tan(alpha)
            + dy
            + h * math.sin(kTheta) / math.tan(alpha)
        )
        zFinal = h * math.cos(kPhi) + h*math.sin(kPhi) * math.tan(beta) / math.tan(alpha) +dz

        return (xFinal, yFinal, zFinal)

    def getDetectedRelative(self):
        if self.detected is None:
            return None
        objectX = self.detected.centerX - (320/2)
        objectY = self.detected.centerY - (200/2)

        kDegPerPixel = 0.1875
        kRadPerPixel = kDegPerPixel * math.pi / 180

        alpha = -objectY * kRadPerPixel
        beta = -objectX * kRadPerPixel
        return (alpha, beta)

    def periodic(self):
        obj = self.camera.take_snapshot(1)
        self.detected = self.camera.largest_object()

        d = self.getDetectedRelative()
        # if obj is not None:
        #     print(obj[0].centerX, obj[0].centerY)
        if d is not None:
            brain.screen.print_at(d[0], x=200, y=100)
            brain.screen.print_at(d[1], x=200, y=150)


drive = DriveSubsystem()
arm = ArmSubsystem()
cam = CameraSubsystem()

ControllerTriggers.buttonA.onTrue = TankDrive(
    drive, ControllerTriggers.axis3, ControllerTriggers.axis2
)

# Bumpers.startBumper.onTrue = SequentialCommandGroup(
#     [
#         DriveBack(drive, -2 / 4 / pi * 5),
#         StopMotors(drive),
#     ]
# )
# Bumpers.fwBump.onTrue = DriveBack(drive, 10)

ControllerTriggers.buttonB.onTrue = DriveBack(drive, 10)
ControllerTriggers.buttonX.onTrue = DriveCamAligned(drive, cam)
ControllerTriggers.buttonY.onTrue = SetTop(arm)

ControllerTriggers.buttonR1.onTrue = SetMid(arm)
ControllerTriggers.buttonR2.onTrue = SetLow(arm)

ControllerTriggers.buttonL1.onTrue = SetFlick(arm)
ControllerTriggers.buttonL1.onFalse = ClearFlick(arm)

# -------------------------------------------------------
# ----------------main loop, do not touch----------------
# -------------------------------------------------------

for subsystem in Scheduler.subsystems:
    subsystem.initialize()

while True:
    for subsystem in Scheduler.subsystems:
        subsystem.periodic()

    for trigger in Scheduler.registeredTriggers:
        trigger.poll()

    for command in Scheduler.scheduledCommands:
        command.periodic()
        if command.isFinished():
            print("Command", command.__class__.__name__)
            command.end()
            command.cancel()
    sleep(25, MSEC)  # time it takes for controllers to update
