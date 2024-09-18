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
    startBumper = Trigger(_dBump.pressing)

    _abump = Bumper(brain.three_wire_port.a)
    fwBump = Trigger(_abump.pressing)


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
        self.leftMotor = Motor(Ports.PORT1)
        self.rightMotor = Motor(Ports.PORT10)

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


class SensorTests(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)
        self.light = Line(brain.three_wire_port.g)

    def getReflect(self) -> int:
        return self.light.reflectivity()

    def periodic(self) -> None:
        brain.screen.print_at(self.getReflect(), x=200, y=50)


class ScoopSubsystem(Subsystem):
    scoopTolerance = 0.1  # Turns

    class ScoopState:
        Up = 0
        Down = 1
        Hold = 2

    def __init__(self):
        Subsystem.__init__(self)
        self.scoopMotor = Motor(Ports.PORT8)

        self.state = ScoopSubsystem.ScoopState.Up
        self.target = 0

    def setTarget(self, state):
        self.state = state
        if self.state == ScoopSubsystem.ScoopState.Up:
            self.target = 0
        elif self.state == ScoopSubsystem.ScoopState.Down:
            self.target = -2.4
        elif self.state == ScoopSubsystem.ScoopState.Hold:
            self.target = -2.20

    def periodic(self) -> None:
        if self.state == ScoopSubsystem.ScoopState.Up:
            self.target = 0
        elif self.state == ScoopSubsystem.ScoopState.Down:
            self.target = -2.4
        elif self.state == ScoopSubsystem.ScoopState.Hold:
            self.target = -2.20

        self.scoopMotor.spin_to_position(self.target, TURNS, wait=False)

        brain.screen.print_at(self.scoopMotor.torque(), x=100, y=100)
        brain.screen.print_at(self.scoopMotor.position(), x=100, y=150)

    def atPosition(self) -> bool:
        return (
            abs(self.scoopMotor.position(TURNS) - self.target)
            < ScoopSubsystem.scoopTolerance
        )


class RaiseScoop(Command):
    def __init__(self, scoop) -> None:
        Command.__init__(self)
        self.scoop = scoop
        self.started = False

    def initialize(self):
        self.started = False

    def periodic(self):
        self.scoop.setTarget(ScoopSubsystem.ScoopState.Up)
        self.started = True

    def isFinished(self) -> bool:
        return self.scoop.atPosition() and self.started


class LowerScoop(Command):
    def __init__(self, scoop) -> None:
        Command.__init__(self)
        self.scoop = scoop
        self.started = False

    def initialize(self):
        self.started = False

    def periodic(self):
        self.scoop.setTarget(ScoopSubsystem.ScoopState.Down)
        self.started = True

    def isFinished(self) -> bool:
        return self.scoop.atPosition() and self.started


class HoldScoop(Command):
    def __init__(self, scoop) -> None:
        Command.__init__(self)
        self.scoop = scoop
        self.started = False

    def initialize(self):
        self.started = False

    def periodic(self):
        self.scoop.setTarget(ScoopSubsystem.ScoopState.Hold)
        self.started = True

    def isFinished(self) -> bool:
        return self.scoop.atPosition() and self.started


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


class StopMotors(Command):
    def __init__(self, drive: DriveSubsystem) -> None:
        Command.__init__(self)
        self.drive = drive

    def initialize(self):
        self.drive.leftMotor.stop()
        self.drive.rightMotor.stop()

    def isFinished(self) -> bool:
        return True


drive = DriveSubsystem()
scoop = ScoopSubsystem()
sens = SensorTests()

ControllerTriggers.buttonR2.onTrue = RaiseScoop(scoop)
ControllerTriggers.buttonR1.onTrue = LowerScoop(scoop)
ControllerTriggers.buttonL1.onTrue = HoldScoop(scoop)

ControllerTriggers.buttonA.onTrue = TankDrive(
    drive, ControllerTriggers.axis3, ControllerTriggers.axis2
)

Bumpers.startBumper.onTrue = SequentialCommandGroup(
    [
        LowerScoop(scoop),
        RunUntil(
            DriveBack(drive, 100 / 2.54 / 4 / pi * 5), lambda: sens.getReflect() > 30
        ),
        DriveBack(drive, -2 / 4 / pi * 5),
        StopMotors(drive),
        HoldScoop(scoop),
    ]
)
Bumpers.fwBump.onTrue = DriveBack(drive, 10)

ControllerTriggers.buttonB.onTrue = DriveBack(drive, 10)
ControllerTriggers.buttonX.onTrue = SequentialCommandGroup(
    [DriveBack(drive, 5), DriveBack(drive, -5)]
)

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
