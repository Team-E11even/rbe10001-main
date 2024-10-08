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


class RepeatingCommandGroup(Command):
    def __init__(self, commands: Command) -> None:
        Command.__init__(self)
        self.comm = commands

        reqs = self.comm.requirements
        self.addRequirements(reqs)

    def initialize(self):
        self.comm.initialize()

    def periodic(self):
        self.comm.periodic()
        if self.comm.isFinished():
            self.comm.end()
            self.comm.initialize()


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


def optimizeAngle(currentAngle: float, targetAngle: float) -> float:

    tau = 2 * math.pi
    closestFullRotation = (
        math.floor(abs(currentAngle / tau)) * (-1 if currentAngle < 0 else 1) * tau
    )

    currentOptimalAngle = targetAngle + closestFullRotation - currentAngle

    potentialNewAngles = [
        currentOptimalAngle,
        currentOptimalAngle - tau,
        currentOptimalAngle + tau,
    ]  # closest other options

    deltaAngle = tau  # max possible error, a full rotation!
    for potentialAngle in potentialNewAngles:
        if abs(deltaAngle) > abs(potentialAngle):
            deltaAngle = potentialAngle

    return deltaAngle + currentAngle


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

        newAngle = optimizeAngle(self.pose.rot, gyro)
        newPose = Pose2d(twistPose.x + self.pose.x, twistPose.y + self.pose.y, newAngle)

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

        self.active = True

    def periodic(self):
        if self.active:
            self.shoulderMotor.spin_to_position(
                self.target.shoulder, RotationUnits.REV, wait=False
            )
            self.elbowMotor.spin_to_position(
                self.target.elbow, RotationUnits.REV, wait=False
            )
            self.wristMotor.spin_to_position(
                self.target.wrist + self.flickAmount, RotationUnits.REV, wait=False
            )
        else:
            self.shoulderMotor.stop(COAST)
            self.elbowMotor.stop(COAST)
            self.wristMotor.stop(COAST)

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

    def wristAtTarget(self) -> bool:
        return abs(
            self.target.wrist
            + self.flickAmount
            - self.wristMotor.position(RotationUnits.REV)
        ) < 0.01


class WaitForWrist(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def isFinished(self):
        return self.arm.wristAtTarget()


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
    def __init__(
        self, drive: DriveSubsystem, distance: float, speed: float = 25
    ) -> None:
        Command.__init__(self)
        self.distanceTarget = distance
        self.leftcurrentDistance = 0
        self.rightcurrentDistance = 0
        self.drive = drive

        self.speed = speed

        self.done = False

        self.addRequirements([drive])

    def initialize(self):
        print("Starting")
        self.leftDistanceTarget = (
            self.drive.leftMotor.position(TURNS) - self.distanceTarget
        )
        self.drive.leftMotor.spin_to_position(
            self.leftDistanceTarget, TURNS, self.speed, wait=False
        )

        self.rightDistanceTarget = (
            self.drive.rightMotor.position(TURNS) + self.distanceTarget
        )
        self.drive.rightMotor.spin_to_position(
            self.rightDistanceTarget, TURNS, self.speed, wait=False
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


class DriveToAngle(Command):
    def __init__(
        self,
        drive: DriveSubsystem,
        targetAngle: float,
        pGain: float,
        maxspeed: float = 30,
    ) -> None:
        Command.__init__(self)
        self.drive = drive
        self.target = targetAngle
        self.kP = pGain
        self.maxspeed = maxspeed
        self.addRequirements([self.drive])

    def periodic(self):
        driveAngle = self.drive.odometry.pose.rot
        error = optimizeAngle(driveAngle, self.target) - driveAngle

        effort = min(self.kP * -error, self.maxspeed)

        self.drive.setSpeed(-effort, effort)

    def isFinished(self):
        driveAngle = self.drive.odometry.pose.rot
        error = optimizeAngle(driveAngle, self.target) - driveAngle
        return abs(error) < 0.02

    def end(self):
        self.drive.setSpeed(0, 0)


class DriveBucket(Command):
    def __init__(
        self, drive: DriveSubsystem, came, control: Callable[[], float]
    ) -> None:
        Command.__init__(self)
        self.drive = drive
        self.cam = came
        self.control = control
        self.done = False

        self.addRequirements([self.drive, self.cam])

    def initialize(self):
        self.done = False

    def periodic(self):
        offset = self.cam.getBuckerRelative()

        if offset is not None:
            offsetY, offsetX = offset
            turn = offsetX * 80
            fw = self.control()
            # fw = (offset[1] - 4)
            self.drive.setSpeed(fw - turn, fw + turn)
        else:
            self.drive.setSpeed(-20, 20)

    def isFinished(self):
        return self.cam.ultrasonic.distance(DistanceUnits.CM) < 5

    def end(self):
        self.drive.setSpeed(0, 0)


def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class DriveCamAligned(Command):
    def __init__(
        self,
        drive: DriveSubsystem,
        arm: ArmSubsystem,
        came,
        control: Callable[[], float],
    ) -> None:
        Command.__init__(self)
        self.drive = drive
        self.cam = came
        self.arm = arm
        self.control = control
        self.done = False

        self.resetting = False

        self.theTimer = Timer()
        self.oscillating = False

        self.addRequirements([self.drive, self.cam])

        self.changeHeight = 0
        self.didOscillate = False

    def initialize(self):
        self.done = False
        self.didOscillate = False
        self.changeHeight = 0
        arm.setArmAngles(*SetMid.angles)

    def periodic(self):
        offset = self.cam.getDetectedRelative()

        if self.cam.sensorDetected():
            arm.flickAmount = 0.4
            self.done = True
        else:
            if self.didOscillate:
                arm.flickAmount = 0.12
            else:
                arm.flickAmount = 0

        # if self.resetting:
        #     self.drive.setSpeed(0,0)
        #     arm.setArmAngles(*SetTop.angles)
        #     arm.flickAmount = 0.4
        #     if arm.wristAtTarget():
        #         self.resetting = False
        #         arm.flickAmount = 0

        # if not self.resetting and not arm.wristAtTarget():
        #     return

        if offset is not None:
            offsetY, offsetX = offset

            if offsetY > 0.15 and arm.wristAtTarget() and abs(arm.flickAmount) < 0.01:
                self.changeHeight += 1
                if self.changeHeight > 10:
                    arm.setArmAngles(*SetTop.angles)
            elif offsetY < -0.25:
                self.changeHeight -= 1
                if self.changeHeight < -10:
                    arm.setArmAngles(*SetMid.angles)


            # if not self.resetting:
            turn = offsetX * 60
            fw = 30 * (max(map_range(abs(offsetX), 0, 0.5, 1, 0), 0) ** 2)
            # fw = (offset[1] - 4)
            self.drive.setSpeed(fw - turn, fw + turn)

            self.oscillating = False
        else:
            if not self.oscillating:
                self.oscillating = True
                self.theTimer.reset()

            arm.flickAmount = -0.2

            t = self.theTimer.value() + 0.5
            if t > 1:
                self.didOscillate = True
            vel = 4 * t * math.cos(4 * t) + math.sin(4 * t)
            self.drive.setSpeed(-vel, vel)

    def isFinished(self):
        return self.done

    def end(self):
        self.drive.setSpeed(0, 0)


# ARM COMMANDS


class SetTop(Command):
    angles = (1.02, 1.71, 1.85)

    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.setArmAngles(*self.angles)

    def isFinished(self):
        return True


class SetMid(Command):
    angles = (0.8, 2.09, 1.78)

    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.setArmAngles(*self.angles)

    def isFinished(self):
        return True


class SetSafe(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.setArmAngles(0.6, 2.10, 1.72)

    def isFinished(self):
        return True


class SetLow(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.setArmAngles(1.10, 2.39, 2.70)

    def isFinished(self):
        return True


class SetPush(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.setArmAngles(1.43, 2.39, 2.70)

    def isFinished(self):
        return True


class SetBucketLook(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.setArmAngles(1.60, 1.74, 2.14)

    def isFinished(self):
        return True


class SetFlick(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.flickAmount = 0.4

    def isFinished(self):
        return True


class ClearFlick(Command):
    def __init__(self, arm: ArmSubsystem) -> None:
        Command.__init__(self)
        self.arm = arm

        self.addRequirements([arm])

    def initialize(self):
        self.arm.flickAmount = 0

    def isFinished(self):
        return True


class CameraSubsystem(Subsystem):
    class CameraMode:
        GREEN = 1
        ORANGE = 2
        YELLOW = 3

    def __init__(self):
        Subsystem.__init__(self)
        self.sig_orange = Signature(2, 2569, 5159, 3864, -2487, -2047, -2267, 2.3, 0)
        self.sig_green = Signature(1, -7477, -6735, -7091, -2605, -2119, -2362, 2.5, 0)
        self.sig_yellow = Signature(3, 173, 753, 463, -3765, -3359, -3562, 5, 0)

        self.bucket_pink = Signature(4, 4217, 4627, 4422, 3091, 3759, 3425, 2.5, 1)
        bucket_orange = Signature(5, 4061, 4217, 4139, 2999, 3213, 3106, 2.5, 1)
        self.bucketCode = Code(self.bucket_pink, bucket_orange)
        self.camera = Vision(
            Ports.PORT10,
            40,
            self.sig_green,
            self.sig_orange,
            self.sig_yellow,
            self.bucket_pink,
        )
        self.linePresence = Line(brain.three_wire_port.e)

        self.bLine = Line(brain.three_wire_port.f)

        self.rLine = Line(brain.three_wire_port.g)
        self.lLine = Line(brain.three_wire_port.h)

        self.ultrasonic = Sonar(brain.three_wire_port.c)

        self.sideSonar = Sonar(brain.three_wire_port.a)

        self.detected = None
        self.bucket = None

        self.currentMode = CameraSubsystem.CameraMode.ORANGE

    def getDetected(self):
        if self.detected is None:
            return None
        kTheta = 30 * math.pi / 180
        kPhi = 0
        kObjectHeight = 14.5  # inches

        objectX = self.detected.centerX - (320 / 2)
        objectY = self.detected.centerY - (200 / 2)

        kDegPerPixel = 0.1875
        kRadPerPixel = kDegPerPixel * math.pi / 180

        alpha = -objectY * kRadPerPixel
        beta = -objectX * kRadPerPixel

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
        zFinal = (
            h * math.cos(kPhi)
            + h * math.sin(kPhi) * math.tan(beta) / math.tan(alpha)
            + dz
        )

        return (xFinal, yFinal, zFinal)

    def sensorDetected(self):
        return self.linePresence.reflectivity() > 20

    def getDetectedRelative(self):
        if self.detected is None:
            return None
        objectX = self.detected.centerX - (320 / 2)
        objectY = self.detected.centerY - (200 / 2)

        kDegPerPixel = 0.1875
        kRadPerPixel = kDegPerPixel * math.pi / 180

        alpha = -objectY * kRadPerPixel
        beta = -objectX * kRadPerPixel
        return (alpha, beta)

    def getBuckerRelative(self):
        if self.bucket is None:
            return None
        objectX = self.bucket.centerX - (320 / 2)
        objectY = self.bucket.centerY - (200 / 2)

        kDegPerPixel = 0.1875
        kRadPerPixel = kDegPerPixel * math.pi / 180

        alpha = -objectY * kRadPerPixel
        beta = -objectX * kRadPerPixel
        return (alpha, beta)

    def periodic(self):
        obj = self.camera.take_snapshot(self.currentMode)
        if obj is None:
            self.detected = None
        else:
            self.detected = self.camera.largest_object()

        buck = self.camera.take_snapshot(self.bucket_pink)
        if buck is None:
            self.bucket = None
        else:
            self.bucket = self.camera.largest_object()

        d = self.getDetectedRelative()
        if d is not None:
            brain.screen.print_at(d[0], x=210, y=100)
            brain.screen.print_at(d[1], x=210, y=150)


class ChangeCameraMode(Command):
    def __init__(self, cam: CameraSubsystem, state: CameraSubsystem.CameraMode):
        Command.__init__(self)

        self.cam = cam
        self.state = state

    def initialize(self):
        self.cam.currentMode = self.state

    def isFinished(self):
        return True


class BlockerSubsystem(Subsystem):
    class BlockerState:
        Lower = 0
        Up = 1

    def __init__(self):
        Subsystem.__init__(self)
        self.motor = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
        self.state = BlockerSubsystem.BlockerState.Lower

    def periodic(self):
        if self.state == BlockerSubsystem.BlockerState.Lower:
            self.motor.spin_to_position(0, wait=False)
        if self.state == BlockerSubsystem.BlockerState.Up:
            self.motor.spin_to_position(0.25, RotationUnits.REV, wait=False)


class LiftBlocker(Command):
    def __init__(self, blocker: BlockerSubsystem):
        Command.__init__(self)
        self.blocker = blocker
        self.addRequirements([self.blocker])

    def initialize(self):
        self.blocker.state = BlockerSubsystem.BlockerState.Up

    def isFinished(self):
        return True


class LowerBlocker(Command):
    def __init__(self, blocker: BlockerSubsystem):
        Command.__init__(self)
        self.blocker = blocker
        self.addRequirements([self.blocker])

    def initialize(self):
        self.blocker.state = BlockerSubsystem.BlockerState.Lower

    def isFinished(self):
        return True


class FreeArm(Command):
    def __init__(self, arm: ArmSubsystem):
        Command.__init__(self)
        self.arm = arm
        self.addRequirements([self.arm])

    def initialize(self):
        self.arm.active = False


class ActiveArm(Command):
    def __init__(self, arm: ArmSubsystem):
        Command.__init__(self)
        self.arm = arm
        self.addRequirements([self.arm])

    def initialize(self):
        self.arm.active = True


class DriveLineIgnoring(Command):
    def __init__(self, drive: DriveSubsystem, cam: CameraSubsystem, distance: float):
        Command.__init__(self)
        self.drive = drive
        self.cam = cam
        self.distance = distance
        self.addRequirements([self.drive])

        self.distanceDriven = 0

        self.theTimer = Timer()
        self.oscillating = False

        self.totalTimer = Timer()

        self.leftDistanceTarget = 0
        self.rightDistanceTarget = 0

        self.fw = 40

    def initialize(self):
        self.distanceDriven = 0
        self.leftDistanceTarget = -(self.drive.leftMotor.position(TURNS))
        self.rightDistanceTarget = self.drive.rightMotor.position(TURNS)

        self.totalTimer.reset()

    def periodic(self):
        lDetect = self.cam.lLine.reflectivity()
        rDetect = self.cam.rLine.reflectivity()

        if lDetect < 40 and rDetect < 40:
            if not self.oscillating:
                self.oscillating = True
                self.theTimer.reset()

            t = self.theTimer.value() + 0.5
            vel = 4 * t * math.cos(4 * t) + math.sin(4 * t)
            self.drive.setSpeed(-vel, vel)
        else:
            self.oscillating = False

            err = lDetect - rDetect
            effort = err * 0.1

            self.drive.setSpeed(self.fw - effort, self.fw + effort)

    def isFinished(self):
        return self.totalTimer.value() >= self.distance

    def end(self):
        self.drive.setSpeed(0, 0)


class LineFollow(Command):
    def __init__(self, drive: DriveSubsystem, cam: CameraSubsystem):
        Command.__init__(self)
        self.drive = drive
        self.cam = cam
        self.addRequirements([self.drive])
        self.detectAmount = 0

        self.detectThreshold = 25

        self.fw = 40

        self.theTimer = Timer()
        self.oscillating = False

    def initialize(self):
        self.detectAmount = 0

    def periodic(self):
        lDetect = self.cam.lLine.reflectivity()
        rDetect = self.cam.rLine.reflectivity()

        if lDetect < 40 and rDetect < 40:
            if not self.oscillating:
                self.oscillating = True
                self.theTimer.reset()

            t = self.theTimer.value() + 0.5
            vel = 4 * t * math.cos(4 * t) + math.sin(4 * t)
            self.drive.setSpeed(-vel, vel)
        else:
            self.oscillating = False

            err = lDetect - rDetect
            effort = err * 0.1

            self.drive.setSpeed(self.fw - effort, self.fw + effort)

    def isFinished(self):
        if self.oscillating:
            return False

        if self.cam.sideSonar.distance(DistanceUnits.CM) < 55:
            self.detectAmount += 1
        return self.detectAmount > self.detectThreshold

    def end(self):
        self.drive.setSpeed(0, 0)


class LineFollowToFront(LineFollow):
    def __init__(self, drive: DriveSubsystem, cam: CameraSubsystem):
        LineFollow.__init__(self, drive, cam)
        self.detectThreshold = 5

        self.fw = 30

    def isFinished(self):
        if self.cam.ultrasonic.distance(DistanceUnits.CM) < 5:
            self.detectAmount += 1
        return self.detectAmount > self.detectThreshold


class WaitCommand(Command):
    def __init__(self, amount: float):
        Command.__init__(self)
        self.timer = Timer()
        self.waitAmount = amount

    def initialize(self):
        self.timer.reset()

    def isFinished(self):
        return self.timer.value() >= self.waitAmount


class DriveUntilLine(Command):
    def __init__(self, drive: DriveSubsystem, cam: CameraSubsystem):
        Command.__init__(self)
        self.drive = drive
        self.cam = cam
        self.addRequirements([self.drive])
        self.initialValue = 0.0

    def initialize(self):
        self.initialValue = (
            self.cam.lLine.reflectivity() + self.cam.rLine.reflectivity()
        )

    def periodic(self):
        self.drive.setSpeed(20, 20)

    def isFinished(self):
        currentValue = self.cam.lLine.reflectivity() + self.cam.rLine.reflectivity()
        # return abs(currentValue - self.initialValue) > 20 or self.cam.ultrasonic.distance(DistanceUnits.CM) < 5
        return self.cam.ultrasonic.distance(DistanceUnits.CM) < 5

    def end(self):
        self.drive.setSpeed(0, 0)


drive = DriveSubsystem()
arm = ArmSubsystem()
cam = CameraSubsystem()
blocker = BlockerSubsystem()

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

ControllerTriggers.buttonB.onTrue = LineFollow(drive, cam)
ControllerTriggers.buttonUp.onTrue = SequentialCommandGroup(
    [
        LineFollowToFront(drive, cam),
        DriveBack(drive, -0.5),
        LiftBlocker(blocker),
        WaitCommand(0.2),
        DriveBack(drive, 0.5),
        WaitCommand(1),
        LowerBlocker(blocker),
    ]
)

ControllerTriggers.buttonY.onTrue = SetTop(arm)

ControllerTriggers.buttonR1.onTrue = SetMid(arm)
ControllerTriggers.buttonR2.onTrue = SetLow(arm)
ControllerTriggers.buttonL2.onTrue = SetPush(arm)

ControllerTriggers.buttonL1.onTrue = SetFlick(arm)
ControllerTriggers.buttonL1.onFalse = ClearFlick(arm)

ControllerTriggers.buttonRight.onTrue = LiftBlocker(blocker)
ControllerTriggers.buttonRight.onFalse = LowerBlocker(blocker)

def SeekFruit(offset):
    return SequentialCommandGroup(
        [
            DriveToAngle(drive, math.pi / 2 + offset, 40),
            SetMid(arm),
            SetFlick(arm),
            WaitForWrist(arm),
            ClearFlick(arm),
            DriveCamAligned(drive, arm, cam, ControllerTriggers.axis3),
            WaitForWrist(arm),
            ClearFlick(arm),
            WaitForWrist(arm),
            SetSafe(arm),
            DriveBack(drive, -0.5),
        ]
    )

def GoToWall(offset):
    return SequentialCommandGroup(
        [
            DriveToAngle(drive, -math.pi / 2 + offset, 40),
            DriveUntilLine(drive, cam),
            DriveBack(drive, -0.3),
            DriveToAngle(drive, -math.pi + offset, 40),
        ]
    )

DeliverFruit = SequentialCommandGroup(
    [
        LineFollowToFront(drive, cam),
        DriveBack(drive, -0.5),
        LiftBlocker(blocker),
        WaitCommand(0.2),
        DriveBack(drive, 0.7, 125),
        WaitCommand(1),
        LowerBlocker(blocker),
    ]
)


def GoToClosestWall(dist, offset):
    return SequentialCommandGroup(
        [
            LineFollow(drive, cam),
            DriveLineIgnoring(drive, cam, dist),
            SeekFruit(offset),
            GoToWall(offset),
            DeliverFruit,
        ]
    )


def GoToMiddleFruit(dist, offset):
    return SequentialCommandGroup(
        [
            LineFollow(drive, cam),
            DriveLineIgnoring(drive, cam, 1),
            LineFollow(drive, cam),
            DriveLineIgnoring(drive, cam, dist),
            SeekFruit(offset),
            GoToWall(offset),
            DeliverFruit,
        ]
    )


def GoToFarFruit(dist, offset):
    return SequentialCommandGroup(
        [
            LineFollow(drive, cam),
            DriveLineIgnoring(drive, cam, 1),
            LineFollow(drive, cam),
            DriveLineIgnoring(drive, cam, 1),
            LineFollow(drive, cam),
            DriveLineIgnoring(drive, cam, dist),
            SeekFruit(offset),
            GoToWall(offset),
            DeliverFruit,
        ]
    )


ResetPositioning = SequentialCommandGroup(
    [
        DriveBack(drive, -0.4),
        DriveToAngle(drive, -math.pi / 4, 40),
        DriveBack(drive, 0.5),
        DriveToAngle(drive, 0, 40),
    ]
)

ResetPositioningFar = SequentialCommandGroup(
    [
        DriveBack(drive, -0.4),
        DriveToAngle(drive, math.pi-math.pi / 4, 40),
        DriveBack(drive, 0.5),
        DriveToAngle(drive, math.pi, 40),
    ]
)

GoToOtherSide = SequentialCommandGroup([
    LineFollowToFront(drive,cam),
    DriveToAngle(drive, math.pi/2, 40),
    DriveUntilLine(drive,cam),
    DriveToAngle(drive, math.pi, 40),
])


ControllerTriggers.buttonX.onTrue = RepeatingCommandGroup(
    SequentialCommandGroup(
        [
            ChangeCameraMode(cam, CameraSubsystem.CameraMode.ORANGE),
            SetMid(arm),
            GoToClosestWall(-0.5, 0),
            ResetPositioning,
            ChangeCameraMode(cam, CameraSubsystem.CameraMode.GREEN),
            SetMid(arm),
            GoToClosestWall(0.5,0),
            ResetPositioning,

            ChangeCameraMode(cam, CameraSubsystem.CameraMode.YELLOW),
            SetMid(arm),
            GoToMiddleFruit(-0.5,0),
            ResetPositioning,
            ChangeCameraMode(cam, CameraSubsystem.CameraMode.GREEN),
            SetMid(arm),
            GoToMiddleFruit(0.5,0),
            ResetPositioning,

            ChangeCameraMode(cam, CameraSubsystem.CameraMode.ORANGE),
            SetMid(arm),
            GoToFarFruit(-0.5,0),
            ResetPositioning,
            ChangeCameraMode(cam, CameraSubsystem.CameraMode.YELLOW),
            SetMid(arm),
            GoToFarFruit(0.5,0),
            ResetPositioning,

            GoToOtherSide,

            ChangeCameraMode(cam, CameraSubsystem.CameraMode.ORANGE),
            SetMid(arm),
            GoToClosestWall(-0.5, math.pi),
            ResetPositioningFar,
            ChangeCameraMode(cam, CameraSubsystem.CameraMode.GREEN),
            SetMid(arm),
            GoToClosestWall(0.5, math.pi),
            ResetPositioningFar,

            ChangeCameraMode(cam, CameraSubsystem.CameraMode.YELLOW),
            SetMid(arm),
            GoToMiddleFruit(-0.5, math.pi),
            ResetPositioningFar,
            ChangeCameraMode(cam, CameraSubsystem.CameraMode.GREEN),
            SetMid(arm),
            GoToMiddleFruit(0.5, math.pi),
            ResetPositioningFar,

            ChangeCameraMode(cam, CameraSubsystem.CameraMode.ORANGE),
            SetMid(arm),
            GoToFarFruit(-0.5, math.pi),
            ResetPositioningFar,
            ChangeCameraMode(cam, CameraSubsystem.CameraMode.YELLOW),
            SetMid(arm),
            GoToFarFruit(0.5, math.pi),
            ResetPositioningFar,
        ]
    )
)

ControllerTriggers.buttonLeft.onTrue = FreeArm(arm)
ControllerTriggers.buttonLeft.onFalse = ActiveArm(arm)
ControllerTriggers.buttonDown.onTrue = SetBucketLook(arm)

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
    sleep(10, MSEC)  # time it takes for controllers to update
