package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale

@Autonomous(name = "BASKET_Meet2", group = "AAAA")
class BASKET_MEET_2 : LinearOpMode() {
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)

        fun setMotorModeEncoder(motor: DcMotor) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.power = 0.0
        }

        fun setMotorModePosition(motor: DcMotor) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.targetPosition = 0
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        // MOTORS
        val motorFL = hardwareMap.dcMotor["motorFL"]
        val motorFR = hardwareMap.dcMotor["motorFR"]
        val motorBL = hardwareMap.dcMotor["motorBL"]
        val motorBR = hardwareMap.dcMotor["motorBR"]
        val slideVerticalMotor = hardwareMap.dcMotor["slideVertical"]
        val slideHorizontalMotor = hardwareMap.dcMotor["slideHorizontal"]

        //MOTORS MODES
        setMotorModePosition(slideVerticalMotor)
        slideVerticalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideHorizontalMotor)
        slideHorizontalMotor.direction = DcMotorSimple.Direction.REVERSE

        //Servos
        val clawRotateRest = 0.71
        val clawRotateUpRight = 0.56
        val clawRotateOut = 0.1
        val transferDownPos = 0.57
        val transferMidPos = 0.4
        val transferUpPos = 0.22
        val clawServoOpen = 0.13
        val clawServoClosed = 0.23

        val intakeServo = hardwareMap.crservo["intakeServo"]
        val clawServo = hardwareMap.servo["clawServo"]
        val transferServo = hardwareMap.servo["transferServo"]
        val clawRotateServo = hardwareMap.servo["rotateServo"]
        clawRotateServo.position = clawRotateUpRight
        val hangPusher = hardwareMap.servo["hangPusher"]

        drive.poseEstimate = Pose2d(-34.09, -63.19, Math.toRadians(-90.00))
        // Setup up the trajectory sequence (drive path)

        //STILL NEED TO CONNECT EVERYTHING TOGETHER
        val traj1: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-34.09, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(-10.04, -34.01))
                .build()

        val traj2: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-10.4, -34.01, Math.toRadians(-90.0)))
                .setReversed(false)
                .splineTo(Vector2d(-47.87, -39.89), Math.toRadians(90.00))
                .build()

        val traj3: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-47.87, -39.89, Math.toRadians(90.0)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-55.0, -55.0, Math.toRadians(45.00)))
                .build()

        val traj4: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-50.0, -50.0, Math.toRadians(45.00)))
                .back(5.0)
                .build()

        val traj5: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-57.071, -57.071, Math.toRadians(225.00)))
                .setReversed(false)
                .lineToLinearHeading(Pose2d(-57.75, -38.37, Math.toRadians(90.0)))
                .build()


        val traj6: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-57.75, -38.37, Math.toRadians(90.0)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-50.0, -50.0, Math.toRadians(45.00)))
                .build()

        val traj7: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-50.0, -50.0, Math.toRadians(45.0)))
                .back(5.0)
                .setReversed(false)
                .build()

        val traj8: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-57.071, -57.071, Math.toRadians(45.0)))
                .forward(2.0)
                .build()


        // Tell the User the Robot has been initialized
        telemetry.addData("Status", "Initialized")
        telemetry.update()


        // Wait for the game to start (driver presses PLAY)
        waitForStart()
        telemetry.addLine("started")
        telemetry.update()
        if (isStopRequested) return
        telemetry.addLine("not stopped")
        telemetry.update()


        //PLACE SPECIMEN ON HIGH BAR
        slideVerticalMotor.targetPosition = 2000
        slideVerticalMotor.power = 1.0

        drive.followTrajectorySequence(traj1)

        slideVerticalMotor.targetPosition = 1800
        slideVerticalMotor.power = -0.5
        sleep(200)
        clawServo.position = clawServoOpen
        sleep(200)
        slideVerticalMotor.targetPosition = 2000
        slideVerticalMotor.power = 0.5
        sleep(100)
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = -0.8

        //GO TO AND PICK UP FIRST SAMPLE
        drive.followTrajectorySequence(traj2)

        slideHorizontalMotor.targetPosition = 1000
        slideHorizontalMotor.power = 1.0
        sleep(500)
        intakeServo.power = 1.0
        sleep(1000)
        intakeServo.power = 0.0
        slideHorizontalMotor.targetPosition = 0
        slideHorizontalMotor.power = -1.0

        //TRANSFER
        slideVerticalMotor.targetPosition = 400
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 0.8 }
        else { slideVerticalMotor.power = -0.8 }
        transferServo.position = transferUpPos
        clawServo.position = clawServoOpen
        clawRotateServo.position = clawRotateRest
        sleep(750)
        slideVerticalMotor.targetPosition = 100
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 0.8 }
        else { slideVerticalMotor.power = -0.8 }
        sleep(500)
        clawServo.position = clawServoClosed
        sleep(500)
        slideVerticalMotor.targetPosition = 1000
        slideVerticalMotor.power = 0.8
        sleep(400)
        clawRotateServo.position = clawRotateOut
        transferServo.position = transferDownPos
        sleep(500)

        //MOVE TO HIGH BASKET AND PLACE SAMPLE
        drive.followTrajectorySequence(traj3)

        slideVerticalMotor.targetPosition = 3650
        slideVerticalMotor.power = 1.0
        sleep(1000)

        //MOVE FORWARD SLIGHTLY
        drive.followTrajectorySequence(traj4)

        clawServo.position = clawServoOpen
        sleep(100)

        //MOVE TO 2nd PIXEL AND PICK UP
        drive.followTrajectorySequence(traj5)
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = -1.0
        clawRotateServo.position = clawRotateRest
        slideHorizontalMotor.targetPosition = 1000
        slideHorizontalMotor.power = 1.0
        sleep(500)
        intakeServo.power = 1.0
        sleep(1000)
        intakeServo.power = 0.0
        slideHorizontalMotor.targetPosition = 0
        slideHorizontalMotor.power = -1.0

        //TRANSFER
        slideVerticalMotor.targetPosition = 400
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 0.8 }
        else { slideVerticalMotor.power = -0.8 }
        transferServo.position = transferUpPos
        clawServo.position = clawServoOpen
        clawRotateServo.position = clawRotateRest
        sleep(750)
        slideVerticalMotor.targetPosition = 100
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 0.8 }
        else { slideVerticalMotor.power = -0.8 }
        sleep(500)
        clawServo.position = clawServoClosed
        sleep(500)
        slideVerticalMotor.targetPosition = 1000
        slideVerticalMotor.power = 0.8
        sleep(400)
        clawRotateServo.position = clawRotateOut
        transferServo.position = transferDownPos
        sleep(500)

        //MOVE TO HIGH BASKET AND PLACE SECOND SAMPLE
        drive.followTrajectorySequence(traj6)

        slideVerticalMotor.targetPosition = 3650
        slideVerticalMotor.power = 1.0
        sleep(1000)

        drive.followTrajectorySequence(traj7)

        clawServo.position = clawServoOpen
        sleep(500)

        drive.followTrajectorySequence(traj8)

        clawRotateServo.position = clawRotateRest
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = 0.5
        sleep(500)

        while (opModeIsActive() && !isStopRequested) {
            drive.update()
        }
        telemetry.addLine("tried to run code")
        telemetry.update()
        sleep(1000)
        val (x, y, heading) = drive.poseEstimate
        telemetry.addData(
            "Current Position",
            String.format(Locale.ENGLISH, "X: %f, Y: %f, and Rotation: %f", x, y, heading)
        )
        telemetry.update()
        PoseStorage.currentPose = drive.poseEstimate
    }
}