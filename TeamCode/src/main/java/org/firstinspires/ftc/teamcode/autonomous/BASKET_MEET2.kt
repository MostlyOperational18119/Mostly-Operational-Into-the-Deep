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
        val clawRotateRest = 0.75
        val clawRotateUpRight = 0.6
        val clawRotateOut = 0.1
        val clawRotateStraight = 0.2
        val clawRotateWall = 0.28
        val transferDownPos = 0.57
        val transferMidPos = 0.4
        val transferUpPos = 0.20
        val clawServoOpen = 0.13
        val clawServoClosed = 0.26

        val intakeServo = hardwareMap.crservo["intakeServo"]
        val clawServo = hardwareMap.servo["clawServo"]
        clawServo.position =  clawServoClosed
        val transferServo = hardwareMap.servo["transferServo"]
        transferServo.position = transferUpPos
        val clawRotateServo = hardwareMap.servo["rotateServo"]
        clawRotateServo.position = clawRotateUpRight
        val hangPusher = hardwareMap.servo["hangPusher"]

        drive.poseEstimate = Pose2d(-34.09, -63.19, Math.toRadians(-90.00))
        // Setup up the trajectory sequence (drive path)

        //STILL NEED TO CONNECT EVERYTHING TOGETHER
        val traj1: TrajectorySequence? =
            drive.trajectorySequenceBuilder( Pose2d(-34.09, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(-10.04, -42.4))
                .setReversed(false)
                .build()

        val traj3: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-10.04, -42.4,Math.toRadians(-90.0)))
                .splineToLinearHeading(Pose2d(-48.25, -40.5, Math.toRadians(90.0)), Math.toRadians(90.00))
                .build()

        val traj4: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-48.25, -40.5, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(Vector2d(-52.5, -52.5), Math.toRadians(225.00))
                .setReversed(false)
                .build()

        val traj5: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .splineToLinearHeading(Pose2d(-57.0, -40.5, Math.toRadians(90.00)), Math.toRadians(90.00))
                .build()

        val traj6: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-57.0, -40.5, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(Vector2d(-52.5, -52.5), Math.toRadians(225.00))
                .setReversed(false)
                .build()

        val traj8: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .lineTo(Vector2d(-45.0, -45.0))
                .build()

        val traj9: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-45.0, -45.0, Math.toRadians(45.00)))
                .lineTo(Vector2d(-56.0, -56.0))
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
        slideVerticalMotor.targetPosition = 1750
        slideVerticalMotor.power = 1.0
        clawRotateServo.position = clawRotateStraight

        drive.followTrajectorySequence(traj1)
        drive.updatePoseEstimate()

        transferServo.position = transferDownPos
        clawServo.position = clawServoClosed
        slideVerticalMotor.targetPosition = 500
        slideVerticalMotor.power = -1.0
        sleep(1000)
        clawRotateServo.position = clawRotateStraight
        sleep(500)
        clawServo.position = clawServoOpen
        sleep(500)
        clawRotateServo.position = clawRotateUpRight
        intakeServo.power = 1.0

        //GO TO AND PICK UP FIRST SAMPLE
        drive.followTrajectorySequence(traj3)
        drive.updatePoseEstimate()

        intakeServo.power = 1.0
        slideHorizontalMotor.targetPosition = 800
        slideHorizontalMotor.power = 1.0
        sleep(3000)
        intakeServo.power = 0.0
        slideHorizontalMotor.targetPosition = 0
        slideHorizontalMotor.power = -1.0

        //TRANSFER
        slideVerticalMotor.targetPosition = 400
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 1.0 }
        else { slideVerticalMotor.power = -1.0 }
        transferServo.position = transferUpPos
        clawServo.position = clawServoOpen
        clawRotateServo.position = clawRotateRest
        sleep(500)
        slideVerticalMotor.targetPosition = 30
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 1.0 }
        else { slideVerticalMotor.power = -1.0 }
        sleep(500)
        clawServo.position = clawServoClosed
        sleep(500)
        slideVerticalMotor.targetPosition = 3650
        slideVerticalMotor.power = 1.0
        clawRotateServo.position = clawRotateOut
        transferServo.position = transferDownPos
        sleep(1500)

        //MOVE TO HIGH BASKET AND PLACE SAMPLE
        drive.followTrajectorySequence(traj4)
        drive.updatePoseEstimate()

        clawRotateServo.position = clawRotateOut
        sleep(500)
        clawServo.position = clawServoOpen
        sleep(500)
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = -0.5

        //MOVE TO 2nd PIXEL AND PICK UP
        drive.followTrajectorySequence(traj5)

        intakeServo.power = 1.0
        slideHorizontalMotor.targetPosition = 800
        slideHorizontalMotor.power = 1.0
        sleep(3000)
        intakeServo.power = 0.0
        slideHorizontalMotor.targetPosition = 0
        slideHorizontalMotor.power = -1.0

        //TRANSFER
        slideVerticalMotor.targetPosition = 400
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 1.0 }
        else { slideVerticalMotor.power = -1.0 }
        transferServo.position = transferUpPos
        clawServo.position = clawServoOpen
        clawRotateServo.position = clawRotateRest
        sleep(500)
        slideVerticalMotor.targetPosition = 30
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 1.0 }
        else { slideVerticalMotor.power = -1.0 }
        sleep(500)
        clawServo.position = clawServoClosed
        sleep(500)
        slideVerticalMotor.targetPosition = 3650
        slideVerticalMotor.power = 1.0
        clawRotateServo.position = clawRotateOut
        transferServo.position = transferDownPos
        sleep(1500)

        //MOVE TO HIGH BASKET AND PLACE SECOND SAMPLE
        drive.followTrajectorySequence(traj6)
        drive.updatePoseEstimate()

        clawRotateServo.position = clawRotateOut
        sleep(500)
        clawServo.position = clawServoOpen
        sleep(500)
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = -0.6

        drive.followTrajectorySequence(traj8)
        drive.updatePoseEstimate()
        clawRotateServo.position = clawRotateUpRight
        drive.followTrajectorySequence(traj9)
        drive.updatePoseEstimate()

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