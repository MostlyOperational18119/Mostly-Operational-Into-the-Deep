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
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale

@Autonomous(name = "BASKET_Meet2", group = "AAAA")
class BASKET_MEET_2 : DriveMethods() {
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)

        // MOTORS
        val motorFL = hardwareMap.dcMotor["motorFL"]
        val motorFR = hardwareMap.dcMotor["motorFR"]
        val motorBL = hardwareMap.dcMotor["motorBL"]
        val motorBR = hardwareMap.dcMotor["motorBR"]
        val slideVerticalMotor = hardwareMap.dcMotor["slideVertical"]
        val slideHorizontalMotor = hardwareMap.dcMotor["slideHorizontal"]
        val hangerMotor = hardwareMap.dcMotor["hanger"]
        val tapeMeasureRotateMotor = hardwareMap.dcMotor["tapeMeasureRotateMotor"]

        //MOTORS MODES
        motorBR.direction = DcMotorSimple.Direction.REVERSE
        motorFR.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideVerticalMotor)
        setMotorModePosition(hangerMotor)
        slideVerticalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideHorizontalMotor)
        slideHorizontalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModeEncoder(tapeMeasureRotateMotor)
        setMotorModeEncoder(hangerMotor)
        tapeMeasureRotateMotor.targetPosition = 0

        //Servos
        val intakeServo = hardwareMap.crservo["intakeServo"]
        val clawServo = hardwareMap.servo["clawServo"]
        clawServo.position =  clawServoClosed
        val transferServo = hardwareMap.servo["transferServo"]
        transferServo.position = transferUpPos
        val clawRotateServo = hardwareMap.servo["rotateServo"]
        clawRotateServo.position = clawRotateUpRight
        val hangPusher = hardwareMap.servo["hangPusher"]

        drive.poseEstimate = Pose2d(-34.09, -63.19, Math.toRadians(-90.00))

        val traj1: TrajectorySequence? =
            drive.trajectorySequenceBuilder( Pose2d(-34.09, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(-10.04, -42.15))
                .setReversed(false)
                .build()

        val traj2: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-10.04, -42.4,Math.toRadians(-90.0)))
                .splineToLinearHeading(Pose2d(-48.25, -40.5, Math.toRadians(90.0)), Math.toRadians(90.00))
                .build()

        val traj3: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-48.25, -40.5, Math.toRadians(90.00)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val traj4: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .lineToLinearHeading(Pose2d(-57.0, -40.5, Math.toRadians(90.00)))
                .build()

        val traj5: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-57.0, -40.5, Math.toRadians(90.00)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

//        val traj6: TrajectorySequence? =
//            drive.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
//                .lineToLinearHeading(Pose2d(-58.0, -40.5, Math.toRadians(127.00)))
//                .build()
//
//        val traj7: TrajectorySequence? =
//            drive.trajectorySequenceBuilder(Pose2d(-62.0, -40.5, Math.toRadians(127.00)))
//                .setReversed(true)
//                .lineToLinearHeading(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
//                .setReversed(false)
//                .build()

        val traj8: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .lineTo(Vector2d(-43.0, -43.0))
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
        slideVerticalMotor.targetPosition = 1600
        slideVerticalMotor.power = 1.0
        clawRotateServo.position = clawRotateStraight

        drive.followTrajectorySequence(traj1)
        drive.updatePoseEstimate()

        transferServo.position = transferDownPos
        clawServo.position = clawServoClosed
        slideVerticalMotor.targetPosition = 500
        slideVerticalMotor.power = -1.0
        sleep(700)
        clawServo.position = clawServoOpen
        sleep(300)
        clawRotateServo.position = clawRotateUpRight
        intakeServo.power = 1.0

        //GO TO AND PICK UP FIRST SAMPLE
        drive.followTrajectorySequence(traj2)
        drive.updatePoseEstimate()

        intakeServo.power = 1.0
        slideHorizontalMotor.targetPosition = 800
        slideHorizontalMotor.power = 1.0
        sleep(3500)
        intakeServo.power = 0.0
        slideHorizontalMotor.targetPosition = 0
        slideHorizontalMotor.power = -1.0

        //TRANSFER
        clawRotateServo.position = clawRotateUpRight
        slideVerticalMotor.targetPosition = 400
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 1.0 }
        else { slideVerticalMotor.power = -1.0 }
        transferServo.position = transferUpPos
        clawServo.position = clawServoOpen
        clawRotateServo.position = clawRotateRest
        sleep(500)
        slideVerticalMotor.targetPosition = 0
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 1.0 }
        else { slideVerticalMotor.power = -1.0 }
        sleep(500)
        clawServo.position = clawServoClosed
        sleep(500)
        slideVerticalMotor.targetPosition = 3650
        slideVerticalMotor.power = 1.0
        clawRotateServo.position = clawRotateOut
        sleep(1500)
        transferServo.position = transferDownPos

        //MOVE TO HIGH BASKET AND PLACE SAMPLE
        drive.followTrajectorySequence(traj3)
        drive.updatePoseEstimate()

        clawRotateServo.position = clawRotateOut
        sleep(200)
        clawServo.position = clawServoOpen
        sleep(200)
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = -0.5

        //MOVE TO 2nd PIXEL AND PICK UP
        drive.followTrajectorySequence(traj4)

        intakeServo.power = 1.0
        slideHorizontalMotor.targetPosition = 800
        slideHorizontalMotor.power = 1.0
        sleep(3500)
        intakeServo.power = 0.0
        slideHorizontalMotor.targetPosition = 0
        slideHorizontalMotor.power = -1.0

        //TRANSFER
        clawRotateServo.position = clawRotateUpRight
        slideVerticalMotor.targetPosition = 400
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 1.0 }
        else { slideVerticalMotor.power = -1.0 }
        transferServo.position = transferUpPos
        clawServo.position = clawServoOpen
        clawRotateServo.position = clawRotateRest
        sleep(500)
        slideVerticalMotor.targetPosition = 0
        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 1.0 }
        else { slideVerticalMotor.power = -1.0 }
        sleep(500)
        clawServo.position = clawServoClosed
        sleep(500)
        slideVerticalMotor.targetPosition = 3650
        slideVerticalMotor.power = 1.0
        clawRotateServo.position = clawRotateOut
        sleep(1500)
        transferServo.position = transferDownPos

        //MOVE TO HIGH BASKET AND PLACE SECOND SAMPLE
        drive.followTrajectorySequence(traj5)
        drive.updatePoseEstimate()

        clawRotateServo.position = clawRotateOut
        sleep(200)
        clawServo.position = clawServoOpen
        sleep(200)
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = -0.6

        drive.followTrajectorySequence(traj8)
        sleep(200)
        clawRotateServo.position = clawRotateUpRight
        sleep(500)
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = -1.0
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