package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import java.util.Locale


@Autonomous(name = "BAR_Meet2", group = "AAAA")
class BAR_MEET2 : Methods() {
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)

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
        val intakeServo = hardwareMap.crservo["intakeServo"]
        val clawServo = hardwareMap.servo["clawServo"]
        val transferServo = hardwareMap.servo["transferServo"]
        transferServo.position = transferUpPos
        val clawRotateServo = hardwareMap.servo["rotateServo"]
        clawRotateServo.position = clawRotateUpRight
        val hangPusher = hardwareMap.servo["hangPusher"]

        drive.poseEstimate = Pose2d(12.08, -63.19, Math.toRadians(-90.00))
        
        val traj1 =
            drive.trajectorySequenceBuilder(Pose2d(12.08, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .splineTo(Vector2d(9.33, -42.4), Math.toRadians(90.0))
                .setReversed(false)
                .build()

        val traj2 =
            drive.trajectorySequenceBuilder(Pose2d(9.33,-42.4 ,Math.toRadians(-90.0)))
                .lineTo(Vector2d(32.12, -48.46))
                .lineTo(Vector2d(37.42, -10.12))
                .lineTo(Vector2d(43.44, -11.49))
                .lineTo(Vector2d(48.62, -59.23))
                .lineTo(Vector2d(43.64, -10.90))
                .lineTo(Vector2d(54.71, -11.10))
                .lineTo(Vector2d(56.09, -60.61))
                .lineTo(Vector2d(56.28, -63.16))
                .build()

        val traj3 =
            drive.trajectorySequenceBuilder(Pose2d(56.28, -63.16 ,Math.toRadians(90.0)))
                .lineTo(Vector2d(9.14, -32.71))
                .build()

        val traj4 =
            drive.trajectorySequenceBuilder(Pose2d(9.14, -32.71 ,Math.toRadians(90.0)))
                .setReversed(true)
                .lineTo(Vector2d(50.98, -61.59))
                .setReversed(false)
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
//        clawRotateServo.position = clawRotateOut
//        slideVerticalMotor.targetPosition = 1100
//        slideVerticalMotor.power = 1.0

        drive.followTrajectorySequence(traj1)
        drive.updatePoseEstimate()

//        slideVerticalMotor.targetPosition = 800
//        slideVerticalMotor.power = -1.0
//        sleep(500)
//        clawServo.position = clawServoOpen
//        sleep(200)
//        slideVerticalMotor.targetPosition = 1100
//        slideVerticalMotor.power = 1.0
//        sleep(300)
//
//        slideVerticalMotor.targetPosition = 0
//        slideVerticalMotor.power = -1.0
//        clawRotateServo.position = clawRotateUpRight

        drive.followTrajectorySequence(traj2)
        drive.updatePoseEstimate()


        drive.followTrajectorySequence(traj3)
        drive.updatePoseEstimate()

        drive.followTrajectorySequence(traj4)
        drive.updatePoseEstimate()
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