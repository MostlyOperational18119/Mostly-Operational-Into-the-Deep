package org.firstinspires.ftc.teamcode.Autonomous.NoSplineAuto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.Autonomous.PoseStorage.colorSide
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale


// Autonomous

@Autonomous(name = "RB_BACKUP", group = "B")
class RB_Backup : DriveMethods() {
    override fun runOpMode() {
        // Setup Odometry :)
        val drive = SampleMecanumDrive(hardwareMap)

        val rotateMotor = hardwareMap.get(DcMotor::class.java, "motorRotate")
        rotateMotor.targetPosition = 300
        rotateMotor.power = 0.5
        rotateMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        rotateMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val slideMotor = hardwareMap.get(DcMotor::class.java, "motorSlide")
        slideMotor.targetPosition = 0
        slideMotor.power = 0.0
        slideMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        slideMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val clawServo = hardwareMap.get(CRServo::class.java, "clawServo")
        val rotateServo = hardwareMap.get(Servo::class.java, "rotateServo")

        rotateServo.position =  0.11

        drive.poseEstimate = Pose2d(-34.09, -63.19, Math.toRadians(90.00))
        // Setup up the trajectory sequence (drive path)
        val traj1: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-34.09, -63.19, Math.toRadians(90.00)))
                .splineTo(Vector2d(-39.64, -47.68), Math.toRadians(144.20))
                .splineTo(Vector2d(-58.26, -57.0), Math.toRadians(225.00))
                .build()

        val traj2: TrajectorySequence?=
            drive.trajectorySequenceBuilder(Pose2d(-58.26,-57.0, Math.toRadians(225.0)))
                .setReversed(true)
                .splineTo(Vector2d(-35.91, -31.73), Math.toRadians(90.00))
                .splineTo(Vector2d(-45.21, -13.68), Math.toRadians(170.00))
                .lineToConstantHeading(Vector2d(-52.81, -60.98))
                .lineTo(Vector2d(-44.83, -15.96))
                .lineTo(Vector2d(-54.90, -15.96))
                .lineToConstantHeading(Vector2d(-60.03, -59.27))
                .lineTo(Vector2d(-53.38, -15.96))
                .splineTo(Vector2d(-62.69, -12.54), Math.toRadians(180.00))
                .lineToConstantHeading(Vector2d(-63.07, -54.71))
                .setReversed(false)
                .waitSeconds(0.25)
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
        drive.followTrajectorySequence(traj1)

        slideMotor.targetPosition = -2000
        slideMotor.power = -0.5
        rotateMotor.targetPosition = 75
        rotateMotor.power = 0.5
        sleep(2000)
        rotateMotor.targetPosition = 200
        rotateMotor.power = 0.5
        sleep(1000)
        clawServo.power = -1.0
        sleep(1000)
        clawServo.power = 0.0
        rotateMotor.targetPosition = 100
        rotateMotor.power = -0.5
        sleep(2000)
        slideMotor.targetPosition = 0
        slideMotor.power = 0.5
        drive.followTrajectorySequence(traj2)

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
        colorSide = "red"
    }
}