/*package org.firstinspires.ftc.teamcode.autonomous.OldAutonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.autonomous.drive.SampleMecanumDrive
import java.util.Locale


// Autonomous


@Autonomous(name = "RP_FIRST", group = "Linear Opmode")
class RP_First : DriveMethods() {
    override fun runOpMode() {
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
        // Setup Odometry :)

        val drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = Pose2d(9.0, -63.19, Math.toRadians(90.00))
        // Setup up the trajectory sequence (drive path)


        val traj1 =
            drive.trajectorySequenceBuilder(Pose2d(13.28, -61.23, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(11.90) {}
                .splineTo(Vector2d(31.13, -35.11), Math.toRadians(0.00))
                .splineTo(Vector2d(49.87, -8.85), Math.toRadians(1.91))
                .lineTo(Vector2d(54.30, -8.70))
                .lineTo(Vector2d(50.61, -59.75))
                .lineTo(Vector2d(51.05, -11.95))
                .lineTo(Vector2d(62.41, -9.74))
                .lineTo(Vector2d(61.52, -58.87))
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
}*/