package org.firstinspires.ftc.teamcode.Autonomous.NoSplineAuto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Autonomous.poseStorage
import org.firstinspires.ftc.teamcode.Autonomous.poseStorage.colorSide
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale


// Autonomous


@Autonomous(name = "BB_FIRST", group = "Linear Opmode")
class BB_First : DriveMethods() {
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
        drive.poseEstimate = Pose2d(33.82, 62.88, Math.toRadians(-90.00))
        // Setup up the trajectory sequence (drive path)
        val traj1: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(33.82, 62.88, Math.toRadians(-90.00)))
                .splineTo(Vector2d(36.66, 47.87), Math.toRadians(-36.37))
                .splineTo(Vector2d(56.99, 56.5), Math.toRadians(45.00))
                .build()

        // SLIDES


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
        rotateMotor.targetPosition = 150
        rotateMotor.power = 0.5
        sleep(2000)
        rotateMotor.targetPosition = 300
        rotateMotor.power = 0.5
        sleep(1000)
        clawServo.power = -1.0
        sleep(1000)
        clawServo.power = 0.0
        rotateMotor.targetPosition = 150
        rotateMotor.power = -0.5
        sleep(5000)
        slideMotor.targetPosition = 0
        slideMotor.power = 0.5
        sleep(5000)
        rotateMotor.targetPosition = 200
        rotateMotor.power = 0.5


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
        poseStorage.currentPose = drive.poseEstimate
        colorSide = "blue"
    }
}