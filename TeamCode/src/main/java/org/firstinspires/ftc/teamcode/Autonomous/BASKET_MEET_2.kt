package org.firstinspires.ftc.teamcode.Autonomous.NoSplineAuto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Autonomous.poseStorage
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale


// Autonomous

@Autonomous(name = "BASKET_Meet2", group = "B")
class BASKET_MEET_2 : DriveMethods() {
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




        drive.poseEstimate = Pose2d(-34.09, -63.19, Math.toRadians(-90.00))
        // Setup up the trajectory sequence (drive path)

        //STILL NEED TO CONNECT EVERYTHING TOGETHER
        val traj1: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-34.09, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(-10.04, -34.01))
                .build()

        val traj2: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-4.4,-34.09, Math.toRadians(-90.0)))
                .setReversed(false)
                .splineTo(Vector2d(-47.87, -39.89), Math.toRadians(90.00))
                .build()

        val traj3: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-4.4,-34.09, Math.toRadians(-90.0)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-55.0, -55.0, Math.toRadians(45.00)))
                .build()

        val traj4: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-49.40, -42.89, Math.toRadians(-90.00)))
                .back(2.0)
                .build()

        val traj5: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-57.26, -57.26, Math.toRadians(225.00)))
                .setReversed(false)
                .lineToLinearHeading(Pose2d(-57.75, -38.37, Math.toRadians(88.83)))
                .build()


        val traj6: TrajectorySequence?=
            drive.trajectorySequenceBuilder(Pose2d(-59.55,-43.09, Math.toRadians(-90.0)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-55.0, -55.0, Math.toRadians(45.00)))
                .build()

        val traj7: TrajectorySequence?=
            drive.trajectorySequenceBuilder(Pose2d(-59.55,-43.09, Math.toRadians(-90.0)))
                .back(2.0)
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


        drive.followTrajectorySequence(traj1)

        slideMotor.targetPosition = -2000
        slideMotor.power = 1.0
        rotateMotor.targetPosition = 75
        rotateMotor.power = 1.0
        sleep(1000)
        rotateMotor.targetPosition = 200
        rotateMotor.power = 1.0
        sleep(1500)
        clawServo.power = -1.0
        sleep(500)
        clawServo.power = 0.0
        rotateMotor.targetPosition = 100
        rotateMotor.power = -1.0
        sleep(600)
        slideMotor.targetPosition = 0
        slideMotor.power = 1.0
        sleep(400)
        rotateMotor.targetPosition = 1100
        rotateMotor.power = 0.75

        drive.followTrajectorySequence(traj2)
        sleep(300)
        slideMotor.targetPosition = -500
        slideMotor.power = 1.0
        rotateServo!!.position = 0.25
        sleep(300)

        drive.followTrajectorySequence(traj3)
        sleep(200)
        clawServo.power = 1.0
        rotateMotor.targetPosition = 1350
        rotateMotor.power = 0.5
        sleep(1000)
        clawServo.power = 0.0
        sleep(200)
        rotateMotor.targetPosition = 75
        rotateMotor.power = 0.75
        rotateServo.position = 0.11
        sleep(300)


        drive.followTrajectorySequence(traj4)
        slideMotor.targetPosition = -2000
        slideMotor.power = 1.0
        sleep(1000)
        rotateMotor.targetPosition = 200
        rotateMotor.power = 1.0
        sleep(2000)
        clawServo.power = -1.0
        sleep(1500)
        clawServo.power = 0.0
        rotateMotor.targetPosition = 100
        rotateMotor.power = -1.0
        sleep(600)
        slideMotor.targetPosition = 0
        slideMotor.power = 1.0
        sleep(400)
        rotateMotor.targetPosition = 1100
        rotateMotor.power = 0.75

        drive.followTrajectorySequence(traj5)
        sleep(10000)




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
    }
}