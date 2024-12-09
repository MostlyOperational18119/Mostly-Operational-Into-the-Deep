package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale

@Autonomous(name = "BASKET_Meet2", group = "AAAA")
class BASKET_MEET_2 : Methods() {
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)

        initMotors()
        initServosAndTouch()

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


        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        telemetry.addLine("started")
        telemetry.update()
        if (isStopRequested) return
        telemetry.addLine("not stopped")
        telemetry.update()

        //HIGH BAR
        verticalSlideTo(1600, 1.0)
        clawRotateServo!!.position = clawRotateStraight

        drive.followTrajectorySequence(traj1)

        transferServo!!.position = transferDownPos
        clawServo!!.position = clawServoClosed
        verticalSlideTo(500, 1.0)
        sleep(700)
        clawServo!!.position = clawServoOpen
        sleep(300)
        clawRotateServo!!.position = clawRotateUpRight
        intakeServo!!.power = 1.0

        //FIRST SAMPLE
        drive.followTrajectorySequence(traj2)

        intakePixel(3500)
        transferSlowDown()
        verticalSlideTo(3650, 1.0)

        //HIGH BASKET #1
        drive.followTrajectorySequence(traj3)

        placeSample()
        verticalSlideTo(0, 0.5)

        //SECOND SAMPLE
        drive.followTrajectorySequence(traj4)

        intakePixel(3500)
        transferSlowDown()
        verticalSlideTo(3650, 1.0)

        //MOVE TO HIGH BASKET AND PLACE SECOND SAMPLE
        drive.followTrajectorySequence(traj5)

        placeSample()
        verticalSlideTo(0, 0.5)

        drive.followTrajectorySequence(traj8)

        sleep(200)
        clawRotateServo!!.position = clawRotateUpRight
        sleep(500)

        verticalSlideTo(0, 1.0)

        drive.followTrajectorySequence(traj9)

        while (opModeIsActive() && !isStopRequested) { drive.update() }

        telemetry.addData("Odometry: ",
            String.format(
                "Pose: %s, Velocity: %s",
                drive.poseEstimate.toString(),
                drive.getWheelVelocities().toString()
            )
        )
        telemetry.update()
        PoseStorage.currentPose = drive.poseEstimate
    }
}