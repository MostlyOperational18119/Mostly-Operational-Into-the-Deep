package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence


@Autonomous(name = "BASKET_Meet2", group = "AAAA")
class BAR3 : Methods() {
    override fun runOpMode() {
        initOdometry()
        initMotors()
        initServosAndTouchWithoutSet()

        drive!!.poseEstimate = Pose2d(14.74, -63.19, Math.toRadians(-90.00))

        val bar0: TrajectorySequence? =
            drive!!.trajectorySequenceBuilder( Pose2d(14.74, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(5.94, -42.15))
                .setReversed(false)
                .build()

        val sample1: TrajectorySequence? =
            drive!!.trajectorySequenceBuilder(Pose2d(5.94, -42.15,Math.toRadians(-90.0)))
                .splineToConstantHeading(Vector2d(35.43, -31.79), Math.toRadians(90.00))
                .setReversed(true)
                .splineToConstantHeading(Vector2d(45.19, -12.83), Math.toRadians(0.00))
                .setReversed(false)
                .build()

        val sample2: TrajectorySequence? =
            drive!!.trajectorySequenceBuilder(Pose2d(45.19, -12.83, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(50.17, -62.04))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(42.89, -28.72))
                .splineToConstantHeading(Vector2d(56.11, -13.21), Math.toRadians(0.00))
                .setReversed(false)
                .lineToConstantHeading(Vector2d(61.85, -57.06))
                .build()

        val pick1: TrajectorySequence? =
            drive!!.trajectorySequenceBuilder(Pose2d(61.85, -57.06, Math.toRadians(90.00)))
                .setReversed(true)
                .splineToConstantHeading(Vector2d(47.5, -62.62), Math.toRadians(-90.00))
                .setReversed(false)
                .build()

        val bar1: TrajectorySequence? =
            drive!!.trajectorySequenceBuilder(Pose2d(47.49, -62.62, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(5.94, -33.75))
                .build()

        val pick2: TrajectorySequence? =
            drive!!.trajectorySequenceBuilder(Pose2d(5.94, -33.75, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(47.5, -62.62))
                .build()

        val bar2: TrajectorySequence? =
            drive!!.trajectorySequenceBuilder(Pose2d(47.5, -62.62, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(11.40, -34.20))
                .build()

        val end: TrajectorySequence? =
            drive!!.trajectorySequenceBuilder(Pose2d(11.4, -34.2, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(55.85, -61.55))
                .build()

        waitForStart()

        //START
//        verticalSlideTo(1600, 1.0)
//        clawRotateServo!!.position = clawRotateStraight
//
//        //BAR
//        drive!!.followTrajectorySequence(bar)
//        placeSpecimen()
//
//        //SAMPLE1
//        drive!!.followTrajectorySequence(sample1)
//        intakePixel(3500)
//        transferFromDownToHigh()
//
//        //BASKET1
//        drive!!.followTrajectorySequence(basket1)
//        placeSample()
//        verticalSlideTo(0, 0.5)
//
//        //SAMPLE2
//        drive!!.followTrajectorySequence(sample2)
//        intakePixel(3500)
//        transferFromDownToHigh()
//        verticalSlideTo(3650, 1.0)
//
//        //BASKET2
//        drive!!.followTrajectorySequence(basket2)
//        placeSample()
//
//        //END
//        drive!!.followTrajectorySequence(end)
//        verticalSlideTo(0, 1.0)
        //clawRotateServo!!.position = clawRotateUpRight

        while (opModeIsActive() && !isStopRequested) { drive!!.update() }
        PoseStorage.currentPose = drive!!.poseEstimate
    }
}