package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

@Autonomous(name = "basket_meet3", group = "AAAA")
class BASKET3 : Methods() {
    override fun runOpMode() {
        initOdometry()
        initMotors()
        initServosAndTouchWithoutSet()

        drive!!.poseEstimate = Pose2d(-34.09, -63.19, Math.toRadians(-90.00))

        val bar: TrajectorySequence =
            drive!!.trajectorySequenceBuilder( Pose2d(-34.09, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(-10.04, -33.0))
                .setReversed(false)
                .build()

        val sample1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-10.04, -42.4,Math.toRadians(-90.0)))
                .splineToLinearHeading(Pose2d(-48.25, -32.5, Math.toRadians(90.0)), Math.toRadians(90.00))
                .build()

        val basket1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-48.25, -40.5, Math.toRadians(90.00)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val sample2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .lineToLinearHeading(Pose2d(-57.0, -32.5, Math.toRadians(90.00)))
                .build()

        val basket2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-57.0, -40.5, Math.toRadians(90.00)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val sample3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .lineToLinearHeading(Pose2d(-58.0, -32.5, Math.toRadians(122.00)))
                .build()

        val basket3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-62.0, -40.5, Math.toRadians(122.00)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val end: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .lineTo(Vector2d(-44.0, -44.0))
                .build()

        waitForStart()

        //START
        verticalSlideTo(1600, 1.0)
        outRotationServo!!.position = outRotationBack

        //BAR
        drive!!.followTrajectorySequence(bar)
        placeSpecimen()

        //SAMPLE1
        drive!!.followTrajectorySequence(sample1)

        transferFromDownToHigh()

        //BASKET1
        drive!!.followTrajectorySequence(basket1)
        placeSample()
        verticalSlideTo(0, 0.5)

        //SAMPLE2
        drive!!.followTrajectorySequence(sample2)
        intakeSample()
        transferFromDownToHigh()

        //BASKET2
        drive!!.followTrajectorySequence(basket2)
        placeSample()
        verticalSlideTo(0, 0.5)

        //SAMPLE3
        drive!!.followTrajectorySequence(sample3)
        intakeSample()
        transferFromDownToHigh()

        //BASKET3
        drive!!.followTrajectorySequence(basket3)
        placeSample()

        //END
        drive!!.followTrajectorySequence(end)
        verticalSlideTo(0, 1.0)
        //clawRotateServo!!.position = clawRotateUpRight

        while (opModeIsActive() && !isStopRequested) { drive!!.update() }
        PoseStorage.currentPose = drive!!.poseEstimate
    }
}