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

        val bar0: TrajectorySequence =
            drive!!.trajectorySequenceBuilder( Pose2d(14.74, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(5.94, -42.15))
                .setReversed(false)
                .build()

        val sample1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(5.94, -42.15,Math.toRadians(-90.0)))
                .splineToConstantHeading(Vector2d(35.43, -31.79), Math.toRadians(90.00))
                .setReversed(true)
                .splineToConstantHeading(Vector2d(45.19, -12.83), Math.toRadians(0.00))
                .setReversed(false)
                .build()

        val sample2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(45.19, -12.83, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(50.17, -62.04))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(42.89, -28.72))
                .splineToConstantHeading(Vector2d(56.11, -13.21), Math.toRadians(0.00))
                .setReversed(false)
                .lineToConstantHeading(Vector2d(61.85, -57.06))
                .setReversed(true)
                .splineToConstantHeading(Vector2d(47.5, -59.0), Math.toRadians(-90.00))
                .setReversed(false)
                .build()

        val bar1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(47.49, -59.0, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(5.94, -33.75))
                .build()

        val pick2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(5.94, -33.75, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(47.5, -59.0))
                .build()

        val bar2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(47.5, -59.0, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(11.40, -34.20))
                .build()

        val end: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(11.4, -34.2, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(55.85, -61.55))
                .build()

        waitForStart()

        //START
        verticalSlideTo(1600, 1.0)
        outRotationServo!!.position = outRotationBack

        //BAR0
        drive!!.followTrajectorySequence(bar0)
        placeSpecimen()
        verticalSlideTo(1600, 1.0)
        sleep(300)
        outRotationServo!!.position = outRotationFront

        //SAMPLE1
        drive!!.followTrajectorySequence(sample1)

        //SAMPLE2 + PICK1
        verticalSlideTo(500,1.0)
        outClawServo!!.position = outClawOpen
        drive!!.followTrajectorySequence(sample2)
        sleep(300)
        outClawServo!!.position = outClawClose
        verticalSlideTo(1600, 1.0)
        sleep(300)
        outClawServo!!.position = outRotationBack

        //BAR1
        drive!!.followTrajectorySequence(bar1)
        placeSpecimen()
        verticalSlideTo(1600, 1.0)
        sleep(300)
        outRotationServo!!.position = outRotationFront

        //PICK2
        verticalSlideTo(500,1.0)
        outClawServo!!.position = outClawOpen
        drive!!.followTrajectorySequence(pick2)
        sleep(300)
        outClawServo!!.position = outClawClose
        verticalSlideTo(1600, 1.0)
        sleep(300)
        outClawServo!!.position = outRotationBack

        //BAR2
        drive!!.followTrajectorySequence(bar2)
        placeSpecimen()
        verticalSlideTo(1600, 1.0)
        sleep(300)
        outRotationServo!!.position = outRotationCenter
        sleep(300)

        //END
        verticalSlideTo(0, 1.0)
        drive!!.followTrajectorySequence(end)

        while (opModeIsActive() && !isStopRequested) { drive!!.update() }
        PoseStorage.currentPose = drive!!.poseEstimate
    }
}