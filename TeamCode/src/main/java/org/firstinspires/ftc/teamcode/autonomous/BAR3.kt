package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence


@Autonomous(name = "BAR3", group = "AAAA")
class BAR3 : Methods() {
    override fun runOpMode() {
        initOdometry()
        initMotors()

        transferServo = hardwareMap.servo["Transfer"]
        transferServo!!.position = transferServoNormal
        outClawServo = hardwareMap.servo["OutClaw"]
        outClawServo!!.position = outClawClose
        outRotationServo = hardwareMap.servo["OutRotation"]
        outSwivelServo = hardwareMap.servo["OutSwivel"]
        outSwivelServo!!.position = outSwivelPerpFront
        inSwivelServo = hardwareMap.servo["InSwivel"]
        inSwivelServo!!.position = inSwivelCenter
        inRotationServo = hardwareMap.servo["InRotation"]
        inClawServo = hardwareMap.servo["InClaw"]

        drive!!.poseEstimate = Pose2d(14.74, -63.19, Math.toRadians(-90.00))

        val bar0: TrajectorySequence =
            drive!!.trajectorySequenceBuilder( Pose2d(14.74, -63.19, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(5.94, -33.0))
                .build()

        val sample1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(5.94, -33.0,Math.toRadians(-90.0)))
                .splineToConstantHeading(Vector2d(35.43, -31.79), Math.toRadians(-90.00))
                .setReversed(true)
                .splineToConstantHeading(Vector2d(45.19, -12.83), Math.toRadians(-90.00))
                .setReversed(false)
                .build()

        val sample2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(45.19, -12.83, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(50.17, -62.04))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(42.89, -28.72))
                .splineToConstantHeading(Vector2d(56.11, -13.21), Math.toRadians(-90.00))
                .setReversed(false)
                .lineToConstantHeading(Vector2d(61.85, -57.06))
                .setReversed(true)
                .splineToConstantHeading(Vector2d(47.5, -63.0), Math.toRadians(-90.00))
                .setReversed(false)
                .build()

        val bar1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(47.49, -63.0, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(5.94, -33.0))
                .build()

        val pick2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(5.94, -33.0, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(47.5, -63.0))
                .build()

        val bar2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(47.5, -63.0, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(11.40, -33.00))
                .build()

        val end: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(11.4, -33.0, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(55.85, -61.55))
                .build()

        waitForStart()

        //START
        verticalSlideTo(1700, 1.0)
        sleep(300)
        outRotationServo!!.position = outRotationBack
        sleep(300)

        //BAR0
        drive!!.followTrajectorySequence(bar0)
        placeSpecimen()
        verticalSlideTo(1700, 1.0)
        sleep(300)
        outRotationServo!!.position = outRotationFront
        outSwivelServo!!.position = outSwivelPerpFront

        //SAMPLE1
        drive!!.followTrajectorySequence(sample1)

        //SAMPLE2 + PICK1
        verticalSlideTo(500,1.0)
        outClawServo!!.position = outClawOpen
        drive!!.followTrajectorySequence(sample2)
        sleep(300)
        outClawServo!!.position = outClawClose
        sleep(500)
        verticalSlideTo(1700, 1.0)
        sleep(300)
        outClawServo!!.position = outRotationBack
        outSwivelServo!!.position = outSwivelPerpBack

        //BAR1
        drive!!.followTrajectorySequence(bar1)
        placeSpecimen()
        verticalSlideTo(1700, 1.0)
        sleep(300)
        outRotationServo!!.position = outRotationFront
        outSwivelServo!!.position = outSwivelPerpFront

        //PICK2
        verticalSlideTo(500,1.0)
        outClawServo!!.position = outClawOpen
        drive!!.followTrajectorySequence(pick2)
        sleep(300)
        outClawServo!!.position = outClawClose
        sleep(500)
        verticalSlideTo(1700, 1.0)
        sleep(300)
        outClawServo!!.position = outRotationBack
        outSwivelServo!!.position = outSwivelPerpBack

        //BAR2
        drive!!.followTrajectorySequence(bar2)
        placeSpecimen()
        verticalSlideTo(1700, 1.0)
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