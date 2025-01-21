package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence


@Autonomous(name = "BAR4", group = "AAAA")
class BAR4 : Methods() {
    override fun runOpMode() {
        initOdometry()
        initMotors()

        transferServo = hardwareMap.servo["Transfer"]
        outClawServo = hardwareMap.servo["OutClaw"]
        outRotationServo = hardwareMap.servo["OutRotation"]
        outRotationServo!!.position = outRotationCenter
        outSwivelServo = hardwareMap.servo["OutSwivel"]
        inStopServo = hardwareMap.servo["InStop"]
        inRotationServo = hardwareMap.servo["InRotation"]

        drive!!.poseEstimate = Pose2d(14.74, -63.19, Math.toRadians(-90.00))

        val bar0: TrajectorySequence =
            drive!!.trajectorySequenceBuilder( Pose2d(14.74, -63.19, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(4.4,-32.0))
                .build()

        val sample1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(4.4, -32.0,Math.toRadians(-90.0)))
                .setReversed(true)
                .splineTo(Vector2d(49.02, -41.16), Math.toRadians(90.00))
                .setReversed(false)
                .build()

        val sample2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(49.02, -41.16, Math.toRadians(90.00)))
                .strafeRight(10.0)
                .build()

        val sample3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(59.02, -41.16, Math.toRadians(90.00)))
                .turn(70.0)
                .build()

        val pick1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(60.0,-57.06, Math.toRadians(70.0)))
                .splineToConstantHeading(Vector2d(46.25, -65.25), Math.toRadians(90.00))
                .build()


        val bar1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(46.25, -65.25, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(0.3, -32.0))
                .build()

        val pick2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(0.3, -32.0, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(46.25, -65.25))
                .build()

        val bar2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(46.25, -65.25, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(-3.5, -32.0))
                .build()

        val end: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-3.5, -32.0, Math.toRadians(90.00)))
                .lineToConstantHeading(Vector2d(43.25, -55.55))
                .build()

//        val push: TrajectorySequence =
//            drive!!.trajectorySequenceBuilder(Pose2d(55.85, -61.55, Math.toRadians(-90.00)))
//                .lineToConstantHeading(Vector2d(55.85, -13.0))
//                .lineToConstantHeading(Vector2d(70.0, -13.0))
//                .lineToConstantHeading(Vector2d(70.0, -61.55))
//                .build()

        waitForStart()

        //START
        verticalSlideTo(1550, 1.0)
        outRotationServo!!.position = outRotationBack
        outSwivelServo!!.position = outSwivelPerpBack

        //BAR0
        drive!!.followTrajectorySequence(bar0)
        sleep(100)
        placeSpecimen()
        sleep(300)
        verticalSlideTo(0,0.4)

        //SAMPLE1
        drive!!.followTrajectorySequence(sample1)
        horizontalSlideTo(500,1.0)

        //SAMPLE2 + PICK1
        drive!!.followTrajectorySequence(sample2)
        sleep(100)
        drive!!.followTrajectorySequence(pick1)
        sleep(300)
        outClawServo!!.position = outClawClose
        sleep(300)
        verticalSlideTo(1550, 1.0)
        sleep(300)
        outRotationServo!!.position = outRotationBack
        outSwivelServo!!.position = outSwivelPerpBack

        //BAR1
        drive!!.followTrajectorySequence(bar1)
        sleep(100)
        placeSpecimen()
        sleep(300)
        verticalSlideTo(0,0.4)
        outRotationServo!!.position = outRotationFront
        outSwivelServo!!.position = outSwivelPerpFront

        //PICK2
        drive!!.followTrajectorySequence(pick2)
        sleep(300)
        outClawServo!!.position = outClawClose
        sleep(300)
        verticalSlideTo(1550, 1.0)
        sleep(300)
        outRotationServo!!.position = outRotationBack
        outSwivelServo!!.position = outSwivelPerpBack

        //BAR2
        drive!!.followTrajectorySequence(bar2)
        sleep(100)
        placeSpecimen()

        //END
        outRotationServo!!.position = outRotationCenter
        verticalSlideTo(0, 0.5)
        drive!!.followTrajectorySequence(end)

        while (opModeIsActive() && !isStopRequested) { drive!!.update() }
        PoseStorage.currentPose = drive!!.poseEstimate
    }
}