package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence


@Autonomous(name = "BAR4Turn", group = "AAAA")
class BAR4Turn : Methods() {
    override fun runOpMode() {
        initOdometry()
        initMotors()

        transferServo = hardwareMap.servo["Transfer"]
        transferServo!!.position = transferServoOpen
        outClawServo = hardwareMap.servo["OutClaw"]
        outClawServo!!.position = outClawClose
        outRotationServo = hardwareMap.servo["OutRotation"]
        outRotationServo!!.position = outRotationCenter
        outSwivelServo = hardwareMap.servo["OutSwivel"]
        inStopServo = hardwareMap.servo["InStop"]
        inStopServo!!.position = inStopClose
        inRotationServo = hardwareMap.servo["InRotation"]
        inRotationServo!!.position = 1.0

        drive!!.poseEstimate = Pose2d(14.74, -63.19, Math.toRadians(-90.00))

        val bar0: TrajectorySequence =
            drive!!.trajectorySequenceBuilder( Pose2d(14.74, -63.19, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(4.4,-32.0))
                .build()

        val sample1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(4.4, -32.0,Math.toRadians(-90.0)))
                .splineTo(Vector2d(46.25, -48.16), Math.toRadians(90.00))
                .build()

        val sample1a: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(46.25, -48.16,Math.toRadians(90.0)))
                .turn(-Math.toRadians(140.0))
                .build()

        val sample2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(46.75, -48.16,Math.toRadians(-50.0)))
                .turn(Math.toRadians(120.0))
                .build()

        val sample2a: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(46.75, -48.16,Math.toRadians(70.0)))
                .turn(-Math.toRadians(120.0))
                .build()

        val sample3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(46.75, -48.16,Math.toRadians(-50.0)))
                .turn(Math.toRadians(100.0))
                .build()

        val sample3a: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(46.75, -48.16,Math.toRadians(50.0)))
                .turn(-Math.toRadians(100.0))
                .build()

        val pick1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(56.5,-50.16, Math.toRadians(-50.0)))
                .splineTo(Vector2d(46.25, -65.25), Math.toRadians(-90.00))
                .build()


        val bar1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(46.25, -65.25, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(0.3, -32.0))
                .build()

        val pick2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(0.3, -32.0, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(46.25, -65.25))
                .build()

        val bar2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(46.25, -65.25, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(-3.5, -32.0))
                .build()

        val pick3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-3.5, -32.0, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(46.25, -65.25))
                .build()

        val bar3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(46.25, -65.25, Math.toRadians(-90.00)))
                .lineToConstantHeading(Vector2d(-5.0, -32.0))
                .build()

        val end: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-5.0, -32.0, Math.toRadians(-90.00)))
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
        placeSpecimen()
        sleep(300)
        verticalSlideTo(200,0.4)
        horizontalSlideTo(100,1.0)
        outRotationServo!!.position = outRotationFront
        outSwivelServo!!.position = outSwivelPerpFront
        inRotationServo!!.position = inRotationPick

        //SAMPLE1
        drive!!.followTrajectorySequence(sample1)
        horizontalSlideTo(800,1.0)
        sleep(100)
        intakeMotor!!.power = 0.7
        sleep(1000)
        horizontalSlideTo(300, 1.0)
        drive!!.followTrajectorySequence(sample1a)
        sleep(100)
        intakeMotor!!.power = -0.7

        //SAMPLE2
        drive!!.followTrajectorySequence(sample2)
        horizontalSlideTo(900,1.0)
        sleep(100)
        intakeMotor!!.power = 0.7
        sleep(1000)
        horizontalSlideTo(300, 1.0)
        drive!!.followTrajectorySequence(sample2a)
        intakeMotor!!.power = -0.7

        //SAMPLE3
        drive!!.followTrajectorySequence(sample3)
        horizontalSlideTo(1100,1.0)
        sleep(100)
        intakeMotor!!.power = 0.7
        sleep(1000)
        horizontalSlideTo(300, 1.0)
        drive!!.followTrajectorySequence(sample3a)
        intakeMotor!!.power = -0.7
        horizontalSlideTo(0,1.0)
        intakeMotor!!.power = 0.0
        inRotationServo!!.position = 1.0

        //PICK1
        verticalSlideTo(0,0.75)
        sleep(200)
        drive!!.followTrajectorySequence(pick1)
        sleep(300)
        outClawServo!!.position = outClawClose
        sleep(300)
        verticalSlideTo(1550, 1.0)
        sleep(300)
        outRotationServo!!.position = outRotationFront
        outSwivelServo!!.position = outSwivelPerpFront

        //BAR1
        drive!!.followTrajectorySequence(bar1)
        sleep(100)
        placeSpecimen()
        sleep(300)
        verticalSlideTo(0,0.4)
        outRotationServo!!.position = outRotationBack
        outSwivelServo!!.position = outSwivelPerpBack

        //PICK2
        drive!!.followTrajectorySequence(pick2)
        sleep(300)
        outClawServo!!.position = outClawClose
        sleep(300)
        verticalSlideTo(1550, 1.0)
        sleep(300)
        outRotationServo!!.position = outRotationFront
        outSwivelServo!!.position = outSwivelPerpFront

        //BAR2
        drive!!.followTrajectorySequence(bar2)
        sleep(100)
        placeSpecimen()
        sleep(300)
        verticalSlideTo(0,0.4)
        outRotationServo!!.position = outRotationBack
        outSwivelServo!!.position = outSwivelPerpBack

        //PICK3
        drive!!.followTrajectorySequence(pick3)
        sleep(300)
        outClawServo!!.position = outClawClose
        sleep(300)
        verticalSlideTo(1550, 1.0)
        sleep(300)
        outRotationServo!!.position = outRotationFront
        outSwivelServo!!.position = outSwivelPerpFront

        //BAR3
        drive!!.followTrajectorySequence(bar3)
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