package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence

@Autonomous(name = "BASKET3+1", group = "AAAA")
class BASKET3AND1 : Methods() {
    override fun runOpMode() {
        initOdometry()
        initMotors()
        transferServo = hardwareMap.servo["Transfer"]
        transferServo!!.position = transferServoClose
        outClawServo = hardwareMap.servo["OutClaw"]
        outClawServo!!.position = outClawClose
        outRotationServo = hardwareMap.servo["OutRotation"]
        outSwivelServo = hardwareMap.servo["OutSwivel"]
        inStopServo = hardwareMap.servo["InStop"]
        inStopServo!!.position = inStopOpen
        inRotationServo = hardwareMap.servo["InRotation"]
        inRotationServo!!.position = inRotationUp

        drive!!.poseEstimate = Pose2d(-8.7, -63.19, Math.toRadians(-90.00))

        val bar: TrajectorySequence =
            drive!!.trajectorySequenceBuilder( Pose2d(-8.7, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(-10.04, -32.4))
                .setReversed(false)
                .build()

        val sample1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-10.04, -33.0,Math.toRadians(-90.0)))
                .splineToLinearHeading(Pose2d(-50.5, -42.6,Math.toRadians(90.0)),Math.toRadians(90.0))
                .addTemporalMarker(0.5){inRotationServo!!.position = inRotationPick}
                .build()

        val basket1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-50.5, -42.6, Math.toRadians(90.00)))
                .setReversed(true)
                .addTemporalMarker(0.5){outRotationServo!!.position = outRotationBackOut}
                .lineToLinearHeading(Pose2d(-56.5, -56.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val move: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-56.5, -56.5, Math.toRadians(90.00)))
                .lineToLinearHeading(Pose2d(-63.5,-59.5,Math.toRadians(45.00)))
                .build()

        val sample2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-63.5, -59.5, Math.toRadians(45.00)))
                .addTemporalMarker(1.0){outRotationServo!!.position = outRotationCenter}
                .addTemporalMarker(0.5){verticalSlideTo(-10,1.0)}
                .lineToLinearHeading(Pose2d(-60.0, -43.3, Math.toRadians(90.00)))
                .build()

        val basket2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-60.0, -43.3, Math.toRadians(90.00)))
                .setReversed(true)
                .addTemporalMarker(0.5){outRotationServo!!.position = outRotationBackOut}
                .lineToLinearHeading(Pose2d(-56.5, -56.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val sample3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-63.5, -59.5, Math.toRadians(45.00)))
                .addTemporalMarker(1.0){outRotationServo!!.position = outRotationCenter}
                .addTemporalMarker(0.5){verticalSlideTo(-10,1.0)}
                .lineToLinearHeading(Pose2d(-59.5, -46.5, Math.toRadians(118.00)))
                .build()

        val basket3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-59.5, -46.5, Math.toRadians(118.00)))
                .setReversed(true)
                .addTemporalMarker(0.5) { outRotationServo!!.position = outRotationBackOut }
                .lineToLinearHeading(Pose2d(-56.5, -56.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val end: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-63.5, -59.6, Math.toRadians(45.00)))
                .addTemporalMarker(0.25){verticalSlideTo(0,1.0)}
                .addTemporalMarker(0.25){outRotationServo!!.position = outRotationCenter}
                .lineTo(Vector2d(-36.0, -10.0))
                .build()

        waitForStart()
        if (isStopRequested) {return}

        //START
        verticalSlideTo(verticalSlideBar, 1.0)
        outRotationServo!!.position = outRotationBack
        outSwivelServo!!.position = outSwivelPerpBack
        sleep(100)

        //BAR
        drive!!.followTrajectorySequence(bar)
        verticalSlideTo(900,1.0)
        sleep(300)
        outClawServo!!.position = outClawOpen
        sleep(300)
        outRotationServo!!.position = outRotationCenter

        //SAMPLE1
        drive!!.followTrajectorySequence(sample1)
        verticalSlideTo(-10,0.5)
        inRotationServo!!.position = inRotationPick
        outSwivelServo!!.position = outSwivelPerpBack
        inStopServo!!.position = inStopClose
        sleep(150)
        horizontalSlideTo(600,1.0)
        intakeMotor!!.power = 0.7
        sleep(800)
        inRotationServo!!.position = inRotationTransfer
        sleep(100)
        horizontalSlideTo(0,1.0)
        inStopServo!!.position = inStopAutoOpen
        sleep(1150)
        outClawServo!!.position = outClawClose
        sleep(300)
        verticalSlideTo(verticalSlideHigh, 1.0)
        intakeMotor!!.power = 0.0


        //BASKET1
        drive!!.followTrajectorySequence(basket1)
        transferServo!!.position = transferServoOpen
        sleep(700)
        drive!!.followTrajectorySequence(move)
        sleep(300)
        horizontalSlideTo(0,1.0)
        inRotationServo!!.position = inRotationPick
        outClawServo!!.position = outClawOpen
        sleep(200)

        //SAMPLE2
        drive!!.followTrajectorySequence(sample2)
        transferServo!!.position = transferServoClose
        outClawServo!!.position = outClawOpen
        inRotationServo!!.position = inRotationPick
        outSwivelServo!!.position = outSwivelPerpBack
        inStopServo!!.position = inStopClose
        sleep(100)
        horizontalSlideTo(600,1.0)
        intakeMotor!!.power = 0.7
        sleep(800)
        inRotationServo!!.position = inRotationTransfer
        sleep(100)
        horizontalSlideTo(0,1.0)
        inStopServo!!.position = inStopAutoOpen
        sleep(1150)
        outClawServo!!.position = outClawClose
        sleep(200)
        intakeMotor!!.power = 0.0
        verticalSlideTo(verticalSlideHigh, 1.0)

        //BASKET2
        drive!!.followTrajectorySequence(basket2)
        transferServo!!.position = transferServoOpen
        sleep(700)
        drive!!.followTrajectorySequence(move)
        horizontalSlideTo(0,1.0)
        inRotationServo!!.position = inRotationPick
        sleep(300)
        outClawServo!!.position = outClawOpen
        sleep(100)

        //SAMPLE3
        drive!!.followTrajectorySequence(sample3)
        transferServo!!.position = transferServoClose
        outClawServo!!.position = outClawOpen
        outSwivelServo!!.position = outSwivelPerpBack
        inRotationServo!!.position = inRotationPick
        inStopServo!!.position = inStopClose
        sleep(100)
        horizontalSlideTo(750,1.0)
        intakeMotor!!.power = 0.7
        sleep(1000)
        inRotationServo!!.position = inRotationTransfer
        sleep(100)
        horizontalSlideTo(0,1.0)
        inStopServo!!.position = inStopAutoOpen
        sleep(1300)
        outClawServo!!.position = outClawClose
        sleep(300)
        verticalSlideTo(verticalSlideHigh, 1.0)
        intakeMotor!!.power = 0.0

        //BASKET3
        drive!!.followTrajectorySequence(basket3)
        sleep(700)
        drive!!.followTrajectorySequence(move)
        inRotationServo!!.position = inRotationPick
        sleep(300)
        outClawServo!!.position = outClawOpen
        sleep(100)

        //END
        drive!!.followTrajectorySequence(end)
    }
}