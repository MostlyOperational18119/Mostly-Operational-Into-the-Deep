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
        outRotationServo!!.position = outRotationCenter
        outSwivelServo = hardwareMap.servo["OutSwivel"]
        inStopServo = hardwareMap.servo["InStop"]
        inStopServo!!.position = inStopOpen
        inRotationServo = hardwareMap.servo["InRotation"]
        inRotationServo!!.position = inRotationUp

        drive!!.poseEstimate = Pose2d(-8.7, -63.19, Math.toRadians(-90.00))

        val bar: TrajectorySequence =
            drive!!.trajectorySequenceBuilder( Pose2d(-8.7, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(-10.04, -33.0))
                .setReversed(false)
                .build()

        val sample1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-10.04, -33.0,Math.toRadians(-90.0)))
                .splineToLinearHeading(Pose2d(-50.3, -42.2,Math.toRadians(90.0)),Math.toRadians(90.0))
                .addTemporalMarker(0.5){inRotationServo!!.position = inRotationPick}
                .build()

        val basket1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-50.3, -42.2, Math.toRadians(90.00)))
                .setReversed(true)
                .addTemporalMarker(0.5){outRotationServo!!.position = outRotationBackOut}
                .lineToLinearHeading(Pose2d(-56.5, -56.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val move: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-56.5, -56.5, Math.toRadians(90.00)))
                .lineToLinearHeading(Pose2d(-60.0,-60.0,Math.toRadians(45.00)))
                .build()

        val sample2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
                .addTemporalMarker(1.0){outRotationServo!!.position = outRotationCenter}
                .addTemporalMarker(1.0){verticalSlideTo(0,0.5)}
                .lineToLinearHeading(Pose2d(-59.55, -42.2, Math.toRadians(90.00)))
                .build()

        val basket2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-59.55, -42.2, Math.toRadians(90.00)))
                .setReversed(true)
                .addTemporalMarker(0.5){outRotationServo!!.position = outRotationBackOut}
                .lineToLinearHeading(Pose2d(-56.5, -56.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val sample3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
                .addTemporalMarker(1.0){outRotationServo!!.position = outRotationCenter}
                .addTemporalMarker(1.0){verticalSlideTo(0,0.5)}
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
            drive!!.trajectorySequenceBuilder(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
                .addTemporalMarker(0.25){verticalSlideTo(0,1.0)}
                .addTemporalMarker(0.25){outRotationServo!!.position = outRotationCenter}
                .lineTo(Vector2d(-32.46, -12.0))
                .build()

        waitForStart()
        if (isStopRequested) {return}

        //START
        verticalSlideTo(1550, 1.0)
        horizontalSlideTo(100,1.0)
        outRotationServo!!.position = outRotationBack
        outSwivelServo!!.position = outSwivelPerpBack

        //BAR
        drive!!.followTrajectorySequence(bar)
        verticalSlideTo(750,1.0)
        sleep(300)
        outClawServo!!.position = outClawOpen
        outRotationServo!!.position = outRotationCenter

        //SAMPLE1
        drive!!.followTrajectorySequence(sample1)
        verticalSlideTo(0,0.5)
        inRotationServo!!.position = inRotationPick
        outSwivelServo!!.position = outSwivelPerpBack
        inStopServo!!.position = inStopClose
        sleep(100)
        horizontalSlideTo(500,1.0)
        intakeMotor!!.power = 0.7
        sleep(1400)
        inRotationServo!!.position = inRotationTransfer
        sleep(100)
        horizontalSlideTo(0,1.0)
        inStopServo!!.position = inStopAutoOpen
        sleep(700)
        outClawServo!!.position = outClawClose
        sleep(300)
        verticalSlideTo(verticalSlideHigh, 1.0)
        horizontalSlideTo(100,1.0)
        intakeMotor!!.power = 0.0


        //BASKET1
        drive!!.followTrajectorySequence(basket1)
        sleep(600)
        drive!!.followTrajectorySequence(move)
        sleep(300)
        inRotationServo!!.position = inRotationPick
        outClawServo!!.position = outClawOpen
        sleep(200)

        //SAMPLE2
        drive!!.followTrajectorySequence(sample2)
        outClawServo!!.position = outClawOpen
        inRotationServo!!.position = inRotationPick
        outSwivelServo!!.position = outSwivelPerpBack
        inStopServo!!.position = inStopClose
        sleep(100)
        horizontalSlideTo(500,1.0)
        intakeMotor!!.power = 0.7
        sleep(1400)
        inRotationServo!!.position = inRotationTransfer
        sleep(100)
        horizontalSlideTo(0,1.0)
        inStopServo!!.position = inStopAutoOpen
        sleep(1000)
        outClawServo!!.position = outClawClose
        sleep(300)
        intakeMotor!!.power = 0.0
        verticalSlideTo(verticalSlideHigh, 1.0)
        horizontalSlideTo(100,1.0)

        //BASKET2
        drive!!.followTrajectorySequence(basket2)
        sleep(600)
        drive!!.followTrajectorySequence(move)
        inRotationServo!!.position = inRotationPick
        sleep(300)
        outClawServo!!.position = outClawOpen
        sleep(100)

        //SAMPLE3
        drive!!.followTrajectorySequence(sample3)
        outClawServo!!.position = outClawOpen
        outSwivelServo!!.position = outSwivelPerpBack
        inRotationServo!!.position = inRotationPick
        inStopServo!!.position = inStopClose
        sleep(100)
        horizontalSlideTo(750,1.0)
        intakeMotor!!.power = 0.7
        sleep(1400)
        inRotationServo!!.position = inRotationTransfer
        sleep(100)
        horizontalSlideTo(0,1.0)
        inStopServo!!.position = inStopAutoOpen
        sleep(1000)
        outClawServo!!.position = outClawClose
        sleep(300)
        verticalSlideTo(verticalSlideHigh, 1.0)
        intakeMotor!!.power = 0.0
        horizontalSlideTo(100,1.0)

        //BASKET3
        drive!!.followTrajectorySequence(basket3)
        sleep(1000)
        drive!!.followTrajectorySequence(move)
        inRotationServo!!.position = inRotationPick
        sleep(300)
        outClawServo!!.position = outClawOpen
        sleep(100)

        //END
        drive!!.followTrajectorySequence(end)

        PoseStorage.currentPose = Pose2d(-44.0, -44.0, Math.toRadians(45.0))
    }
}