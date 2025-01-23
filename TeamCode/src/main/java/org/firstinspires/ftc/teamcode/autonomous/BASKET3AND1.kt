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
        inStopServo!!.position = inStopClose
        inRotationServo = hardwareMap.servo["InRotation"]
        inRotationServo!!.position = 1.0

        drive!!.poseEstimate = Pose2d(-34.09, -63.19, Math.toRadians(-90.00))

        val bar: TrajectorySequence =
            drive!!.trajectorySequenceBuilder( Pose2d(-34.09, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(-10.04, -33.0))
                .setReversed(false)
                .build()

        val sample1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-10.04, -33.0,Math.toRadians(-90.0)))
                .splineToLinearHeading(Pose2d(-48.25, -32.5,Math.toRadians(90.0)),Math.toRadians(90.0))
                .build()

        val basket1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-48.25, -32.5, Math.toRadians(90.00)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val sample2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .lineToLinearHeading(Pose2d(-57.0, -32.5, Math.toRadians(90.00)))
                .addTemporalMarker(1.0){outRotationServo!!.position = outRotationCenter}
                .addTemporalMarker(1.0){verticalSlideTo(0,0.5)}
                .build()

        val basket2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-57.0, -32.5, Math.toRadians(90.00)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val sample3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .lineToLinearHeading(Pose2d(-58.0, -32.5, Math.toRadians(122.00)))
                .addTemporalMarker(1.0){outRotationServo!!.position = outRotationCenter}
                .addTemporalMarker(1.0){verticalSlideTo(0,0.5)}
                .build()

        val basket3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-58.0, -32.5, Math.toRadians(122.00)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .setReversed(false)
                .build()

        val end: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                .lineTo(Vector2d(-44.0, -44.0))
                .addTemporalMarker(1.0){verticalSlideTo(0,0.5)}
                .addTemporalMarker(1.0){outRotationServo!!.position = outRotationCenter}
                .build()

        waitForStart()

        //START
        verticalSlideTo(1550, 1.0)
        outRotationServo!!.position = outRotationBack

        //BAR
        drive!!.followTrajectorySequence(bar)
        verticalSlideTo(750,1.0)
        sleep(400)
        outClawServo!!.position = outClawOpen
        outRotationServo!!.position = outRotationCenter

        //SAMPLE1
        drive!!.followTrajectorySequence(sample1)
        verticalSlideTo(0,0.5)
        outClawServo!!.position = outClawOpen
        inRotationServo!!.position = inRotationPick
        horizontalSlideTo(1000,1.0)
        sleep(100)
        inStopServo!!.position = inStopClose
        intakeMotor!!.power = 0.7
        sleep(1000)
        inRotationServo!!.position = inRotationTransfer
        sleep(100)
        horizontalSlideTo(0,1.0)
        inStopServo!!.position = inStopOpen
        sleep(1000)
        outClawServo!!.position = outClawClose
        sleep(300)
        verticalSlideTo(verticalSlideHigh, 1.0)
        intakeMotor!!.power = 0.0
        outRotationServo!!.position = outRotationBack


        //BASKET1
        drive!!.followTrajectorySequence(basket1)
        inRotationServo!!.position = inRotationPick
        sleep(300)
        outClawServo!!.position = outClawOpen
        sleep(100)

        //SAMPLE2
        drive!!.followTrajectorySequence(sample2)
        outClawServo!!.position = outClawOpen
        inRotationServo!!.position = inRotationPick
        horizontalSlideTo(1000,1.0)
        sleep(100)
        inStopServo!!.position = inStopClose
        intakeMotor!!.power = 0.7
        sleep(1000)
        inRotationServo!!.position = inRotationTransfer
        sleep(100)
        horizontalSlideTo(0,1.0)
        inStopServo!!.position = inStopOpen
        sleep(1000)
        outClawServo!!.position = outClawClose
        sleep(300)
        intakeMotor!!.power = 0.0
        verticalSlideTo(verticalSlideHigh, 1.0)
        outRotationServo!!.position = outRotationBack

        //BASKET2
        drive!!.followTrajectorySequence(basket2)
        inRotationServo!!.position = inRotationPick
        sleep(300)
        outClawServo!!.position = outClawOpen
        sleep(100)

        //SAMPLE3
        drive!!.followTrajectorySequence(sample3)
        outClawServo!!.position = outClawOpen
        inRotationServo!!.position = inRotationPick
        horizontalSlideTo(1000,1.0)
        sleep(100)
        inStopServo!!.position = inStopClose
        intakeMotor!!.power = 0.7
        sleep(1000)
        inRotationServo!!.position = inRotationTransfer
        sleep(100)
        horizontalSlideTo(0,1.0)
        inStopServo!!.position = inStopOpen
        sleep(1000)
        outClawServo!!.position = outClawClose
        sleep(300)
        verticalSlideTo(verticalSlideHigh, 1.0)
        intakeMotor!!.power = 0.0
        outRotationServo!!.position = outRotationBack
        
        //BASKET3
        drive!!.followTrajectorySequence(basket3)
        inRotationServo!!.position = inRotationPick
        sleep(300)
        outClawServo!!.position = outClawOpen
        sleep(100)

        //END
        drive!!.followTrajectorySequence(end)

        while (opModeIsActive() && !isStopRequested) { drive!!.update() }
        PoseStorage.currentPose = drive!!.poseEstimate
    }
}