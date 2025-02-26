package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence


@Autonomous(name = "BASKET6", group = "AAAA")
class BASKET6 : Methods() {
    override fun runOpMode() {
        initMotors()
        initServosAndSensorsSet()
        transferServo!!.position = transferServoClose
        outClawServo!!.position = outClawClose
        inStopServo!!.position = inStopOpen
        inRotationServo!!.position = inRotationUp
        outRotationServo!!.position = outRotationBackOut + 0.04

        drive!!.poseEstimate = Pose2d(-36.5, -63.19, Math.toRadians(0.00))

        val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
                TranslationalVelocityConstraint(12.0),
                AngularVelocityConstraint(1.2)))

        val secondSlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
                TranslationalVelocityConstraint(60.0),
                AngularVelocityConstraint(3.0)))

        val thirdSlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
                TranslationalVelocityConstraint(8.0),
                AngularVelocityConstraint(1.0)))

        val begin: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-38.0, -63.19, Math.toRadians(0.00)))
                .addTemporalMarker{
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)), Math.toRadians(225.00))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) {outClawServo!!.position = outClawOpen}
                .waitSeconds(0.12)

                //SAMPLE 1
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    horizontalSlideTo(300, 1.0)
                }
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    horizontalSlideTo(800, 0.5)
                    inRotationServo!!.position = inRotationPick
                    inStopServo!!.position = inStopClose
                    intakeMotor!!.power = 1.0
                }
                .splineToLinearHeading(Pose2d(16.3, -64.0, Math.toRadians(-6.0)), Math.toRadians(-6.0))
                .build()

        val basket1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(16.3, -64.0, Math.toRadians(-6.0)))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .UNSTABLE_addTemporalMarkerOffset(0.5) {
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(0.9) { outClawServo!!.position = outClawClose }
                .UNSTABLE_addTemporalMarkerOffset(1.1) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }
                .setVelConstraint(secondSlowConstraint)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)), Math.toRadians(225.00))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(-0.1){outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .waitSeconds(0.12)
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.4) }

                //SAMPLE2
                .lineToLinearHeading(Pose2d(-49.0, -43.3, Math.toRadians(90.0)))
                .build()

        val basket2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-49.0, -43.3, Math.toRadians(90.0)))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .UNSTABLE_addTemporalMarkerOffset(0.5) {
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(0.9) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.1) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                    intakeMotor!!.power = -1.0
                }

                .setVelConstraint(thirdSlowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) { outClawServo!!.position = outClawOpen }
                .waitSeconds(0.12)
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.5) }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                //SAMPLE3
                .lineToLinearHeading(Pose2d(-58.5, -43.3, Math.toRadians(90.00)))
                .build()

        val basket3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-58.5, -43.3, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .UNSTABLE_addTemporalMarkerOffset(0.5) {
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(0.9) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.1) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                    intakeMotor!!.power = -1.0
                }

                .setVelConstraint(thirdSlowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()

                //SAMPLE4
                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) { outClawServo!!.position = outClawOpen }
                .waitSeconds(0.12)
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.5) }
                .lineToLinearHeading(Pose2d(-60.5, -46.5, Math.toRadians(118.00)))
                .build()

        val basket4: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-60.5, -46.5, Math.toRadians(118.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .UNSTABLE_addTemporalMarkerOffset(0.5) {
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(0.9) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.1) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                    intakeMotor!!.power = -1.0
                }

                .setVelConstraint(thirdSlowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()//loganwashere

                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) { outClawServo!!.position = outClawOpen }
                .waitSeconds(0.12)
                .UNSTABLE_addDisplacementMarkerOffset(0.0) {
                    verticalSlideTo(30, 1.0)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.7
                    horizontalSlideTo(300, 1.0)
                }

                //SAMPLE 5
                .splineTo(Vector2d(-24.5, -11.5), Math.toRadians(0.0))
                .build()

        val basket5: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-24.5, -11.5, Math.toRadians(0.0)))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .UNSTABLE_addTemporalMarkerOffset(0.5) {
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(0.9) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.1) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                .setVelConstraint(secondSlowConstraint)
                .setReversed(true)
                .splineTo(Vector2d(-60.0, -60.0), Math.toRadians(225.0))
                .setReversed(false)
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .waitSeconds(0.12)
                .build()

        while(!opModeIsActive()){
            telemetry.addData("Starting Color :", startingColor)
            telemetry.update()
        }

        if (isStopRequested) {return}
        waitForStart()

        drive!!.followTrajectorySequence(begin)
        elapsedTime.reset()
        colorSeen = "none"
        moveOn = false
        while(!moveOn && opModeIsActive()){
            telemetry.addLine(colorSeen)
            telemetry.update()
            colors = colorSensor!!.normalizedColors
            if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true }
            else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true }
            else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true }
            if (elapsedTime.time() > 1.2){ moveOn = true }
        }

        drive!!.followTrajectorySequence(basket1)
        elapsedTime.reset()
        colorSeen = "none"
        moveOn = false
        while(!moveOn && opModeIsActive()){
            telemetry.addLine(colorSeen)
            telemetry.update()
            colors = colorSensor!!.normalizedColors
            if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true }
            else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true }
            else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true }
            if (elapsedTime.time() > 1.2){ moveOn = true }
        }

        drive!!.followTrajectorySequence(basket2)
        elapsedTime.reset()
        colorSeen = "none"
        moveOn = false
        while(!moveOn && opModeIsActive()){
            telemetry.addLine(colorSeen)
            telemetry.update()
            colors = colorSensor!!.normalizedColors
            if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true }
            else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true }
            else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true }
            if (elapsedTime.time() > 1.2){ moveOn = true }
        }

        drive!!.followTrajectorySequence(basket3)
        elapsedTime.reset()
        colorSeen = "none"
        moveOn = false
        while(!moveOn && opModeIsActive()){
            telemetry.addLine(colorSeen)
            telemetry.update()
            colors = colorSensor!!.normalizedColors
            if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true }
            else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true }
            else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true }
            if (elapsedTime.time() > 1.2){ moveOn = true }
        }

        drive!!.followTrajectorySequence(basket4)
        drive!!.updatePoseEstimate()
        inRotationServo!!.position = inRotationPick
        horizontalSlideTo(900,0.2)

        elapsedTime.reset()
        colorSeen = "none"
        moveOn = false
        while(!moveOn && opModeIsActive()){
            telemetry.addLine(colorSeen)
            telemetry.update()
            colors = colorSensor!!.normalizedColors
            if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true }
            else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true }
            else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true }
            if (elapsedTime.time() > 1.5){ moveOn = true }
        }

        if (startingColor == "blue" && colorSeen == "red"){ spitOut(2000);  requestOpModeStop() }
        else if (startingColor == "red" && colorSeen == "blue"){ spitOut(2000);  requestOpModeStop() }
        else if (colorSeen == "none"){ spitOut(2000);  requestOpModeStop() }

        drive!!.followTrajectorySequence(basket5)
        drive!!.updatePoseEstimate()

        outRotationServo!!.position = outRotationCenter
        verticalSlideTo(30, 1.0)
    }
}