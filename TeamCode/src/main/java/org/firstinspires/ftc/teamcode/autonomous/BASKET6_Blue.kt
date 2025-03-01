package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence


@Autonomous(name = "BASKET6_Blue", group = "AAAA")
class BASKET6_Blue : Methods() {
    override fun runOpMode() {
        startingColor = "blue"
        val autoTimer = ElapsedTime()

        initMotors()
        initServosAndSensorsSet()
        initOdometry()
        transferServo!!.position = transferServoClose
        outClawServo!!.position = outClawClose
        inStopServo!!.position = inStopOpen
        inRotationServo!!.position = inRotationUp
        outRotationServo!!.position = outRotationBackOut + 0.04

        drive!!.poseEstimate = Pose2d(-36.5, -63.19, Math.toRadians(0.00))

        val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(13.0),
            AngularVelocityConstraint(1.25)))

        val basket1SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(44.0),
            AngularVelocityConstraint(3.0)))

        val basket2SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(3.9),
            AngularVelocityConstraint(1.0)))

        val basket3SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(3.4),
            AngularVelocityConstraint(1.0)))

        val basket4SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(3.9),
            AngularVelocityConstraint(1.0)))

        val basket5SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(37.0),
            AngularVelocityConstraint(2.5)))

        val begin: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-38.0, -63.19, Math.toRadians(0.00)))
                .addTemporalMarker{
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-59.0, -61.0, Math.toRadians(45.00)), Math.toRadians(225.00))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(-0.05) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) {outClawServo!!.position = outClawOpen}
                .waitSeconds(0.13)

                //SAMPLE 1
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(20, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    horizontalSlideTo(500, 1.0)
                }
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    inRotationServo!!.position = inRotationPick
                    inStopServo!!.position = inStopClose
                    intakeMotor!!.power = 0.6
                }
                .splineToLinearHeading(Pose2d(19.5, -63.75, Math.toRadians(-6.0)), Math.toRadians(-6.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.2) {
                    horizontalSlideTo(800,0.7)
                }
                .build()

        val recoverSample1 : TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(19.5, -63.75, Math.toRadians(-6.0)))
                .addTemporalMarker {horizontalSlideTo(50, 1.0); intakeMotor!!.power = -1.0}
                .lineToLinearHeading(Pose2d(-49.0, -50.3, Math.toRadians(90.0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5){horizontalSlideTo(600, 0.3); intakeMotor!!.power = 1.0}
                .build()

        val basket1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(19.5, -63.75, Math.toRadians(-6.0)))
                .addTemporalMarker(0.0) {
                    horizontalSlideTo(-30, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .addTemporalMarker(0.45) {
                    inStopServo!!.position = inStopAutoOpen
                }
                .addTemporalMarker(0.8) { outClawServo!!.position = outClawClose }
                .addTemporalMarker(1.2) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inRotationServo!!.position = inRotationPick
                }
                .setVelConstraint(basket1SlowConstraint)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-59.0, -61.0, Math.toRadians(45.00)), Math.toRadians(225.00))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(-0.05){outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) {
                    inStopServo!!.position = inStopClose
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .waitSeconds(0.13)
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(20, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.6
                    transferServo!!.position = transferServoClose
                }
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.4) }

                //SAMPLE2
                .lineToLinearHeading(Pose2d(-48.3, -52.3, Math.toRadians(90.0)))
                .build()

        val recoverSample2 : TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-48.3, -52.3, Math.toRadians(90.0)))
                .addTemporalMarker {horizontalSlideTo(50, 1.0); intakeMotor!!.power = -1.0}
                .lineToLinearHeading(Pose2d(-57.5, -52.3, Math.toRadians(90.0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5){horizontalSlideTo(600, 0.3); intakeMotor!!.power = 0.5}
                .build()

        val basket2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-48.3, -52.3, Math.toRadians(90.0)))
                .addTemporalMarker(0.0) {
                    horizontalSlideTo(-30, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .addTemporalMarker(0.45) {
                    inStopServo!!.position = inStopAutoOpen
                }
                .addTemporalMarker(0.8) {
                    outClawServo!!.position = outClawClose
                }
                .addTemporalMarker(1.2) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(200, 1.0)
                    inRotationServo!!.position = inRotationPick
                    intakeMotor!!.power = -1.0
                }

                .setVelConstraint(basket2SlowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-59.0, -61.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(-0.05) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) { outClawServo!!.position = outClawOpen }
                .waitSeconds(0.13)
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.4) }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    inStopServo!!.position = inStopClose
                    verticalSlideTo(20, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.6
                    transferServo!!.position = transferServoClose
                }
                //SAMPLE3
                .lineToLinearHeading(Pose2d(-57.5, -52.3, Math.toRadians(90.00)))
                .build()

        val recoverSample3 : TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-57.5, -52.3, Math.toRadians(90.00)))
                .addTemporalMarker {horizontalSlideTo(200, 1.0); intakeMotor!!.power = -1.0}
                .lineToLinearHeading(Pose2d(-57.5, -46.5, Math.toRadians(130.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.3){horizontalSlideTo(800, 0.5); intakeMotor!!.power = 0.5}
                .build()


        val basket3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-57.5, -52.3, Math.toRadians(90.00)))
                .addTemporalMarker(0.0) {
                    horizontalSlideTo(-30, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .addTemporalMarker(0.45) {
                    inStopServo!!.position = inStopAutoOpen
                }
                .addTemporalMarker(0.8) {
                    outClawServo!!.position = outClawClose
                }
                .addTemporalMarker(1.2) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inRotationServo!!.position = inRotationPick
                    intakeMotor!!.power = -1.0
                }

                .setVelConstraint(basket3SlowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-59.0, -61.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()

                //SAMPLE4
                .UNSTABLE_addTemporalMarkerOffset(-0.05) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) { outClawServo!!.position = outClawOpen }
                .waitSeconds(0.13)
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(20, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.6
                    transferServo!!.position = transferServoClose
                    inStopServo!!.position = inStopClose
                }
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.5) }
                .lineToLinearHeading(Pose2d(-57.5, -46.5, Math.toRadians(130.00)))
                .build()

        val recoverSample4 : TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-57.5, -46.5, Math.toRadians(130.00)))
                .addTemporalMarker {horizontalSlideTo(100, 1.0); intakeMotor!!.power = -1.0}
                .turn(Math.toRadians(-30.0))
                .UNSTABLE_addTemporalMarkerOffset(0.0){inRotationServo!!.position = inRotationTransfer}
                .splineTo(Vector2d(-22.0, -11.5), Math.toRadians(0.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.05){horizontalSlideTo(800, 0.3); intakeMotor!!.power = 1.0; inRotationServo!!.position = inRotationPick}
                .build()

        val basket4: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-57.5, -46.5, Math.toRadians(130.00)))
                .addTemporalMarker(0.0) {
                    horizontalSlideTo(-30, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .addTemporalMarker(0.45) {
                    inStopServo!!.position = inStopAutoOpen
                }
                .addTemporalMarker(0.8) {
                    outClawServo!!.position = outClawClose
                }
                .addTemporalMarker(1.2) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inRotationServo!!.position = inRotationPick
                    intakeMotor!!.power = -1.0
                }

                .setVelConstraint(basket4SlowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-59.0, -61.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()//loganwashere

                .UNSTABLE_addTemporalMarkerOffset(-0.05) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) { outClawServo!!.position = outClawOpen }
                .waitSeconds(0.13)
                .UNSTABLE_addDisplacementMarkerOffset(0.0) {
                    inStopServo!!.position = inStopClose
                    verticalSlideTo(20, 1.0)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.6
                    horizontalSlideTo(400, 1.0)
                    transferServo!!.position = transferServoClose
                    inRotationServo!!.position = inRotationTransfer
                }

                .splineTo(Vector2d(-22.0, -11.5), Math.toRadians(0.0))
                //SAMPLE 5
                .UNSTABLE_addTemporalMarkerOffset(-0.05) {
                    inRotationServo!!.position = inRotationPick
                    horizontalSlideTo(900,0.3)
                }
                .build()

        val basket5: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-22.0, -11.5, Math.toRadians(0.0)))
                .addTemporalMarker(0.0) {
                    horizontalSlideTo(-30, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .addTemporalMarker(0.45) {
                    inStopServo!!.position = inStopAutoOpen
                }
                .addTemporalMarker(0.8) {
                    outClawServo!!.position = outClawClose
                }
                .addTemporalMarker(1.2) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inRotationServo!!.position = inRotationPick
                }

                .setVelConstraint(basket5SlowConstraint)
                .setReversed(true)
                .splineTo(Vector2d(-59.0, -61.0), Math.toRadians(225.0))
                .setReversed(false)
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(-0.05) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.02) {   outClawServo!!.position = outClawOpen}
                .waitSeconds(0.13)
                .build()

        while(!opModeIsActive()){
            telemetry.addData("Starting Color :", startingColor)
            telemetry.update()
        }

        if (isStopRequested) {return}
        waitForStart()

        autoTimer.reset()

        drive!!.followTrajectorySequence(begin)

        elapsedTime.reset()
        colorSeen = "none"
        moveOn = false
        while(!moveOn && opModeIsActive()){
            telemetry.addLine(colorSeen)
            telemetry.update()
            colors = colorSensor!!.normalizedColors
            colors = colorSensor!!.normalizedColors
            if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update()}
            else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            if (elapsedTime.time() > 1.5){ moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
        }

        if (colorSeen == "none") { drive!!.followTrajectorySequence(recoverSample1) }
        else{ drive!!.followTrajectorySequence(basket1) }

        elapsedTime.reset()
        colorSeen = "none"
        moveOn = false
        while(!moveOn && opModeIsActive()){
            telemetry.addLine(colorSeen)
            telemetry.update()
            colors = colorSensor!!.normalizedColors
            if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update()}
            else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            if (elapsedTime.time() > 1.0){ moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
        }

        if (colorSeen == "none") { drive!!.followTrajectorySequence(recoverSample2) }
        else{ drive!!.followTrajectorySequence(basket2) }

        elapsedTime.reset()
        colorSeen = "none"
        moveOn = false
        while(!moveOn && opModeIsActive()){
            telemetry.addLine(colorSeen)
            telemetry.update()
            colors = colorSensor!!.normalizedColors
            if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            if (elapsedTime.time() > 1.0){ moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
        }

        if (colorSeen == "none") { drive!!.followTrajectorySequence(recoverSample3) }
        else{ drive!!.followTrajectorySequence(basket3) }

        elapsedTime.reset()
        colorSeen = "none"
        moveOn = false
        while(!moveOn && opModeIsActive()){
            telemetry.addLine(colorSeen)
            telemetry.update()
            colors = colorSensor!!.normalizedColors
            if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            if (elapsedTime.time() > 1.0){ moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
        }

        if (colorSeen == "none") { drive!!.followTrajectorySequence(recoverSample4) }
        else{ drive!!.followTrajectorySequence(basket4) }
        drive!!.updatePoseEstimate()

        elapsedTime.reset()
        colorSeen = "none"
        moveOn = false
        while(!moveOn && opModeIsActive()){
            telemetry.addLine(colorSeen)
            telemetry.addLine(autoTimer.time().toString())
            telemetry.update()
            colors = colorSensor!!.normalizedColors
            if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
            if (elapsedTime.time() > 2.5){ moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
        }

        if (startingColor == "blue" && colorSeen == "red"){ spitOut(2000);  requestOpModeStop() }
        else if (startingColor == "red" && colorSeen == "blue"){ spitOut(2000);  requestOpModeStop() }
        else if (colorSeen == "none"){ spitOut(2000);  requestOpModeStop() }

        if (autoTimer.time() < 25.0) {
            drive!!.followTrajectorySequence(basket5)
            drive!!.updatePoseEstimate()
        }
        outRotationServo!!.position = outRotationCenter
    }
}