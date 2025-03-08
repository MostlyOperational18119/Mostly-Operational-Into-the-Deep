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
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence


@Autonomous(name = "BASKETObs6_Red", group = "AAAA")
class BASKET6Red : Methods() {
    override fun runOpMode() {
        startingColor = "red"

        val autoTimer = ElapsedTime()

        initMotors()
        initServosAndSensorsNoSet()

        val drive = SampleMecanumDriveCancelable(hardwareMap)
        transferServo!!.position = transferServoClose
        outClawServo!!.position = outClawClose
        inStopServo!!.position = inStopOpen
        inRotationServo!!.position = inRotationUp
        outRotationServo!!.position = outRotationBackOut + 0.04

        drive!!.poseEstimate = Pose2d(-36.5, -63.19, Math.toRadians(0.00))

        val beginSlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(13.4),
            AngularVelocityConstraint(1.25)))

        val basket1SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(33.0),
            AngularVelocityConstraint(3.0)))

        val basket2SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(4.0),
            AngularVelocityConstraint(1.0)))

        val basket3SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(3.0),
            AngularVelocityConstraint(1.0)))

        val basket4SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(4.0),
            AngularVelocityConstraint(1.0)))

        val basket5SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
            TranslationalVelocityConstraint(27.0),
            AngularVelocityConstraint(2.5)))

        val begin: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-38.0, -63.19, Math.toRadians(0.00)))
                .addTemporalMarker{
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpBack
                }

                .setVelConstraint(beginSlowConstraint)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-59.0, -61.0, Math.toRadians(45.00)), Math.toRadians(225.00))
                .setReversed(false)
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(-0.05) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.0) {outClawServo!!.position = outClawOpen}
                .waitSeconds(0.11)

                //OBSERVATION ZONE
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    inStopServo!!.position = inStopClose
                    verticalSlideTo(20, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.4
                    horizontalSlideTo(400, 1.0)
                }
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    inRotationServo!!.position = inRotationPick
                }
                .splineToLinearHeading(Pose2d(19.5, -64.0, Math.toRadians(-6.0)), Math.toRadians(-6.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.2) {
                    horizontalSlideTo(800,0.7)
                }
                .build()

        val recoverToTape1 : TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(19.5, -64.0, Math.toRadians(-6.0)))
                .addTemporalMarker {horizontalSlideTo(50, 1.0); intakeMotor!!.power = -1.0}
                .lineToLinearHeading(Pose2d(-48.3, -50.3, Math.toRadians(90.0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5){horizontalSlideTo(600, 0.3); intakeMotor!!.power = 0.4}
                .build()

        val observationToBasketToTape1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(19.5, -64.0, Math.toRadians(-6.0)))
                .addTemporalMarker(0.0) {
                    horizontalSlideTo(-30, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    transferServo!!.position = transferServoClose}
                .addTemporalMarker(0.55) { inStopServo!!.position = inStopAutoOpen }
                .addTemporalMarker(1.2) { outClawServo!!.position = outClawClose }
                .addTemporalMarker(1.5) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inRotationServo!!.position = inRotationPick
                    intakeMotor!!.power = -1.0
                }

                .setVelConstraint(basket1SlowConstraint)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-59.0, -61.0, Math.toRadians(45.00)), Math.toRadians(225.00))
                .setReversed(false)
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(-0.05) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.0) {outClawServo!!.position = outClawOpen}
                .waitSeconds(0.11)

                //TAPE 1
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    inStopServo!!.position = inStopClose
                    verticalSlideTo(20, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.4
                }
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.4) }
                .lineToLinearHeading(Pose2d(-48.3, -50.3, Math.toRadians(90.0)))
                .build()

        val recoverToTape2 : TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-48.3, -50.3, Math.toRadians(90.0)))
                .addTemporalMarker {horizontalSlideTo(50, 1.0); intakeMotor!!.power = -1.0}
                .lineToLinearHeading(Pose2d(-59.0, -50.3, Math.toRadians(90.0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5){horizontalSlideTo(600, 0.3); intakeMotor!!.power = 0.4}
                .build()

        val tape1ToBasketToTape2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-48.3, -50.3, Math.toRadians(90.0)))
                .addTemporalMarker(0.0) {
                    horizontalSlideTo(-30, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    transferServo!!.position = transferServoClose
                }
                .addTemporalMarker(0.45) { inStopServo!!.position = inStopAutoOpen }
                .addTemporalMarker(1.1) { outClawServo!!.position = outClawClose }
                .addTemporalMarker(1.3) {
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
                .UNSTABLE_addTemporalMarkerOffset(0.0) {outClawServo!!.position = outClawOpen}
                .waitSeconds(0.11)

                //TAPE 2
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.4) }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    inStopServo!!.position = inStopClose
                    verticalSlideTo(20, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.4
                }
                .lineToLinearHeading(Pose2d(-59.0, -50.3, Math.toRadians(90.00)))
                .build()

        val recoverToTape3 : TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-59.0, -50.3, Math.toRadians(90.00)))
                .addTemporalMarker {horizontalSlideTo(300, 1.0); intakeMotor!!.power = -1.0}
                .waitSeconds(0.2)
                .lineToLinearHeading(Pose2d(-58.0, -46.5, Math.toRadians(127.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.1){horizontalSlideTo(800, 0.5); intakeMotor!!.power = 0.4}
                .build()

        val tape2ToBasketToTape3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-59.0, -50.3, Math.toRadians(90.00)))
                .addTemporalMarker(0.0) {
                    horizontalSlideTo(-30, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    transferServo!!.position = transferServoClose
                }
                .addTemporalMarker(0.45) { inStopServo!!.position = inStopAutoOpen }
                .addTemporalMarker(1.1) { outClawServo!!.position = outClawClose }
                .addTemporalMarker(1.3) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(200, 1.0)
                    inRotationServo!!.position = inRotationPick
                    intakeMotor!!.power = -1.0
                }

                .setVelConstraint(basket3SlowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-59.0, -61.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(-0.05) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.0) {outClawServo!!.position = outClawOpen}
                .waitSeconds(0.11)

                //TAPE 3
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(20, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.4
                    inStopServo!!.position = inStopClose
                }
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.5) }
                .lineToLinearHeading(Pose2d(-58.0, -46.5, Math.toRadians(127.00)))
                .build()

        val recoverToSubmersible : TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-58.0, -46.5, Math.toRadians(127.00)))
                .addTemporalMarker {horizontalSlideTo(250, 1.0); intakeMotor!!.power = -1.0}
                .turn(Math.toRadians(-30.0))
                .UNSTABLE_addTemporalMarkerOffset(0.0){inRotationServo!!.position = inRotationTransfer}
                .splineTo(Vector2d(-22.0, -11.5), Math.toRadians(0.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.05){
                    horizontalSlideTo(900, 0.15)
                    intakeMotor!!.power = 0.4
                    inRotationServo!!.position = inRotationPick
                }
                .build()

        val tape3ToBasketToSubmersible: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-58.0, -46.5, Math.toRadians(127.00)))
                .addTemporalMarker(0.0) {
                    horizontalSlideTo(-30, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    transferServo!!.position = transferServoClose
                }
                .addTemporalMarker(0.45) { inStopServo!!.position = inStopAutoOpen }
                .addTemporalMarker(1.1) { outClawServo!!.position = outClawClose }
                .addTemporalMarker(1.3) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(200, 1.0)
                    inRotationServo!!.position = inRotationPick
                    intakeMotor!!.power = -1.0
                }

                .setVelConstraint(basket4SlowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-59.0, -61.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(-0.05) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.0) {outClawServo!!.position = outClawOpen}
                .waitSeconds(0.11)

                //SUBMERSIBLE
                .UNSTABLE_addDisplacementMarkerOffset(0.0) {
                    inStopServo!!.position = inStopClose
                    verticalSlideTo(20, 1.0)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.4
                    horizontalSlideTo(250, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .splineTo(Vector2d(-22.0, -11.5), Math.toRadians(0.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.05) {
                    inRotationServo!!.position = inRotationPick
                    horizontalSlideTo(900,0.15)
                }
                .build()

        val submersibleToBasket: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-22.0, -11.5, Math.toRadians(0.0)))
                .addTemporalMarker(0.0) {
                    horizontalSlideTo(-30, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    transferServo!!.position = transferServoClose
                }
                .addTemporalMarker(0.45) { inStopServo!!.position = inStopAutoOpen }
                .addTemporalMarker(1.1) { outClawServo!!.position = outClawClose }
                .addTemporalMarker(1.3) {
                    transferServo!!.position = transferServoOpen
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(200, 1.0)
                    inRotationServo!!.position = inRotationPick
                    intakeMotor!!.power = -1.0
                }

                .setVelConstraint(basket5SlowConstraint)
                .setReversed(true)
                .splineTo(Vector2d(-59.0, -61.0), Math.toRadians(225.0))
                .setReversed(false)
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(-0.05) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.0) {outClawServo!!.position = outClawOpen}
                .waitSeconds(0.11)
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

        if (colorSeen == "none") { drive!!.followTrajectorySequence(recoverToTape1) }
        else{ drive!!.followTrajectorySequence(observationToBasketToTape1) }

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

        if (colorSeen == "none") { drive!!.followTrajectorySequence(recoverToTape2) }
        else{ drive!!.followTrajectorySequence(tape1ToBasketToTape2) }

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

        if (colorSeen == "none") { drive!!.followTrajectorySequence(recoverToTape3) }
        else{ drive!!.followTrajectorySequence(tape2ToBasketToTape3) }

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

        if (colorSeen == "none") { drive!!.followTrajectorySequence(recoverToSubmersible) }
        else{ drive!!.followTrajectorySequence(tape3ToBasketToSubmersible) }
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

        if (startingColor == "blue" && colorSeen == "red"){ spitOut(1000);  requestOpModeStop() }
        else if (startingColor == "red" && colorSeen == "blue"){ spitOut(1000);  requestOpModeStop() }
        else if (colorSeen == "none"){ spitOut(1000);  requestOpModeStop() }

        if (autoTimer.time() < 27.75) {
            drive!!.followTrajectorySequence(submersibleToBasket)
            drive!!.updatePoseEstimate()
        }
        outRotationServo!!.position = outRotationCenter
        sleep(100)
        verticalSlideTo(0,1.0)
    }
}