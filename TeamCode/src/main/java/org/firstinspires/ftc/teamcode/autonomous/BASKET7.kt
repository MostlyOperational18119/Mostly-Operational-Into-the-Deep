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


@Autonomous(name = "BASKET7", group = "AAAA")
class BASKET7 : Methods() {
    override fun runOpMode() {
        drive = SampleMecanumDriveCancelable(hardwareMap)
        initMotors()
        initServosAndSensorsSet()
        transferServo!!.position = transferServoClose
        outClawServo!!.position = outClawClose
        inStopServo!!.position = inStopOpen
        inRotationServo!!.position = inRotationUp
        outRotationServo!!.position = outRotationBackOut + 0.05

        drive!!.poseEstimate = Pose2d(-32.5, -63.19, Math.toRadians(0.00))

        val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            listOf(
                TranslationalVelocityConstraint(12.0),
                AngularVelocityConstraint(1.2)
            )
        )

        val secondSlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            listOf(
                TranslationalVelocityConstraint(60.0),
                AngularVelocityConstraint(3.0)
            )
        )

        val thirdSlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            listOf(
                TranslationalVelocityConstraint(60.0),
                AngularVelocityConstraint(3.0)
            )
        )

        var firstSamp = -11.3

        val begin: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-34.0, -63.19, Math.toRadians(0.00)))
                //BASKET 0
                .addTemporalMarker {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-54.0, -60.0, Math.toRadians(45.00)), Math.toRadians(225.00))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {outClawServo!!.position = outClawOpen}
                .waitSeconds(0.15)

                //SAMPLE 1
                .UNSTABLE_addDisplacementMarkerOffset(40.0) {
                    horizontalSlideTo(800, 0.5)
                    inRotationServo!!.position = inRotationPick
                    inStopServo!!.position = inStopClose
                    intakeMotor!!.power = 1.0
                }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    horizontalSlideTo(300, 1.0)
                }
                .splineToLinearHeading(Pose2d(22.30, -64.0, Math.toRadians(-6.0)), Math.toRadians(-6.0))
                .build()

        val basket1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(22.30, -64.0, Math.toRadians(-6.0)))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                }
                .UNSTABLE_addTemporalMarkerOffset(0.5) {
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(0.9) { outClawServo!!.position = outClawClose }
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                    outRotationServo!!.position = outRotationBackOut
                }
                .setVelConstraint(secondSlowConstraint)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-54.0, -60.0, Math.toRadians(45.00)), Math.toRadians(225.00))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }

                //SAMPLE2
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.4) }
                .lineToLinearHeading(Pose2d(-43.5, -43.3, Math.toRadians(90.0)))
                .build()

        val basket2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-43.5, -43.3, Math.toRadians(90.0)))
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
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                .setVelConstraint(thirdSlowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-54.0, -60.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()

                //SAMPLE3
                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .waitSeconds(0.15)
                .lineToLinearHeading(Pose2d(-55.0, -43.3, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.5) }
                .build()

        val basket3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-55.0, -43.3, Math.toRadians(90.00)))
                //BASKET3
                .setVelConstraint(thirdSlowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-54.0, -60.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()
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
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                //SAMPLE4
                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .waitSeconds(0.15)
                .lineToLinearHeading(Pose2d(-58.5, -46.5, Math.toRadians(118.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .UNSTABLE_addDisplacementMarkerOffset(5.0) { horizontalSlideTo(800, 0.5) }
                .build()

        val basket4: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-58.5, -46.5, Math.toRadians(118.00)))
                //BASKET4
                .setVelConstraint(thirdSlowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-54.0, -60.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()
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
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    inStopServo!!.position = inStopClose
                }

                //SAMPLE 5
                .splineTo(Vector2d(-20.0, -11.5), Math.toRadians(0.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .waitSeconds(0.15)
                .UNSTABLE_addDisplacementMarkerOffset(0.0) {
                    verticalSlideTo(30, 1.0)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.7
                    horizontalSlideTo(300, 1.0)
                }
                .build()

        val basket5: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-24.5, firstSamp, Math.toRadians(0.0)))
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
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    inStopServo!!.position = inStopClose
                }
                .setVelConstraint(secondSlowConstraint)
                .setReversed(true)
                .splineTo(Vector2d(-54.0, -60.0), Math.toRadians(225.0))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .waitSeconds(0.15)
                .splineTo(Vector2d(-24.5, -11.5), Math.toRadians(0.0))
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
            if (elapsedTime.time() > 2.0){ moveOn = true }
        }

        if (startingColor == "blue" && colorSeen == "red"){ spitOut(2000);  requestOpModeStop() }
        else if (startingColor == "red" && colorSeen == "blue"){ spitOut(2000);  requestOpModeStop() }
        else if (colorSeen == "none"){ spitOut(2000);  requestOpModeStop() }

        drive!!.followTrajectorySequence(basket5)
        drive!!.updatePoseEstimate()
    }
}