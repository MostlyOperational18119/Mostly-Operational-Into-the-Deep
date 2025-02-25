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

        drive!!.poseEstimate = Pose2d(-32.5, -63.19, Math.toRadians(0.00))

        val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            listOf(
                TranslationalVelocityConstraint(30.0),
                AngularVelocityConstraint(1.2)
            )
        )

        var firstSamp = -11.3
        var secondSamp = -8.3
        while(!opModeIsActive()){
            if (controller1.a){ startingColor = "blue" }
            if (controller1.b){ startingColor = "red" }
            if (controller1.dpad_up){firstSamp += 0.1}
            if (controller1.dpad_down){firstSamp -= 0.1}
            if (controller1.dpad_left){secondSamp += 0.1}
            if (controller1.dpad_right){secondSamp -= 0.1}
            telemetry.addData("Starting Color :", startingColor)
            telemetry.addData("First Sample Inches :", -1 * firstSamp)
            telemetry.addData("Second Sample Inches :", -1 * secondSamp)
            telemetry.update()
        }

        val all: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-34.0, -63.19, Math.toRadians(0.00)))
                //BASKET 0
                .addTemporalMarker {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .waitSeconds(0.2)
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)), Math.toRadians(225.00))
                .setReversed(false)
                .resetConstraints()

                //SAMPLE 1
                .splineToLinearHeading(Pose2d(22.30, -61.78, Math.toRadians(0.00)), Math.toRadians(0.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.25) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outClawServo!!.position = outClawOpen }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(-30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    horizontalSlideTo(300, 1.0)
                    inRotationServo!!.position = inRotationPick
                    inStopServo!!.position = inStopClose
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(-0.5) { horizontalSlideTo(800, 0.9) }
                //BASKET 1
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)), Math.toRadians(225.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(1.5) { outClawServo!!.position = outClawClose }
                .UNSTABLE_addTemporalMarkerOffset(1.7) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                //SAMPLE2
                .lineToLinearHeading(Pose2d(-48.75, -43.3, Math.toRadians(90.0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(-30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5) { horizontalSlideTo(800, 0.5) }
                //BASKET2
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(1.5) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.7) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                //SAMPLE3
                .lineToLinearHeading(Pose2d(-59.0, -43.3, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(-30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5) { horizontalSlideTo(800, 0.5) }
                //BASKET3
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(1.5) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.7) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                //SAMPLE4
                .lineToLinearHeading(Pose2d(-58.5, -46.5, Math.toRadians(118.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(-30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5) { horizontalSlideTo(800, 0.5) }
                //BASKET4
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(1.5) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.7) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    inStopServo!!.position = inStopClose
                }

                //SAMPLE 5
                .splineTo(Vector2d(-24.5, firstSamp), Math.toRadians(0.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.25) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = 0.0
                    inRotationServo!!.position = inRotationUpAuto
                }
                .UNSTABLE_addDisplacementMarkerOffset(0.0) {
                    verticalSlideTo(50, 1.0)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.7
                    horizontalSlideTo(300, 1.0)
                }
                .build()

        val backToBasket1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-24.5, -11.3, Math.toRadians(0.0)))
                .setReversed(true)
                .splineTo(Vector2d(-57.0, -57.0), Math.toRadians(225.0))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(1.5) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.7) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationUp
                    inStopServo!!.position = inStopClose
                }
                .splineTo(Vector2d(-24.5, -8.3), Math.toRadians(0.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.25) { outRotationServo!!.position = outRotationBackOut}
                .UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = 0.0
                    inRotationServo!!.position = inRotationUpAuto
                }
                .UNSTABLE_addDisplacementMarkerOffset(0.0) {
                    verticalSlideTo(50, 1.0)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.7
                    horizontalSlideTo(300, 1.0)
                }
                .build()

        val backToBasket2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-24.5, -8.3, Math.toRadians(0.0)))
                .setReversed(true)
                .splineTo(Vector2d(-57.0, -57.0), Math.toRadians(225.0))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(1.5) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.7) {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationBackOut
                    inStopServo!!.position = inStopClose
                }
                .build()

        if (isStopRequested) {return}
        waitForStart()

        drive!!.followTrajectorySequence(all)
        inRotationServo!!.position = inRotationPick
        horizontalSlideTo(900,0.2)

        var moveOn = false
        val timer = ElapsedTime()
        timer.reset()
        startingColor = "none"
        while(!moveOn){
            telemetry.addLine(colorSeen)
            telemetry.update()
            colors = colorSensor!!.normalizedColors
            if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true }
            else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true }
            else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true }
            if (timer.time() > 2.0){ moveOn = true }
        }
        moveOn = false

        if (startingColor == "blue" && colorSeen == "red"){
            intakeMotor!!.power = -1.0
            sleep(2000)
            intakeMotor!!.power = 0.0
        }
        else if (startingColor == "red" && colorSeen == "blue"){
            intakeMotor!!.power = -1.0
            sleep(2000)
            intakeMotor!!.power = 0.0
        }
        else if (startingColor == "none"){
            intakeMotor!!.power = -1.0
            sleep(2000)
            intakeMotor!!.power = 0.0
        }
        else{
            drive!!.followTrajectorySequence(backToBasket1)
            inRotationServo!!.position = inRotationPick
            horizontalSlideTo(900,0.2)
            timer.reset()
            startingColor = "none"
            while(!moveOn){
                telemetry.addLine(colorSeen)
                telemetry.update()
                colors = colorSensor!!.normalizedColors
                if (colors!!.green > 0.6){ colorSeen = "yellow"; moveOn = true }
                else if (colors!!.red > 0.6){ colorSeen = "red"; moveOn = true }
                else if (colors!!.blue > 0.6){ colorSeen= "blue"; moveOn = true }
                if (timer.time() > 2.0){ moveOn = true }
            }
            moveOn = false
            if (startingColor == "blue" && colorSeen == "red"){
                intakeMotor!!.power = -1.0
                sleep(2000)
                intakeMotor!!.power = 0.0
            }
            else if (startingColor == "red" && colorSeen == "blue"){
                intakeMotor!!.power = -1.0
                sleep(2000)
                intakeMotor!!.power = 0.0
            }
            else if (startingColor == "none"){
                intakeMotor!!.power = -1.0
                sleep(2000)
                intakeMotor!!.power = 0.0
            }
            else{
                drive!!.followTrajectorySequence(backToBasket2)
                outClawServo!!.position = outClawOpen
                sleep(100)
                outRotationServo!!.position = outRotationCenter
                verticalSlideTo(50,1.0)
            }
        }

        drive!!.updatePoseEstimate()
        PoseStorage.currentPose = drive!!.poseEstimate
    }
}