package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Arrays


@Autonomous(name = "BASKET6", group = "AAAA")
class BASKET6 : Methods() {
    override fun runOpMode() {
        drive = SampleMecanumDriveCancelable(hardwareMap)
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
        val ColorSens = hardwareMap.get(NormalizedColorSensor::class.java, "color")
        ColorSens.gain = 50.0F

        drive!!.poseEstimate = Pose2d(-32.5, -63.19, Math.toRadians(180.00))

        val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            Arrays.asList(
                TranslationalVelocityConstraint(20.0),
                AngularVelocityConstraint(1.0)
            )
        )

        val all: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-34.0, -63.19, Math.toRadians(180.00)))
                //BASKET 0
                .addTemporalMarker {
                    verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationBackOut
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .waitSeconds(0.5)
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .splineToLinearHeading(
                    Pose2d(-57.0, -57.0, Math.toRadians(225.00)),
                    Math.toRadians(225.00)
                )
                .setReversed(false)
                .resetConstraints()

                //SAMPLE 1
                .splineToLinearHeading(
                    Pose2d(22.30, -61.78, Math.toRadians(0.00)),
                    Math.toRadians(0.0)
                )
                .UNSTABLE_addTemporalMarkerOffset(-0.1) { outClawServo!!.position = outClawOpen }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(-30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    horizontalSlideTo(300, 1.0)
                    inRotationServo!!.position = inRotationPick
                    inStopServo!!.position = inStopClose
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(-1.0) {
                    horizontalSlideTo(800, 1.0)
                }
                //BASKET 1
                .setReversed(true)
                .splineToLinearHeading(
                    Pose2d(-57.0, -57.0, Math.toRadians(45.00)),
                    Math.toRadians(225.00)
                )
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
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                //SAMPLE2
                .lineToLinearHeading(Pose2d(-48.75, -43.3, Math.toRadians(90.0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(-30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(-1.0) {
                    horizontalSlideTo(600, 1.0)
                }
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
                    outRotationServo!!.position = outRotationBackOut
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                //SAMPLE3
                .lineToLinearHeading(Pose2d(-59.0, -43.3, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(-30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(-1.0) {
                    horizontalSlideTo(600, 1.0)
                }
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
                    outRotationServo!!.position = outRotationBackOut
                    horizontalSlideTo(300, 1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                //SAMPLE4
                .lineToLinearHeading(Pose2d(-58.5, -46.5, Math.toRadians(118.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(-30, 0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(-1.0) {
                    horizontalSlideTo(600, 1.0)
                }
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
                    outRotationServo!!.position = outRotationBackOut
                    inStopServo!!.position = inStopClose
                }

                //SAMPLE 5
                .splineTo(Vector2d(-24.5, -11.3), Math.toRadians(0.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.1) {
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = 0.0
                }
                .UNSTABLE_addDisplacementMarkerOffset(0.0) {
                    verticalSlideTo(50, 1.0)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 0.7
                    horizontalSlideTo(300, 1.0)
                }
                .waitSeconds(4.0)
                .UNSTABLE_addDisplacementMarkerOffset(-0.2) {
                    inRotationServo!!.position = inRotationPick
                    intakeMotor!!.power = 0.7
                }
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
                .setReversed(true)
                .splineTo(Vector2d(-57.0, -57.0), Math.toRadians(225.0))
                .setReversed(false)
                .build()

        val backToBasket: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-24.5, -11.3, Math.toRadians(0.00)))
                .setReversed(true)
                .splineTo(Vector2d(-57.0, -57.0), Math.toRadians(225.0))
                .setReversed(false)
                .build()

        waitForStart()
//
//        var startingColor = "blue"
//        while(!opModeIsActive()){
//            if (controller1.a){
//                startingColor = "blue"
//            }
//            if (controller1.b){
//                startingColor = "red"
//            }
//        }

        if (isStopRequested) {return}

        drive!!.followTrajectorySequence(all)
//        var colors: NormalizedRGBA?
//        var moveOn = false
//        var highestValue = 0.0F
//        var colorSeen = "red"
//
//        while(!moveOn){
//            colors = ColorSens.normalizedColors
//            if (colors.red > 0.25 || colors.green > 0.25 || colors.blue > 0.25) {
//                highestValue = colors.red
//                colorSeen = "red"
//                if (colors.green >= highestValue) {
//                    highestValue = colors.green
//                    colorSeen = "green"
//                }
//                if (colors.blue > highestValue) {
//                    colorSeen = "blue"
//                    highestValue = colors.blue
//                }
//                moveOn = true
//            }
//        }
//
//        if (startingColor == "blue" && highestValue )
//
//
        drive!!.updatePoseEstimate()
        PoseStorage.currentPose = drive!!.poseEstimate
    }
}