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
        val colorSens = hardwareMap.get(NormalizedColorSensor::class.java, "color")
        colorSens.gain = 50.0F

        drive!!.poseEstimate = Pose2d(-32.5, -63.19, Math.toRadians(180.00))

        val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            listOf(
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
                .build()

        val backToBasket: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(drive!!.poseEstimate)
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

        waitForStart()

        var startingColor = "blue"
        while(!opModeIsActive()){
            if (controller1.a){ startingColor = "blue" }
            if (controller1.b){ startingColor = "red" }
        }

        if (isStopRequested) {return}

        drive!!.followTrajectorySequence(all)
        inRotationServo!!.position = inRotationPick
        horizontalSlideTo(900,0.1)

        var colors: NormalizedRGBA?
        var moveOn = false
        var colorSeen = "red"

        while(!moveOn){
            telemetry.addLine(colorSeen)
            telemetry.update()
            colors = colorSens.normalizedColors
            if (colors.red > 0.6){ colorSeen = "red"; moveOn = true }
            if (colors.green > 0.2){ colorSeen = "yellow"; moveOn = true }
            if (colors.blue > 0.6){ colorSeen = "blue"; moveOn = true }
        }

        if (startingColor == "blue" && colorSeen == "red"){
            intakeMotor!!.power = -0.7
            sleep(2000)
            intakeMotor!!.power = 0.0
        }
        else if (startingColor == "red" && colorSeen == "blue"){
            intakeMotor!!.power = -0.7
            sleep(2000)
            intakeMotor!!.power = 0.0
        }
        else{
            drive!!.followTrajectorySequence(backToBasket)
            outClawServo!!.position = outClawOpen
            outRotationServo!!.position = outRotationCenter
        }

        drive!!.updatePoseEstimate()
        PoseStorage.currentPose = drive!!.poseEstimate
    }
}