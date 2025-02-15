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
import java.util.Arrays


@Autonomous(name = "BASKET5", group = "AAAA")
class BASKET5 : Methods() {
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

        drive!!.poseEstimate = Pose2d(-32.5, -63.19, Math.toRadians(180.00))

        val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            Arrays.asList(
                TranslationalVelocityConstraint(20.0),
                AngularVelocityConstraint(1.0)
            )
        )

        val all: TrajectorySequence =
            drive!!.trajectorySequenceBuilder( Pose2d(-34.0, -63.19, Math.toRadians(180.00)))
                    //BASKET 0
                .addTemporalMarker{ verticalSlideTo(verticalSlideHigh, 1.0)
                    outRotationServo!!.position = outRotationBackOut
                    outSwivelServo!!.position = outSwivelPerpBack}
                .waitSeconds(0.5)
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(225.00)), Math.toRadians(225.00))
                .UNSTABLE_addTemporalMarkerOffset(-0.3){outClawServo!!.position = outClawOpen}
                .setReversed(false)
                .resetConstraints()

                    //SAMPLE 1
                .splineToLinearHeading(Pose2d(22.30, -61.78, Math.toRadians(0.00)),Math.toRadians(0.0))
                .UNSTABLE_addTemporalMarkerOffset(0.0){verticalSlideTo(-30,0.5)
                    outRotationServo!!.position = outRotationCenter
                    horizontalSlideTo(300, 1.0)
                    inRotationServo!!.position = inRotationPick
                    inStopServo!!.position = inStopClose
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(-1.0){
                    horizontalSlideTo(800, 1.0)
                }
                    //BASKET 1
                .setReversed(true)
                .splineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)), Math.toRadians(225.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.0){
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(1.5) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.7) {
                    verticalSlideTo(verticalSlideHigh,1.0)
                    outRotationServo!!.position = outRotationBackOut
                    horizontalSlideTo(300,1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                    //SAMPLE2
                .lineToLinearHeading(Pose2d(-48.75, -43.3, Math.toRadians(90.0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.3){
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .UNSTABLE_addTemporalMarkerOffset(0.0){verticalSlideTo(-30,0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(-1.0){
                    horizontalSlideTo(600,1.0)
                }
                    //BASKET2
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()

                .UNSTABLE_addTemporalMarkerOffset(0.0){
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(1.5) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.7) {
                    verticalSlideTo(verticalSlideHigh,1.0)
                    outRotationServo!!.position = outRotationBackOut
                    horizontalSlideTo(300,1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                    //SAMPLE3
                .lineToLinearHeading(Pose2d(-59.0, -43.3, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.3){
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .UNSTABLE_addTemporalMarkerOffset(0.0){verticalSlideTo(-30,0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(-1.0){
                    horizontalSlideTo(600,1.0)
                }
                //BASKET3
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.0){
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(1.5) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.7) {
                    verticalSlideTo(verticalSlideHigh,1.0)
                    outRotationServo!!.position = outRotationBackOut
                    horizontalSlideTo(300,1.0)
                    inStopServo!!.position = inStopClose
                    inRotationServo!!.position = inRotationPick
                }

                //SAMPLE4
                .lineToLinearHeading(Pose2d(-58.5, -46.5, Math.toRadians(118.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.3){
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = -1.0
                }
                .UNSTABLE_addTemporalMarkerOffset(0.0){verticalSlideTo(-30,0.5)
                    outRotationServo!!.position = outRotationCenter
                    intakeMotor!!.power = 1.0
                }
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(-1.0){
                    horizontalSlideTo(600,1.0)
                }
                //BASKET4
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.0){
                    horizontalSlideTo(0, 1.0)
                    inRotationServo!!.position = inRotationTransfer
                    inStopServo!!.position = inStopAutoOpen
                }
                .UNSTABLE_addTemporalMarkerOffset(1.5) {
                    outClawServo!!.position = outClawClose
                }
                .UNSTABLE_addTemporalMarkerOffset(1.7) {
                    verticalSlideTo(verticalSlideHigh,1.0)
                    outRotationServo!!.position = outRotationBackOut
                    inStopServo!!.position = inStopClose
                }

                    //END
                .splineTo(Vector2d(-24.5, -11.3), Math.toRadians(0.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.3){
                    outClawServo!!.position = outClawOpen
                    intakeMotor!!.power = 0.0
                }
                .UNSTABLE_addDisplacementMarkerOffset(4.0){
                    verticalSlideTo(1200,1.0)
                    outRotationServo!!.position = outRotationFront
                    intakeMotor!!.power = 0.0
                }
                .build()

        waitForStart()
        if (isStopRequested) {return}

        drive!!.followTrajectorySequence(all)

        drive!!.updatePoseEstimate()
        PoseStorage.currentPose = drive!!.poseEstimate
    }
}