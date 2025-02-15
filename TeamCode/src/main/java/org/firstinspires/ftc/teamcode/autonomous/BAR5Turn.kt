package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Arrays


@Autonomous(name = "BAR5Turn", group = "AAAA")

class BAR5Turn : Methods() {
    override fun runOpMode() {
        drive = SampleMecanumDriveCancelable(hardwareMap)
        initMotors()

        transferServo = hardwareMap.servo["Transfer"]
        transferServo!!.position = transferServoOpen
        outClawServo = hardwareMap.servo["OutClaw"]
        outClawServo!!.position = outClawClose
        outRotationServo = hardwareMap.servo["OutRotation"]
        outRotationServo!!.position = outRotationCenter
        outSwivelServo = hardwareMap.servo["OutSwivel"]
        inStopServo = hardwareMap.servo["InStop"]
        inStopServo!!.position = inStopClose
        inRotationServo = hardwareMap.servo["InRotation"]
        inRotationServo!!.position = 1.0

        drive!!.poseEstimate = Pose2d(14.5, -63.19, Math.toRadians(-90.00))

        val accelConstraint: TrajectoryAccelerationConstraint = ProfileAccelerationConstraint(50.0)
        val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            Arrays.asList(
                TranslationalVelocityConstraint(20.0),
                AngularVelocityConstraint(1.0)
            )
        )

        val all: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(14.5, -63.19, Math.toRadians(-90.00)))
                    //BAR0
                .addTemporalMarker{
                    verticalSlideTo(1550, 1.0)
                    outRotationServo!!.position = outRotationBack
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .lineToConstantHeading(Vector2d(-6.0,-32.0))

                .waitSeconds(0.01)

                    //SAMPLE1
                .UNSTABLE_addTemporalMarkerOffset(0.5){horizontalSlideTo(600,1.0)}
                .splineTo(Vector2d(36.05, -29.0), Math.toRadians(40.00))
                .UNSTABLE_addTemporalMarkerOffset(-0.3){inRotationServo!!.position = inRotationPick}
                .UNSTABLE_addTemporalMarkerOffset(0.0){verticalSlideTo(0,0.3)}
                .lineToLinearHeading(Pose2d(37.03, -46.07, Math.toRadians(-30.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.2){inRotationServo!!.position = inRotationUpAuto}

                    //SAMPLE2
                .lineToLinearHeading(Pose2d(46.26, -29.37, Math.toRadians(35.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.2){inRotationServo!!.position = inRotationPick}
                .lineToLinearHeading(Pose2d(43.71, -45.68, Math.toRadians(-35.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.2){inRotationServo!!.position = inRotationUpAuto}
                .UNSTABLE_addTemporalMarkerOffset(0.0){horizontalSlideTo(400,1.0)}

                    //SAMPLE 3
                .lineToLinearHeading(Pose2d(57.87, -29.76, Math.toRadians(35.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.2){inRotationServo!!.position = inRotationPick}
                .UNSTABLE_addTemporalMarkerOffset(0.0){horizontalSlideTo(300, 0.1)}
                .setVelConstraint(slowConstraint)
                .setReversed(true)
                .splineToLinearHeading(Pose2d(47.44, -60.20, Math.toRadians(-90.00)), Math.toRadians(-90.0))
                .setReversed(false)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(-1.0){inRotationServo!!.position = inRotationUpAuto}
                .UNSTABLE_addTemporalMarkerOffset(-1.0){horizontalSlideTo(0, 1.0)}

                    //PICK1
                .UNSTABLE_addTemporalMarkerOffset(-0.3){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(1550, 1.0)
                    outRotationServo!!.position = outRotationBack
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                    //BAR 1
                .setReversed(true)
                .splineToConstantHeading(Vector2d(-4.0, -33.5), Math.toRadians(90.0))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.5) { verticalSlideTo(750, 1.0) }
                .UNSTABLE_addTemporalMarkerOffset(0.0) { outClawServo!!.position = outClawOpen}
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationFront
                    outSwivelServo!!.position = outSwivelPerpFront
                }

                    //PICK2
                .splineToLinearHeading(Pose2d(47.44, -60.20,Math.toRadians(90.0)), Math.toRadians(-90.00))
                .UNSTABLE_addTemporalMarkerOffset(-0.3){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(1550, 1.0)
                    outRotationServo!!.position = outRotationBack
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                //BAR 2
                .splineToConstantHeading(Vector2d(-2.0, -33.5), Math.toRadians(90.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5) { verticalSlideTo(750, 1.0) }
                .UNSTABLE_addTemporalMarkerOffset(0.0) { outClawServo!!.position = outClawOpen}
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationFront
                    outSwivelServo!!.position = outSwivelPerpFront
                }

                //PICK3
                .setReversed(true)
                .splineToConstantHeading(Vector2d(47.44, -60.20), Math.toRadians(-90.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.3){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(1550, 1.0)
                    outRotationServo!!.position = outRotationBack
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                //BAR 3
                .splineToConstantHeading(Vector2d(0.0, -33.5), Math.toRadians(90.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5) { verticalSlideTo(750, 1.0) }
                .UNSTABLE_addTemporalMarkerOffset(0.0) { outClawServo!!.position = outClawOpen}
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationFront
                    outSwivelServo!!.position = outSwivelPerpFront
                }

                //PICK4
                .setReversed(true)
                .splineToConstantHeading(Vector2d(47.44, -60.20), Math.toRadians(-90.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.3){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(1550, 1.0)
                    outRotationServo!!.position = outRotationBack
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                //BAR 4
                .splineToConstantHeading(Vector2d(2.0, -33.5), Math.toRadians(90.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5) { verticalSlideTo(750, 1.0) }
                .UNSTABLE_addTemporalMarkerOffset(0.0) { outClawServo!!.position = outClawOpen}
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationCenter
                    outSwivelServo!!.position = outSwivelPerpFront
                }

                //PICK5
                .setReversed(true)
                .splineToConstantHeading(Vector2d(47.44, -60.20), Math.toRadians(-90.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.3){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.0) {
                    verticalSlideTo(1550, 1.0)
                    outRotationServo!!.position = outRotationBack
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                //BAR 5
                .splineToConstantHeading(Vector2d(4.0, -33.5), Math.toRadians(90.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5) { verticalSlideTo(750, 1.0) }
                .UNSTABLE_addTemporalMarkerOffset(0.0) { outClawServo!!.position = outClawOpen}
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationCenter
                    outSwivelServo!!.position = outSwivelPerpFront
                }

                .lineToConstantHeading(Vector2d(43.25, -55.55))
                .resetConstraints()

                .build()

        waitForStart()
        if (isStopRequested) {return}

        drive!!.followTrajectorySequence(all)

        drive!!.updatePoseEstimate()
        PoseStorage.currentPose = drive!!.poseEstimate
    }
}