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
        initOdometry()
        initMotors()
        initServosAndSensorsNoSet()
        transferServo!!.position = transferServoOpen
        outClawServo!!.position = outClawClose
        outRotationServo!!.position = outRotationBackOut + 0.04
        outSwivelServo!!.position = outSwivelPerpBack
        inStopServo!!.position = inStopOpen
        inRotationServo!!.position = inRotationUp

        drive!!.poseEstimate = Pose2d(14.5, -63.19, Math.toRadians(-90.00))


        val all: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(14.5, -63.2, Math.toRadians(-90.00)))
                    //BAR0
                .addTemporalMarker{
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .setReversed(true)
                .lineToConstantHeading(Vector2d(-4.5,-31.5))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.0){outRotationServo!!.position = outRotationBackPlace}
                .UNSTABLE_addTemporalMarkerOffset(0.1){ outClawServo!!.position = outClawOpenAuto}
                    .waitSeconds(0.05)

                    //SAMPLE1
                .UNSTABLE_addTemporalMarkerOffset(1.0){horizontalSlideTo(600,1.0)}
                .splineTo(Vector2d(31.05, -32.0), Math.toRadians(42.00))
                .UNSTABLE_addTemporalMarkerOffset(-0.05){inRotationServo!!.position = inRotationPick}
                .UNSTABLE_addTemporalMarkerOffset(0.0){verticalSlideTo(0,0.3)}
                .lineToLinearHeading(Pose2d(38.03, -47.07, Math.toRadians(-70.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.0){inRotationServo!!.position = inRotationUpAuto}

                    //SAMPLE2
                .lineToLinearHeading(Pose2d(40.26, -32.0, Math.toRadians(50.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.05){inRotationServo!!.position = inRotationPick}
                .lineToLinearHeading(Pose2d(47.71, -50.68, Math.toRadians(-60.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.05){inRotationServo!!.position = inRotationUpAuto}
                .UNSTABLE_addTemporalMarkerOffset(0.0){horizontalSlideTo(600,0.5)}

                    //SAMPLE 3
                .lineToLinearHeading(Pose2d(52.5, -30.0, Math.toRadians(50.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.05){inRotationServo!!.position = inRotationPick}
                .UNSTABLE_addTemporalMarkerOffset(0.0){
                    horizontalSlideTo(550, 0.07)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .lineToLinearHeading(Pose2d(48.0, -55.20, Math.toRadians(-60.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5){inRotationServo!!.position = inRotationUpAuto}
                .UNSTABLE_addTemporalMarkerOffset(-0.5){horizontalSlideTo(0, 1.0)}

                .resetConstraints()
                .splineToLinearHeading(Pose2d(47.44, -59.0, Math.toRadians(90.00)), Math.toRadians(-90.0))
                .splineToLinearHeading(Pose2d(47.44, -63.0, Math.toRadians(90.00)), Math.toRadians(-90.0))

                //PICK 1
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 1
                .lineToConstantHeading(Vector2d(-3.25, -31.5))
                .UNSTABLE_addTemporalMarkerOffset(0.0){outRotationServo!!.position = outRotationFrontPlace}
            .UNSTABLE_addTemporalMarkerOffset(0.1){ outClawServo!!.position = outClawOpenAuto}
            .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }

                //PICK 2
                .setReversed(true)
                .splineToConstantHeading(Vector2d(47.44, -63.0), Math.toRadians(-90.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 2
                .lineToConstantHeading(Vector2d(-2.0, -31.5))
                .UNSTABLE_addTemporalMarkerOffset(0.0){outRotationServo!!.position = outRotationFrontPlace}
                .UNSTABLE_addTemporalMarkerOffset(0.1){ outClawServo!!.position = outClawOpenAuto}
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }

                //PICK 3
                .setReversed(true)
                .splineToConstantHeading(Vector2d(47.44, -63.0), Math.toRadians(-90.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 3
                .lineToConstantHeading(Vector2d(-0.75, -31.5))
                .UNSTABLE_addTemporalMarkerOffset(0.0){outRotationServo!!.position = outRotationFrontPlace}
                .UNSTABLE_addTemporalMarkerOffset(0.1){ outClawServo!!.position = outClawOpenAuto}
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }

                //PICK 4
                .setReversed(true)
                .splineToConstantHeading(Vector2d(47.44, -63.0), Math.toRadians(-90.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 4
                .lineToConstantHeading(Vector2d(0.5, -31.5))
                .UNSTABLE_addTemporalMarkerOffset(0.0){outRotationServo!!.position = outRotationFrontPlace}
                .UNSTABLE_addTemporalMarkerOffset(0.1){ outClawServo!!.position = outClawOpenAuto}
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }

                //PICK 5
                .setReversed(true)
                .splineToConstantHeading(Vector2d(47.44, -63.0), Math.toRadians(-90.00))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 5
                .lineToConstantHeading(Vector2d(1.75, -31.5))
                .UNSTABLE_addTemporalMarkerOffset(0.0){outRotationServo!!.position = outRotationFrontPlace}
                .UNSTABLE_addTemporalMarkerOffset(0.1){ outClawServo!!.position = outClawOpenAuto}
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }

                .resetConstraints()

                .build()

        waitForStart()
        if (isStopRequested) {return}

        drive!!.followTrajectorySequence(all)
        drive!!.updatePoseEstimate()
    }
}