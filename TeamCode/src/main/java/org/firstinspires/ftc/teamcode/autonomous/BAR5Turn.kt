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

        val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            Arrays.asList(
                TranslationalVelocityConstraint(30.0),
                AngularVelocityConstraint(2.0)
            )
        )

        val fastConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            Arrays.asList(
                TranslationalVelocityConstraint(80.0),
                AngularVelocityConstraint(4.0)
            )
        )

        val accelConstraint: TrajectoryAccelerationConstraint = ProfileAccelerationConstraint(90.0)

        val fastestAccel: TrajectoryAccelerationConstraint = ProfileAccelerationConstraint(105.0)


        val all: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(14.5, -63.19, Math.toRadians(-90.00)))
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                    //BAR0
                .addTemporalMarker{
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .setReversed(true)
                .lineToConstantHeading(Vector2d(-4.5,-30.5))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outRotationServo!!.position = outRotationBackPlace}
                .UNSTABLE_addTemporalMarkerOffset(0.05){ outClawServo!!.position = outClawOpen}

                .setAccelConstraint(fastestAccel)

                    //SAMPLE1
                .UNSTABLE_addTemporalMarkerOffset(1.0){horizontalSlideTo(600,1.0)}
                .splineTo(Vector2d(31.05, -34.0), Math.toRadians(42.00))
                .UNSTABLE_addTemporalMarkerOffset(-0.1){inRotationServo!!.position = inRotationPick}
                .UNSTABLE_addTemporalMarkerOffset(0.0){verticalSlideTo(0,0.3)}
                .lineToLinearHeading(Pose2d(38.03, -47.07, Math.toRadians(-50.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.0){inRotationServo!!.position = inRotationUpAuto}

                    //SAMPLE2
                .lineToLinearHeading(Pose2d(38.26, -30.5, Math.toRadians(37.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.1){inRotationServo!!.position = inRotationPick}
                .lineToLinearHeading(Pose2d(44.71, -47.68, Math.toRadians(-50.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.05){inRotationServo!!.position = inRotationUpAuto}
                .UNSTABLE_addTemporalMarkerOffset(0.0){horizontalSlideTo(600,1.0)}

                    //SAMPLE 3
                .lineToLinearHeading(Pose2d(49.0, -29.0, Math.toRadians(37.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.1){inRotationServo!!.position = inRotationPick}
                .UNSTABLE_addTemporalMarkerOffset(0.0){
                    horizontalSlideTo(570, 0.1)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .lineToLinearHeading(Pose2d(48.44, -47.20, Math.toRadians(-55.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5){inRotationServo!!.position = inRotationUpAuto}
                .UNSTABLE_addTemporalMarkerOffset(-0.5){horizontalSlideTo(0, 1.0)}

                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .splineToLinearHeading(Pose2d(47.44, -63.0, Math.toRadians(90.00)), Math.toRadians(-90.0))

                .setVelConstraint(fastConstraint)

                //PICK 1
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 1
                .lineToConstantHeading(Vector2d(-3.25, -30.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.01) { outRotationServo!!.position = outRotationFrontPlace }
                .UNSTABLE_addTemporalMarkerOffset(0.05) { outClawServo!!.position = outClawOpen}
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
                .lineToConstantHeading(Vector2d(-2.0, -30.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.01) { outRotationServo!!.position = outRotationFrontPlace }
                .UNSTABLE_addTemporalMarkerOffset(0.05) { outClawServo!!.position = outClawOpen}
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
                .lineToConstantHeading(Vector2d(-0.75, -30.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.01) { outRotationServo!!.position = outRotationFrontPlace }
                .UNSTABLE_addTemporalMarkerOffset(0.05) { outClawServo!!.position = outClawOpen}
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
                .lineToConstantHeading(Vector2d(0.5, -30.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.01) { outRotationServo!!.position = outRotationFrontPlace }
                .UNSTABLE_addTemporalMarkerOffset(0.05) { outClawServo!!.position = outClawOpen}
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
                .lineToConstantHeading(Vector2d(1.75, -30.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.01) { outRotationServo!!.position = outRotationFrontPlace }
                .UNSTABLE_addTemporalMarkerOffset(0.05) { outClawServo!!.position = outClawOpen}
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