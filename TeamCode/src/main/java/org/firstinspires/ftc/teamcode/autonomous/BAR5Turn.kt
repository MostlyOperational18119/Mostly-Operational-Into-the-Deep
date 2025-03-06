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

        drive!!.poseEstimate = Pose2d(14.5, -63.2, Math.toRadians(-90.00))


        val all: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(14.5, -63.2, Math.toRadians(-90.00)))
                    //BAR0
                .addTemporalMarker{
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .lineToConstantHeading(Vector2d(-4.5,-30.4))
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outRotationServo!!.position = outRotationBackPlace}
                .UNSTABLE_addTemporalMarkerOffset(0.05){ outClawServo!!.position = outClawOpenAuto}
                .waitSeconds(0.07)

                    //SAMPLE1
                .UNSTABLE_addTemporalMarkerOffset(1.0){horizontalSlideTo(600,1.0)}
                .splineTo(Vector2d(30.75, -32.0), Math.toRadians(42.00))
                .UNSTABLE_addTemporalMarkerOffset(-0.05){inRotationServo!!.position = inRotationPick}
                .UNSTABLE_addTemporalMarkerOffset(0.0){verticalSlideTo(0,0.3)}
                .lineToLinearHeading(Pose2d(35.0, -51.5, Math.toRadians(-60.00)))
                .UNSTABLE_addTemporalMarkerOffset(0.0){inRotationServo!!.position = inRotationUpAuto}

                    //SAMPLE2
                .lineToLinearHeading(Pose2d(39.75, -32.0, Math.toRadians(42.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.05){inRotationServo!!.position = inRotationPick}
                .lineToLinearHeading(Pose2d(44.0, -51.5, Math.toRadians(-60.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.05){inRotationServo!!.position = inRotationUpAuto}
                .UNSTABLE_addTemporalMarkerOffset(-0.05){horizontalSlideTo(50,1.0)}

                    //SAMPLE 3
                .lineToLinearHeading(Pose2d(53.5, -25.0, Math.toRadians(30.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.25){horizontalSlideTo(350,1.0)}
                .UNSTABLE_addTemporalMarkerOffset(-0.02){inRotationServo!!.position = inRotationPick}
                .UNSTABLE_addTemporalMarkerOffset(0.0){
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .lineToLinearHeading(Pose2d(51.0, -60.20, Math.toRadians(-40.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5){inRotationServo!!.position = inRotationUp}
                .UNSTABLE_addTemporalMarkerOffset(-0.5){horizontalSlideTo(0, 1.0)}

                .resetConstraints()
                .splineToLinearHeading(Pose2d(47.44, -59.0, Math.toRadians(90.00)), Math.toRadians(-90.0))
                .lineToConstantHeading(Vector2d(47.44, -63.2))

                //PICK 1
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 1
                .lineToConstantHeading(Vector2d(-3.0, -30.4))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outRotationServo!!.position = outRotationFrontPlace}
                .UNSTABLE_addTemporalMarkerOffset(0.05){ outClawServo!!.position = outClawOpenAuto}
                .waitSeconds(0.07)
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }

                //PICK 2
                .lineToConstantHeading(Vector2d(47.44, -63.2))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 2
                .lineToConstantHeading(Vector2d(-1.5, -30.4))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outRotationServo!!.position = outRotationFrontPlace}
                .UNSTABLE_addTemporalMarkerOffset(0.05){ outClawServo!!.position = outClawOpenAuto}
                .waitSeconds(0.07)
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }

                //PICK 3
                .lineToConstantHeading(Vector2d(47.44, -63.2))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 3
                .lineToConstantHeading(Vector2d(0.0, -30.4))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outRotationServo!!.position = outRotationFrontPlace}
                .UNSTABLE_addTemporalMarkerOffset(0.05){ outClawServo!!.position = outClawOpenAuto}
                .waitSeconds(0.07)
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }

                //PICK 4
                .lineToConstantHeading(Vector2d(47.44, -63.2))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 4
                .lineToConstantHeading(Vector2d(1.5, -30.4))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outRotationServo!!.position = outRotationFrontPlace}
                .UNSTABLE_addTemporalMarkerOffset(0.05){ outClawServo!!.position = outClawOpenAuto}
                .UNSTABLE_addTemporalMarkerOffset(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .waitSeconds(0.07)
                .resetConstraints()

                .build()

        waitForStart()
        if (isStopRequested) {return}

        drive!!.followTrajectorySequence(all)
        drive!!.updatePoseEstimate()
    }
}