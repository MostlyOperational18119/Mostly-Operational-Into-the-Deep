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

        val fastConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
            Arrays.asList(
                TranslationalVelocityConstraint(80.0),
                AngularVelocityConstraint(4.0)
            )
        )
        val accelConstraint: TrajectoryAccelerationConstraint = ProfileAccelerationConstraint(90.0)


        val bar0: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(14.5, -63.2, Math.toRadians(-90.00)))
                    //BAR0
                .lineToConstantHeading(Vector2d(-4.5,-31.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outRotationServo!!.position = outRotationBackPlace}
                .build()

        val sample1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-4.5,-31.0, Math.toRadians(-90.00)))
                .splineTo(Vector2d(41.5, -40.0), Math.toRadians(70.00))
                .UNSTABLE_addTemporalMarkerOffset(-0.6){intakeMotor!!.power = 0.0}
                .UNSTABLE_addTemporalMarkerOffset(-0.25){horizontalSlideTo(350,0.7);intakeMotor!!.power = 0.4}
                .build()

        val sample2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(41.5,-40.0, Math.toRadians(70.00)))
                .lineToLinearHeading(Pose2d(36.0, -51.5, Math.toRadians(-60.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.1){intakeMotor!!.power = -0.5}

                    //SAMPLE2
                .UNSTABLE_addTemporalMarkerOffset(0.3){horizontalSlideTo(100,1.0)}
                .lineToLinearHeading(Pose2d(50.0, -40.0, Math.toRadians(70.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.6){intakeMotor!!.power = 0.0}
                .UNSTABLE_addTemporalMarkerOffset(-0.25){horizontalSlideTo(350,0.7); intakeMotor!!.power = 0.4}
                .build()

        val sample3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(50.0,-40.0, Math.toRadians(70.00)))
                .lineToLinearHeading(Pose2d(43.0, -51.5, Math.toRadians(-60.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.05){intakeMotor!!.power = -0.5}
                .UNSTABLE_addTemporalMarkerOffset(-0.05){horizontalSlideTo(50,1.0)}

                //SAMPLE 3
                .lineToLinearHeading(Pose2d(53.5, -25.0, Math.toRadians(30.00)))
                .UNSTABLE_addTemporalMarkerOffset(-1.0){
                    inRotationServo!!.position = inRotationUpAuto;
                    intakeMotor!!.power = 0.0;
                    verticalSlideTo(0, 1.0)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                .UNSTABLE_addTemporalMarkerOffset(-0.25){horizontalSlideTo(350,1.0)}
                .UNSTABLE_addTemporalMarkerOffset(-0.02){inRotationServo!!.position = inRotationPick}
                .UNSTABLE_addTemporalMarkerOffset(0.0){
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                    horizontalSlideTo(200,0.1)
                }
                .lineToLinearHeading(Pose2d(51.0, -57.0, Math.toRadians(-50.00)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5){inRotationServo!!.position = inRotationUp}
                .UNSTABLE_addTemporalMarkerOffset(-0.5){horizontalSlideTo(0, 1.0)}

                .UNSTABLE_addTemporalMarkerOffset(0.1){inRotationServo!!.position = inRotationUp; intakeMotor!!.power = 0.0}
                .UNSTABLE_addTemporalMarkerOffset(0.1){horizontalSlideTo(0, 1.0)}

                .splineToLinearHeading(Pose2d(47.44, -57.0, Math.toRadians(90.00)), Math.toRadians(-90.0))
                .splineToLinearHeading(Pose2d(47.44, -62.3, Math.toRadians(90.00)), Math.toRadians(-90.0))

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
                .build()

        val bar1: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-3.0, -30.4, Math.toRadians(90.00)))
                .addTemporalMarker(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }

                //PICK 2
                .lineToConstantHeading(Vector2d(47.44, -61.6))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }

                //BAR 2
                .lineToConstantHeading(Vector2d(-1.5, -30.0))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outRotationServo!!.position = outRotationFrontPlace}
                .build()

        val bar2: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(-1.5, -30.0, Math.toRadians(90.00)))
                .addTemporalMarker(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }

                //PICK 3
                .lineToConstantHeading(Vector2d(47.44, -61.9))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 3
                .lineToConstantHeading(Vector2d(0.0, -29.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outRotationServo!!.position = outRotationFrontPlace}
                .build()

        val bar3: TrajectorySequence =
            drive!!.trajectorySequenceBuilder(Pose2d(0.0, -29.8, Math.toRadians(90.00)))
                .addTemporalMarker(0.2) {
                    verticalSlideTo(0, 0.3)
                    outRotationServo!!.position = outRotationBackWall
                    outSwivelServo!!.position = outSwivelPerpBack
                }
                //PICK 4
                .lineToConstantHeading(Vector2d(47.44, -61.3))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outClawServo!!.position = outClawClose}
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .UNSTABLE_addTemporalMarkerOffset(0.05) {
                    verticalSlideTo(verticalSlideBar, 1.0)
                    outRotationServo!!.position = outRotationUp
                    outSwivelServo!!.position = outSwivelPerpFront
                }
                //BAR 4
                .lineToConstantHeading(Vector2d(1.5, -29.5))
                .UNSTABLE_addTemporalMarkerOffset(-0.01){outRotationServo!!.position = outRotationFrontPlace}
                .resetConstraints()

                .build()

        waitForStart()
        if (isStopRequested) {return}

        verticalSlideTo(verticalSlideBar, 1.0)
        outRotationServo!!.position = outRotationUp
        outSwivelServo!!.position = outSwivelPerpBack

        drive!!.followTrajectorySequence(bar0)
        drive!!.updatePoseEstimate()
        outClawServo!!.position = outClawOpenAuto

        horizontalSlideTo(100,1.0)
        inStopServo!!.position = inStopClose
        inRotationServo!!.position = inRotationPick;
        drive!!.followTrajectorySequence(sample1)
        drive!!.updatePoseEstimate()

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
            if (elapsedTime.time() > 0.6){ moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
        }

        drive!!.followTrajectorySequence(sample2)
        drive!!.updatePoseEstimate()

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
            if (elapsedTime.time() > 0.6){ moveOn = true; telemetry.addLine(colorSeen); telemetry.update() }
        }

        drive!!.followTrajectorySequence(sample3)
        drive!!.updatePoseEstimate()
        outClawServo!!.position = outClawOpenAuto

        drive!!.followTrajectorySequence(bar1)
        drive!!.updatePoseEstimate()
        outClawServo!!.position = outClawOpenAuto

        drive!!.followTrajectorySequence(bar2)
        drive!!.updatePoseEstimate()
        outClawServo!!.position = outClawOpenAuto

        drive!!.followTrajectorySequence(bar3)
        drive!!.updatePoseEstimate()
        outClawServo!!.position = outClawOpenAuto
    }
}