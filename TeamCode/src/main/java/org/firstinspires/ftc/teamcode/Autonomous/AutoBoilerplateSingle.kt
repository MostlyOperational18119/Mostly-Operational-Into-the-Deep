@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale

abstract class AutoBoilerplateSingle: DriveMethods() {
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)

        val trajectorySequence = getAutoTrajectorySequence(drive, startPose)

        telemetry.addLine("Init complete, got trajectorySequence")
        telemetry.update()

        val timer = ElapsedTime()

        drive.followTrajectorySequenceAsync(trajectorySequence)
        timer.reset()

        while (opModeIsActive() && !isStopRequested && drive.isBusy) {
            drive.update()

            val poseEstimate = drive.poseEstimate

            telemetry.addData("Current position:",
                String.format(
                    Locale.ENGLISH,
                    "X: %d, Y: %d, Heading: %d degrees",
                    poseEstimate.x,
                    poseEstimate.y,
                    poseEstimate.heading
                )
            )

            telemetry.addLine(String.format(Locale.ENGLISH, "Auto is %.2f% complete (%d of %d seconds)", (timer.seconds()/trajectorySequence.duration())*100, timer.seconds(), trajectorySequence.duration()))
            telemetry.update()
        }
    }
    abstract val startPose: Pose2d
    abstract fun getAutoTrajectorySequence(drive: SampleMecanumDrive, startPose2d: Pose2d): TrajectorySequence
}