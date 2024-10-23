@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale

abstract class AutoBoilerplateMulti: DriveMethods() {
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)

        val trajectorySequences = getAutoTrajectorySequences(drive, startPose)
        var totalDuration = 0.0
        var sequenceNumber = 0
        trajectorySequences.forEach { totalDuration += it.duration() }
        telemetry.addLine("Init complete, got trajectorySequence")
        telemetry.update()

        val timer = ElapsedTime()
        val sequenceTimer = ElapsedTime()

        // It better not be empty
        assert(trajectorySequences.isNotEmpty())

        drive.followTrajectorySequenceAsync(trajectorySequences[0])
        timer.reset()
        sequenceTimer.reset()

        while (opModeIsActive() && !isStopRequested && (drive.isBusy || sequenceNumber < (trajectorySequences.size-1))) {
            drive.update()

            if (!drive.isBusy) {
                sequenceNumber++
                drive.followTrajectorySequenceAsync(trajectorySequences[sequenceNumber])
                sequenceTimer.reset()
            }

            val poseEstimate = drive.poseEstimate
            val curTime = timer.seconds()

            telemetry.addLine(
                String.format(
                    Locale.ENGLISH,
                    "Current position: X: %d, Y: %d, Heading: %d degrees",
                    poseEstimate.x,
                    poseEstimate.y,
                    poseEstimate.heading
                )
            )

            telemetry.addLine(
                String.format(
                    Locale.ENGLISH,
                    "Auto is %.2f% complete (%d of %d seconds)",
                    (curTime/totalDuration)*100,
                    curTime, totalDuration
                )
            )

            telemetry.addLine(
                String.format(
                    Locale.ENGLISH,
                    "Trajectory sequence %d of %d, (%.2f% complete)",
                    sequenceNumber+1,
                    trajectorySequences.size,
                    sequenceTimer.seconds() / trajectorySequences[sequenceNumber].duration()
                )
            )

            telemetry.update()
        }
    }

    abstract val startPose: Pose2d
    abstract fun getAutoTrajectorySequences(drive: SampleMecanumDrive, startPose2d: Pose2d): ArrayList<TrajectorySequence>
}