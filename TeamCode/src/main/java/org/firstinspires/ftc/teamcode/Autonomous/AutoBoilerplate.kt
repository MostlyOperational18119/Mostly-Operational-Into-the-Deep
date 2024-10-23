@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.ThreadPool
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale
import java.util.concurrent.ExecutorService

enum class AutoActionType {
    Drive,
    Function
}

class AutoAction {
    val autoActionType: AutoActionType
    val eta: Double
    val trajectorySequence: TrajectorySequence?
    val callbackFunction: ((HardwareMap) -> Unit)?

    constructor(constructorTrajectorySequence: TrajectorySequence) {
        autoActionType = AutoActionType.Drive
        eta = constructorTrajectorySequence.duration()
        trajectorySequence = constructorTrajectorySequence
        callbackFunction = null
    }

    constructor(duration: Double, callback: (HardwareMap) -> Unit) {
        autoActionType = AutoActionType.Function
        eta = duration
        trajectorySequence = null
        callbackFunction = callback
    }
}

abstract class AutoBoilerplate: DriveMethods() {
    private fun spawnFunctionThread(threadPool: ExecutorService, callback: (HardwareMap) -> (Unit)) {
        TODO("Actually spawn the thread, haven't done that yet :(")
    }

    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)

        val trajectorySequences = getAutoTrajectorySequences(drive, startPose)
        var totalDuration = 0.0
        var sequenceNumber = 0
        val threadPool = ThreadPool.newFixedThreadPool(1, "AutoBoilerplateMulti.runOpMode().threadPool")
        trajectorySequences.forEach { totalDuration += it.eta }
        telemetry.addLine("Init complete, got trajectorySequence")
        telemetry.update()

        val timer = ElapsedTime()
        val sequenceTimer = ElapsedTime()

        // It better not be empty
        assert(trajectorySequences.isNotEmpty())

        {
            val autoAction = trajectorySequences[0]
            if (autoAction.autoActionType == AutoActionType.Drive) {
                drive.followTrajectorySequenceAsync(autoAction.trajectorySequence)
            } else {
                spawnFunctionThread(threadPool, autoAction.callbackFunction!!)
            }
        }
        timer.reset()
        sequenceTimer.reset()

        while (opModeIsActive() && !isStopRequested && (drive.isBusy || sequenceNumber < (trajectorySequences.size-1))) {
            drive.update()

            if (!drive.isBusy) {
                sequenceNumber++
                val autoAction = trajectorySequences[sequenceNumber]
                if (autoAction.autoActionType == AutoActionType.Drive) {
                    drive.followTrajectorySequenceAsync(autoAction.trajectorySequence)
                } else {
                    spawnFunctionThread(threadPool, autoAction.callbackFunction!!)
                }
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
                    sequenceTimer.seconds() / trajectorySequences[sequenceNumber].eta
                )
            )

            telemetry.update()
        }
    }

    abstract val startPose: Pose2d
    abstract fun getAutoTrajectorySequences(drive: SampleMecanumDrive, startPose2d: Pose2d): ArrayList<AutoAction>
}