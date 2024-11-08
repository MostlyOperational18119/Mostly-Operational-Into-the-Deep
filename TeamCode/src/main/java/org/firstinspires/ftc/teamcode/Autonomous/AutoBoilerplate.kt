@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.ThreadPool
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale
import java.util.concurrent.ExecutorService

enum class AutoActionType {
    Drive,
    Function,
    ActionOnDevice
}

data class DeviceActionTarget(val target: Double, val deviceName: String, val deviceType: Class<Any>)

@Suppress("Unused")
class AutoAction {
    val autoActionType: AutoActionType
    val eta: Double
    val trajectorySequence: TrajectorySequence?
    val callbackFunction: ((HardwareMap) -> Unit)?
    val deviceActionTarget: DeviceActionTarget?

    // DRIVE
    constructor(constructorTrajectorySequence: TrajectorySequence) {
        autoActionType = AutoActionType.Drive
        eta = constructorTrajectorySequence.duration()
        trajectorySequence = constructorTrajectorySequence
        callbackFunction = null
        deviceActionTarget = null
    }

    // FUNCTION
    constructor(duration: Double, callback: (HardwareMap) -> Unit) {
        autoActionType = AutoActionType.Function
        eta = duration
        trajectorySequence = null
        callbackFunction = callback
        deviceActionTarget = null
    }

    // ActionOnDevice
    constructor(target: DeviceActionTarget, duration: Double) {
        autoActionType = AutoActionType.ActionOnDevice
        eta = duration
        trajectorySequence = null
        callbackFunction = null
        deviceActionTarget = target
    }
}

abstract class AutoBoilerplate: DriveMethods() {
    private fun spawnFunctionThread(threadPool: ExecutorService, callback: (HardwareMap) -> (Unit), hardwareMap: HardwareMap) {
        val runnable = Runnable {
            callback(hardwareMap)
        }

        threadPool.submit(runnable)
    }

    private fun handleAction(action: AutoAction, hardwareMap: HardwareMap, drive: SampleMecanumDrive, threadPool: ExecutorService): AutoActionType {
        when (action.autoActionType) {
            AutoActionType.Drive -> {
                drive.followTrajectorySequenceAsync(action.trajectorySequence)
            }
            AutoActionType.Function -> {
                spawnFunctionThread(threadPool, action.callbackFunction!!, hardwareMap)
            }
            AutoActionType.ActionOnDevice -> {
                val deviceActionTarget = action.deviceActionTarget!!
                val device = hardwareMap.get(deviceActionTarget.deviceName)

                when (deviceActionTarget.deviceType) {
                    is DcMotor -> {
                        (device as DcMotor).power = deviceActionTarget.target
                    }
                    is Servo -> {
                        (device as Servo).position = deviceActionTarget.target
                    }
                    else -> {
                        println("What the heck, what's a ${deviceActionTarget.deviceType.name}")
                    }
                }
            }
        }

        return action.autoActionType
    }

    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)

        val autoActions = getAutoTrajectorySequences(drive, startPose)
        var totalDuration = 0.0
        var actionNumber = 0
        val threadPool = ThreadPool.newFixedThreadPool(1, "AutoBoilerplate.runOpMode().threadPool")
        autoActions.forEach { totalDuration += it.eta }
        telemetry.addLine("Init complete, got autoActions")
        telemetry.update()

        val timer = ElapsedTime()
        val actionTimer = ElapsedTime()

        // It better not be empty
        assert(autoActions.isNotEmpty())

        handleAction(autoActions[0], hardwareMap, drive, threadPool)
        timer.reset()
        actionTimer.reset()

        while (opModeIsActive() && !isStopRequested && (drive.isBusy || actionNumber < (autoActions.size-1))) {
            drive.update()

            if (!drive.isBusy) {
                actionNumber++
                handleAction(autoActions[actionNumber], hardwareMap, drive, threadPool)
                actionTimer.reset()
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
                    "Auto action %d of %d, (%.2f% complete)",
                    actionNumber+1,
                    autoActions.size,
                    actionTimer.seconds() / autoActions[actionNumber].eta
                )
            )

            telemetry.update()
        }
    }

    abstract val startPose: Pose2d
    abstract fun getAutoTrajectorySequences(drive: SampleMecanumDrive, startPose2d: Pose2d): ArrayList<AutoAction>
}