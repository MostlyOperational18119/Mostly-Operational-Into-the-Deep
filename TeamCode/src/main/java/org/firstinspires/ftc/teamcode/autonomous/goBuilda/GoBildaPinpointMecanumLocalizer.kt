package org.firstinspires.ftc.teamcode.autonomous.goBuilda

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.PI

class GoBildaPinpointMecanumLocalizer(hardwareMap: HardwareMap) : Localizer {
    private val pinpointDriver = hardwareMap.get(GoBildaPinpointDriver::class.java, "odo")

    init {
        pinpointDriver.setOffsets(-84.0, -168.0)

        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

        pinpointDriver.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        )

        pinpointDriver.resetPosAndIMU()
    }

    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) { _poseEstimate = value }
    private var _poseVelocity = Pose2d()
    override val poseVelocity: Pose2d
        get() = _poseVelocity

    fun degToRadian(deg: Double): Double {
        return deg / 180.0 * PI
    }

    fun mmToInches(mm: Double): Double {
        return mm / 25.4
    }

    override fun update() {
        pinpointDriver.update()

        val posX = mmToInches(pinpointDriver.getPosX())
        val posY = mmToInches(pinpointDriver.getPosY())
        val posHeading = degToRadian(pinpointDriver.getHeading())

        _poseEstimate = Pose2d(posX, posY, posHeading)

        val velX = mmToInches(pinpointDriver.getVelX())
        val velY = mmToInches(pinpointDriver.getVelY())
        val velHeading = mmToInches(pinpointDriver.getHeadingVelocity())

        _poseVelocity = Pose2d(velX, velY, velHeading)
    }
}
