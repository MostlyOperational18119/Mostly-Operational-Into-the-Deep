package org.firstinspires.ftc.teamcode.drive

import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.autonomous.goBuilda.GoBildaPinpointDriver
import kotlin.math.PI

@Config
class GoBildaPinpointMecanumLocalizer(hardwareMap: HardwareMap) : Localizer {
    private val pinpointDriver = hardwareMap.get(GoBildaPinpointDriver::class.java, "odo")

    init {
        pinpointDriver.setOffsets(-84.0, -168.0)

        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

        pinpointDriver.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.REVERSED, // MAYBE TRY REVERSED AGAIN
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        )

        pinpointDriver.resetPosAndIMU()
    }

    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            pinpointDriver.setPosition(Pose2D(DistanceUnit.INCH, value.x, value.y, AngleUnit.RADIANS, value.heading))
            _poseEstimate = value
        }
    private var _poseVelocity = Pose2d()
    override val poseVelocity: Pose2d
        get() = _poseVelocity

    override fun update() {
        pinpointDriver.update()

        Log.i("GoBildaPinpointMecanumLocalizer", "Position Pose2D ${pinpointDriver.getPosition()}")

        val pos = pinpointDriver.getPosition()

        val posX = pos.getX(DistanceUnit.INCH)
        val posY = pos.getY(DistanceUnit.INCH)
        val posHeading = pos.getHeading(AngleUnit.RADIANS)

        _poseEstimate = Pose2d(posX, posY, posHeading)

        val vel = pinpointDriver.getVelocity()

        val velX = vel.getX(DistanceUnit.INCH)
        val velY = vel.getY(DistanceUnit.INCH)
        val velHeading = vel.getHeading(AngleUnit.RADIANS)

        _poseVelocity = Pose2d(velX, velY, velHeading)
    }
}
