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
            GoBildaPinpointDriver.EncoderDirection.REVERSED, // HAVEN'T TESTED YET, BUT SHOULD FIX ROBOT
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

    fun degToRadian(deg: Double): Double {
        val result = deg / 180.0 * PI
        Log.i("degToRadian", "Called with degrees of $deg, returning $result (would be ${Math.toRadians(deg)}")
        return result
    }

    fun radianToDeg(rad: Double): Double {
        return rad * 180.0 / PI
    }

    fun mmToInches(mm: Double): Double {
        return mm / 25.4
    }

    fun inchesToMM(inches: Double): Double {
        return inches * 25.4
    }

    override fun update() {
        pinpointDriver.update()

        Log.i("GoBildaPinpointMecanumLocalizer", "Position Pose2D ${pinpointDriver.getPosition()}")

        val posX = mmToInches(pinpointDriver.getPosX())
        val posY = mmToInches(pinpointDriver.getPosY())
        val posHeading = pinpointDriver.getHeading()

        _poseEstimate = Pose2d(posX, posY, posHeading)

        val velX = mmToInches(pinpointDriver.getVelX())
        val velY = mmToInches(pinpointDriver.getVelY())
        val velHeading = pinpointDriver.getHeadingVelocity()

        _poseVelocity = Pose2d(velX, velY, velHeading)
    }
}
