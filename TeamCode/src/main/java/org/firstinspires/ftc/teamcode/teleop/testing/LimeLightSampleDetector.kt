package org.firstinspires.ftc.teamcode.teleop.testing

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.DriveMethods
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

@TeleOp(name = "LimeLightSampleDetector")
@Disabled
class LimeLightSampleDetector : DriveMethods() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val frontLeftMotor = hardwareMap.dcMotor["motorFL"]
        val backLeftMotor = hardwareMap.dcMotor["motorBL"]
        val frontRightMotor = hardwareMap.dcMotor["motorFR"]
        val backRightMotor = hardwareMap.dcMotor["motorBR"]

        val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        val pipelineIndex = 0

        frontRightMotor.direction = DcMotorSimple.Direction.REVERSE
        backRightMotor.direction = DcMotorSimple.Direction.REVERSE

        // Retrieve the IMU from the hardware map
        val imu = hardwareMap.get(IMU::class.java, "imuC")
        // Adjust the orientation parameters to match your robot
        val parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        )
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters)

        waitForStart()

        if (isStopRequested) return

        limelight.pipelineSwitch(pipelineIndex)
        limelight.start()

        while (opModeIsActive()) {
            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick value is reversed
            val x = gamepad1.left_stick_x.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw()
            }

            val botHeading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

            // Rotate the movement direction counter to the bot's rotation
            var rotX = x * cos(-botHeading) - y * sin(-botHeading)
            val rotY = x * sin(-botHeading) + y * cos(-botHeading)

            rotX *= 1.1 // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
            val frontLeftPower = (rotY + rotX + rx) / denominator
            val backLeftPower = (rotY - rotX + rx) / denominator
            val frontRightPower = (rotY - rotX - rx) / denominator
            val backRightPower = (rotY + rotX - rx) / denominator

            frontLeftMotor.power = frontLeftPower
            backLeftMotor.power = backLeftPower
            frontRightMotor.power = frontRightPower
            backRightMotor.power = backRightPower

            val llResult = limelight.latestResult

            if (llResult != null && llResult.isValid) {
                val llColorResults = llResult.colorResults
                if (llColorResults.isNotEmpty()) {
                    gamepad1.rumble(100)
                    telemetry.addData("Limelight color results", llColorResults)
                } else {
                    telemetry.addLine("No limelight color results :(")
                }
            }

            telemetry.update()
        }

        // OpMode's done, bye Limelight
        limelight.stop()
    }
}