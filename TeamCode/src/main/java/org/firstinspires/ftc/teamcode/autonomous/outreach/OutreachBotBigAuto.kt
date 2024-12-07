package org.firstinspires.ftc.teamcode.autonomous.outreach

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters
import org.firstinspires.ftc.teamcode.teleop.outreach.OutreachBotBig

@Autonomous(name = "OutreachBotBigAuto")
class OutreachBotBigAuto : LinearOpMode() {
    override fun runOpMode() {
        // Motors
        val motorFL = hardwareMap.dcMotor["motorFL"]
        val motorFR = hardwareMap.dcMotor["motorFR"]
        val motorBL = hardwareMap.dcMotor["motorBL"]
        val motorBR = hardwareMap.dcMotor["motorBR"]

        val launcherRotateMotor = hardwareMap.dcMotor["launcherRotateMotor"]

        val imu = hardwareMap.get(IMU::class.java, "imu")

        val imuParameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        )

        imu.initialize(imuParameters)

        // Set motor directions and mode
        motorFR.direction = DcMotorSimple.Direction.REVERSE
        motorBR.direction = DcMotorSimple.Direction.REVERSE

        launcherRotateMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Servos
        val launcherServo = hardwareMap.servo["launcherServo"]

        telemetry.addLine("Init finished :D")
        telemetry.update()

        waitForStart()

        val robotRotation = imu.robotYawPitchRollAngles

        // Start of Auto

        // Forward for 3 seconds
        motorFL.power = 0.5
        motorFR.power = 0.5
        motorBL.power = 0.5
        motorBR.power = 0.5

        sleep(DriveForwardTime)

        // Back to original positions, then back 3 seconds
        motorFL.power = -0.5
        motorFR.power = -0.5
        motorBL.power = -0.5
        motorBR.power = -0.5

        sleep(DriveForwardTime * 2)

        // Back to original position

        motorFL.power = 0.5
        motorFR.power = 0.5
        motorBL.power = 0.5
        motorBR.power = 0.5

        sleep(DriveForwardTime)

        // Spin in a circle :)
        motorFL.power = -0.5
        motorFR.power = -0.5
        motorBL.power = 0.5
        motorBR.power = 0.5

        sleep(10000)

        // Stop moving, so I don't accidentally gouge out the eyes of a sick child with a paper airplane
        motorFL.power = 0.0
        motorFR.power = 0.0
        motorBL.power = 0.0
        motorBR.power = 0.0

        telemetry.addLine("Going to launch the airplane in 5 seconds")

        sleep(5000)

        // Launch the airplane, hope no one is hurt (it would be such a shame)
        launcherServo.position = OutreachBotBig.LauncherServoLaunchPosition
        sleep(1000)
        launcherServo.position = OutreachBotBig.LauncherServoReadyPosition

        // End of Auto
    }

    companion object {
        const val DriveForwardTime: Long = 3000
    }
}