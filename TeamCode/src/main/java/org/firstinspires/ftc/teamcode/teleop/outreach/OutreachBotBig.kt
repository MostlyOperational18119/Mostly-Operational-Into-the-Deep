package org.firstinspires.ftc.teamcode.teleop.outreach

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation

@TeleOp(name = "OutreachBotBig")
class OutreachBotBig : LinearOpMode() {
    override fun runOpMode() {
        // Motors
        val motorFL = hardwareMap.dcMotor["motorFL"]
        val motorFR = hardwareMap.dcMotor["motorFR"]
        val motorBL = hardwareMap.dcMotor["motorBL"]
        val motorBR = hardwareMap.dcMotor["motorBR"]

        val launcherRotateMotor = hardwareMap.dcMotor["launcherRotateMotor"]

        // Set motor directions and mode
        motorFR.direction = DcMotorSimple.Direction.REVERSE
        motorBR.direction = DcMotorSimple.Direction.REVERSE

        launcherRotateMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Servos
        val launcherServo = hardwareMap.servo["launcherServo"]
        launcherServo.position = LauncherServoReadyPosition

        // Gamepads
        var currentGamepad1 = Gamepad()
        var previousGamepad1 = Gamepad()

        // IMU
        val imu = hardwareMap.get(IMU::class.java, "imu")

        imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            )
        )

        // Orientation
        var orientation: Orientation

        // Other variables
        val speedDiv = 2.0
        var rotateRunCounter = 0
        var movementToggle = true // true - allow movement, false - no moving

        telemetry.addLine("Init done :)")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1)
            currentGamepad1.copy(gamepad1)

            val gamepad1LeftY = -currentGamepad1.left_stick_y
            val gamepadLeftX = currentGamepad1.left_stick_x
            val gamepad1RightX = currentGamepad1.right_stick_x

            orientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
            )

            // Launch servo
            if (currentGamepad1.a && !previousGamepad1.a) {
                launcherServo.position = LauncherServoLaunchPosition
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                launcherServo.position = LauncherServoReadyPosition
            }

            // Launcher rotation motor
            if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) && rotateRunCounter == 0) {
                launcherRotateMotor.power = -0.3
                rotateRunCounter = 3
            } else if ((currentGamepad1.dpad_down && !previousGamepad1.dpad_down) && rotateRunCounter == 0) {
                launcherRotateMotor.power = 0.3
                rotateRunCounter = 3
            }

            if (rotateRunCounter > 0) rotateRunCounter--
            else launcherRotateMotor.power = 0.0

            if ((currentGamepad1.left_bumper && !previousGamepad1.left_bumper) &&
                (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) &&
                (currentGamepad1.right_stick_button && !previousGamepad1.right_stick_button)) {
                movementToggle = !movementToggle
            }

            // Movement
            if (movementToggle) {
                motorFL.power = (-gamepad1LeftY - gamepadLeftX - gamepad1RightX) / speedDiv
                motorBL.power = (-gamepad1LeftY + gamepadLeftX - gamepad1RightX) / speedDiv
                motorFR.power = (-gamepad1LeftY + gamepadLeftX + gamepad1RightX) / speedDiv
                motorBR.power = (-gamepad1LeftY - gamepadLeftX + gamepad1RightX) / speedDiv
            } else {
                motorFL.power = 0.0
                motorBL.power = 0.0
                motorFR.power = 0.0
                motorBR.power = 0.0
            }

            telemetry.addLine("Launch servo position is ${launcherServo.position}")
            telemetry.addLine("Launch servo rotate motor is ${launcherRotateMotor.power} (rotateRunCounter: $rotateRunCounter)")
            telemetry.addLine("Robot movement is ${if (movementToggle) "enabled" else "disabled"}")
            telemetry.addLine("Motor FL power: ${motorFL.power}")
            telemetry.addLine("Motor FR power: ${motorFR.power}")
            telemetry.addLine("Motor BL power: ${motorBL.power}")
            telemetry.addLine("Motor BR power: ${motorBR.power}")
            telemetry.addLine("IMU: X: ${orientation.firstAngle}, Y: ${orientation.secondAngle}, Z: ${orientation.thirdAngle}")
            telemetry.update()

            sleep(30)
        }
    }

    companion object {
        const val LauncherServoReadyPosition = 0.11 // Press B
        const val LauncherServoLaunchPosition = 0.06 // Press A
    }
}