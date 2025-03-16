package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.TouchSensor

@TeleOp(name = "Spinny Bot")
class SpinnyBot : LinearOpMode() {
    override fun runOpMode() {
        val rotate = hardwareMap.get(DcMotor::class.java, "rotate")
        rotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val sensor = hardwareMap.get(TouchSensor::class.java, "sensor")
        val gamepad1Current = Gamepad()
        val gamepad1Previous = Gamepad()
        val clicks = 3896
        var encoderClicks = 0
        while (!sensor.isPressed) {
            rotate.power = 0.3
        }
        rotate.power = 0.0
        rotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rotate.targetPosition = 0
        rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
        rotate.power = 0.3
        waitForStart()
        rotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        while (opModeIsActive()) {
            gamepad1Previous.copy(gamepad1Current)
            gamepad1Current.copy(gamepad1)

            if (gamepad1Current.dpad_up && !gamepad1Previous.dpad_up) {
                rotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                rotate.power = 0.3
            }
            if (gamepad1Current.dpad_down && !gamepad1Previous.dpad_down) {
                rotate.power = -0.3
                rotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }

            if (gamepad1Current.a && !gamepad1Previous.a && rotate.currentPosition < 0) {
                rotate.targetPosition = -1*(clicks/64)
                rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
                rotate.power = 0.3
            }
            if (gamepad1Current.b && !gamepad1Previous.b && rotate.currentPosition < clicks/4) {
                rotate.targetPosition = 33*(clicks/64)
                rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
                rotate.power = 0.3
            }

            if (gamepad1Current.a && !gamepad1Previous.a && rotate.currentPosition > 0) {
                rotate.targetPosition = -1*(clicks/64)
                rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
                rotate.power = -0.3
            }
            if (gamepad1Current.b && !gamepad1Previous.b && rotate.currentPosition > clicks/4) {
                rotate.targetPosition = 33*(clicks/64)
                rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
                rotate.power = -0.3
            }





            telemetry.addData("rotate power", rotate.power)
            telemetry.addData("position", rotate.currentPosition)
            telemetry.addData("sensor val", sensor.isPressed)
            telemetry.addData("target", rotate.targetPosition)
            telemetry.update()

//            sleep(50)
        }
    }
}