package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.TouchSensor

@Disabled
@TeleOp(name = "Spinny Bot")
class SpinnyBot : LinearOpMode() {
    override fun runOpMode() {
        val rotate = hardwareMap.get(DcMotor::class.java, "rotate")
        rotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val sensor = hardwareMap.get(DigitalChannel::class.java, "sensor")
        val gamepad1Current = Gamepad()
        val gamepad1Previous = Gamepad()
        val clicks = 2786
        while (sensor.state) {
            rotate.power = 0.2
        }
        rotate.power = 0.0
        rotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rotate.targetPosition = -clicks/8
        rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
        rotate.power = 0.2
        waitForStart()
        rotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        while (opModeIsActive()) {
            gamepad1Previous.copy(gamepad1Current)
            gamepad1Current.copy(gamepad1)

            if (gamepad1Current.dpad_up && !gamepad1Previous.dpad_up) {
                rotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                rotate.power = 0.2
            }
            if (gamepad1Current.dpad_down && !gamepad1Previous.dpad_down) {
                rotate.power = 0.0
                rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
            }

            if (gamepad1Current.a && !gamepad1Previous.a && rotate.currentPosition < 0) {
                rotate.targetPosition = 0
                rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
                rotate.power = 0.2
            }
            if (gamepad1Current.b && !gamepad1Previous.b && rotate.currentPosition < clicks/4) {
                rotate.targetPosition = clicks/2
                rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
                rotate.power = 0.2
            }



            if (gamepad1Current.a && !gamepad1Previous.a && rotate.currentPosition > 0) {
                rotate.targetPosition = 0
                rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
                rotate.power = -0.2
            }
            if (gamepad1Current.b && !gamepad1Previous.b && rotate.currentPosition > clicks/4) {
                rotate.targetPosition = clicks/2
                rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
                rotate.power = -0.2
            }





            telemetry.addData("rotate power", rotate.power)
            telemetry.addData("position", rotate.currentPosition)
            telemetry.addData("sensor val", !sensor.state)
            telemetry.update()

//            sleep(50)
        }
    }
}