package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name="Baby_Bot", group = "sdsds")
class DB_Small : LinearOpMode() {
    override fun runOpMode() {
        //motors
        val right = hardwareMap.get(DcMotor::class.java, "right")
        val left = hardwareMap.get(DcMotor::class.java, "left")
        val launcherArm = hardwareMap.get(DcMotor::class.java, "launcherArm")
        //servos
        val claw = hardwareMap.get(Servo::class.java, "claw")
        val clawRotation = hardwareMap.get(Servo::class.java, "clawRotation")
        val launcher = hardwareMap.get(Servo::class.java, "launcher")

        //variables
        val resetPos = 0.0
        val launchPos = 0.0
        val speedDiv = 2.0
        val openPos = 0.0
        val closePos = 0.0

        launcher.position = resetPos
        waitForStart()

        while (opModeIsActive()) {
            val y = gamepad1.left_stick_y.toDouble()
            val r = gamepad1.right_stick_x.toDouble()
            val reset = gamepad1.y
            val launch = gamepad1.a

            val toggleClaw = gamepad1.left_bumper

            right.power = (y-r)/speedDiv
            left.power = (y+r)/speedDiv

            if (reset) {
                launcher.position = resetPos
            }
            else if (launch) {
                launcher.position = launchPos
            }



            // telemetry
            telemetry.addData("launcher", launcher.position)
            telemetry.addData("claw", claw.position)
            telemetry.update()
        }
    }
}