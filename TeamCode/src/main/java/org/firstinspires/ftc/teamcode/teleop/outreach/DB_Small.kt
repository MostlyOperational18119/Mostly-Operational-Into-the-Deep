package org.firstinspires.ftc.teamcode.teleop.outreach

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name="Baby_Bot", group = "sdsds")
@Disabled

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
        val resetPos = 0.6
        val launchPos = 0.86
        val speedDiv = 2.0
        val openPos = 0.3
        val closePos = 0.6

        var open = true

        val currentGamepad = Gamepad()
        val previousGamepad = Gamepad()

        launcher.position = resetPos
        clawRotation.position = 0.3
        claw.position = openPos

        waitForStart()

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad)
            currentGamepad.copy(gamepad1)

            val y = gamepad1.left_stick_y.toDouble()
            val r = gamepad1.right_stick_x.toDouble()
            val reset = gamepad1.b
            val launch = (currentGamepad.a && previousGamepad.a)
            val armUp = gamepad1.dpad_up
            val armDown = gamepad1.dpad_down

            val toggleClaw = (currentGamepad.left_bumper && !previousGamepad.left_bumper)
            right.power = (y-r)/speedDiv
            left.power = (y+r)/speedDiv

            if (reset) {
                launcher.position = resetPos
            }
            else if (launch) {
                launcher.position = launchPos
            }

            if (armUp) {
                launcherArm.power = -0.2
            }
            else if (armDown) {
                launcherArm.power = 0.2
            }
            else {launcherArm.power = 0.0}

            if (toggleClaw) {
                claw.position = if (open) closePos else openPos
                open = !open
            }

            // telemetry
            telemetry.addData("Launcher", launcher.position)
            telemetry.addData("Claw", claw.position)
            telemetry.addData("Claw open", open)
            telemetry.addData("Right arm", launcherArm.currentPosition)
            telemetry.update()

            sleep(50)
        }
    }
}