package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Methods

@TeleOp(name = "RESET_MOTORS", group = "AAAA")

class ResetMotors : Methods() {
    override fun runOpMode() {
        initMotors()
    }
}