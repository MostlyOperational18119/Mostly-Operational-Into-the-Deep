package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Methods

@TeleOp(name = "RESETMOTORS", group = "Aardvark")
class ResetMotors : Methods() {
    override fun runOpMode() {
        initMotors()
    }
}