package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Methods

@Autonomous(name = "AUTO COLOR", group = "AAAA")
class SideInAuto : Methods() {
    override fun runOpMode() {
        waitForStart()
        while(opModeIsActive()){
            if (gamepad1.a){ startingColor = "blue" }
            if (gamepad1.b){ startingColor = "red" }
            telemetry.addData("Starting Color :", startingColor)
            telemetry.update()
        }
    }
}