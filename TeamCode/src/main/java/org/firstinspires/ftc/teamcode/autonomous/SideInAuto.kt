package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Methods

@Autonomous(name = "AUTO COLOR", group = "AAAA")
class SideInAuto : Methods() {
    override fun runOpMode() {
        while(!opModeIsActive()){
            if (controller1.a){ startingColor = "blue" }
            if (controller1.b){ startingColor = "red" }
            telemetry.addData("Starting Color :", startingColor)
            telemetry.update()
        }
    }
}