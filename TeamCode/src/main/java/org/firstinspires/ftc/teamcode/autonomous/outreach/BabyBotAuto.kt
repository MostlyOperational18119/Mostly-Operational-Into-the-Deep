package org.firstinspires.ftc.teamcode.autonomous.outreach

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "BabyBotAuto")
class BabyBotAuto : LinearOpMode() {
    override fun runOpMode() {
        val right = hardwareMap.dcMotor["right"]
        val left = hardwareMap.dcMotor["left"]
        val launcherArm = hardwareMap.dcMotor["launcherArm"]

        val launcher = hardwareMap.servo["launcher"]

        right.power = 0.5
        left.power = 0.5

        sleep(1000)

        right.power = 0.5
        left.power = -0.5

        sleep(1000)

        right.power = -0.5
        left.power = -0.5

        sleep(1000)

        right.power = 0.0
        left.power = 0.0

        sleep(1000)

        launcherArm.power = -0.1

        sleep(500)

        launcherArm.power = 0.0

        sleep(2000)

        launcher.position = 0.86

    }
}