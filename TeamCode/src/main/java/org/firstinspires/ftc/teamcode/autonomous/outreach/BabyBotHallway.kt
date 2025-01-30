package org.firstinspires.ftc.teamcode.autonomous.outreach

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "BabyBotHallwayAuto")
@Disabled
class BabyBotHallway : LinearOpMode() {
    override fun runOpMode() {
        val right = hardwareMap.dcMotor["right"]
        val left = hardwareMap.dcMotor["left"]
        val launcherArm = hardwareMap.dcMotor["launcherArm"]

        val launcher = hardwareMap.servo["launcher"]
        val clawRotation = hardwareMap.servo["clawRotation"]

        clawRotation.position = 0.3
        launcher.position = 0.6

        waitForStart()

        launcherArm.power = -0.3
        sleep(300)

        launcherArm.power = 0.0
        sleep(2000)

        launcher.position = 0.86

        sleep(400)

        right.power = -0.5
        left.power = -0.5
        sleep(1000)

        right.power = 0.5
        left.power = -0.5
        sleep(850)

        right.power = 0.5
        left.power = 0.5
        sleep(1000)

        right.power = 0.5
        left.power = -0.5
        sleep(850)

        right.power = -0.5
        left.power = -0.5
        sleep(1000)

        right.power = 0.0
        left.power = 0.0
        sleep(1000)

    }
}