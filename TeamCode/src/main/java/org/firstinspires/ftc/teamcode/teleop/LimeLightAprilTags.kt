package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit


@TeleOp(name = "LimeLightAprilTags")
@Disabled
class LimeLightAprilTags : LinearOpMode() {

    override fun runOpMode() {
        val limelight3A = hardwareMap.get(Limelight3A::class.java, "limelight")
        val imu = hardwareMap.get(IMU::class.java, "imu")

        val logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP
        val usbDirection = UsbFacingDirection.FORWARD

        val orientationOnRobot = RevHubOrientationOnRobot(logoDirection, usbDirection)
        imu.initialize(IMU.Parameters(orientationOnRobot))
        imu.resetYaw();

        telemetry.addLine("Initialized")
        telemetry.update()

        waitForStart()

        limelight3A.start()
        //switch to April Tag pipeline
        if(!limelight3A.pipelineSwitch(1)) {
            telemetry.addLine("Pipeline switch failed");
            telemetry.update()
            sleep(2000);
        }
        while (opModeIsActive()) {
//            telemetry.addData("Limelight status: ", limelight3A.status)
//
//            val latestResult = limelight3A.latestResult
//            if (latestResult != null) {
//                telemetry.addData("Current robot pose: ", latestResult.botpose)
//
//            }

            val orientation = imu.robotYawPitchRollAngles
            val angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES)

            limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES))
            telemetry.addData(
                "Yaw (Z)",
                "%.2f Deg. (Heading)",
                orientation.getYaw(AngleUnit.DEGREES)
            )

            val latestResult = limelight3A.latestResult;
            val pose = latestResult.botpose_MT2;
            telemetry.addLine("Bot Pose: $pose");

//            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES))
//            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES))
//            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate)
//            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate)
//            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate)
//            telemetry.update()

            telemetry.update()
            sleep(100)
        }
    }
}