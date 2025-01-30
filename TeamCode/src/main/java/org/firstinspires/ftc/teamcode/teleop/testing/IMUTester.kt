package org.firstinspires.ftc.teamcode.teleop.testing

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.IntegratingGyroscope
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation

@TeleOp(name = "IMUTester")
@Disabled

class IMUTester : LinearOpMode() {
    override fun runOpMode() {
        val navX = hardwareMap.get(NavxMicroNavigationSensor::class.java, "navX")

        navX.initialize()

        val gyro = navX as IntegratingGyroscope

        var orientation: Orientation

        telemetry.addLine("Gyro calibrating :(")
        telemetry.update()

        while (navX.isCalibrating) {
            telemetry.addLine("Still calibrating :(")
            telemetry.update()
            sleep(50)
        }

        telemetry.addLine("Init done :)")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            orientation = gyro.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
            )

            telemetry.addLine("IMU data")
            telemetry.addLine("X rotation: ${orientation.firstAngle}")
            telemetry.addLine("Y rotation: ${orientation.secondAngle}")
            telemetry.addLine("Z rotation: ${orientation.thirdAngle}")
            telemetry.update()
        }
    }
}