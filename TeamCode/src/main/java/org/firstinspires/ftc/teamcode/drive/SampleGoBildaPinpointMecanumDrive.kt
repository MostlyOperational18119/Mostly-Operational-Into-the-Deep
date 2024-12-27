package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner
import kotlin.math.PI

class SampleGoBildaPinpointMecanumDrive(hardwareMap: HardwareMap) : Drive() {
    fun getAccelerationConstraint(maxAccel: Double): TrajectoryAccelerationConstraint {
        return ProfileAccelerationConstraint(maxAccel)
    }

    companion object {
        val TRANSLATIONAL_PID = PIDCoefficients(8.0, 0.0, 1.0)
        val HEADING_PID = PIDCoefficients(7.0, 0.0, 0.0)

        private val VEL_CONSTRAINT: TrajectoryVelocityConstraint =
            MinVelocityConstraint(
                listOf(
                    AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                    MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH)
                )
            )

        private val ACCEL_CONSTRAINT: TrajectoryAccelerationConstraint =
            ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
    }

    private var motorFL: DcMotor
    private var motorFR: DcMotor
    private var motorBL: DcMotor
    private var motorBR: DcMotor
    private var goBildaLocalizer = GoBildaPinpointMecanumLocalizer(hardwareMap)
    private var batteryVoltageSensor: VoltageSensor

    private var trajectoryFollower: TrajectoryFollower
    private var trajectorySequenceRunner: TrajectorySequenceRunner

    override var localizer: Localizer
        get() = goBildaLocalizer
        set(_) {}
    override val rawExternalHeading: Double
        get() = localizer.poseEstimate.heading

    fun degToRadian(deg: Double): Double {
        return deg / 180.0 * PI
    }

    override fun setDrivePower(drivePower: Pose2d) {
        val powers = MecanumKinematics.robotToWheelVelocities(
            drivePower,
            DriveConstants.TRACK_WIDTH,
            1.0,
            1.0 // TODO: FIND CORRECT VALUES (LATER)
        )
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    override fun setDriveSignal(driveSignal: DriveSignal) {
        val vel = MecanumKinematics.robotToWheelVelocities(
            driveSignal.vel,
            DriveConstants.TRACK_WIDTH,
            1.0,
            1.0 // TODO: AGAIN, FIND THE CORRECT VALUES
        )

        val accel = MecanumKinematics.robotToWheelAccelerations(
            driveSignal.accel,
            DriveConstants.TRACK_WIDTH,
            1.0,
            1.0 // TODO: ALSO FIND THE CORRECT VALUES
        )

        val powers = Kinematics.calculateMotorFeedforward(vel, accel, DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    fun setMotorPowers(v: Double, v1: Double, v2: Double, v3: Double) {
        motorFL.power = v
        motorBL.power = v1
        motorBR.power = v2
        motorFR.power = v3
    }

    fun trajectorySequenceBuilder(startPose: Pose2d): TrajectorySequenceBuilder {
        return TrajectorySequenceBuilder(
            startPose,
            VEL_CONSTRAINT, ACCEL_CONSTRAINT,
            DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        )
    }

    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence)
        waitForIdle()
    }

    fun update() {
        localizer.update()
        val signal = trajectorySequenceRunner.update(localizer.poseEstimate, localizer.poseVelocity)
        if (signal != null) setDriveSignal(signal)
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy())
            update()
    }

    fun isBusy(): Boolean {
        return trajectorySequenceRunner.isBusy
    }

    init {
        motorFL = hardwareMap.dcMotor["motorFL"]
        motorFR = hardwareMap.dcMotor["motorFR"]
        motorBL = hardwareMap.dcMotor["motorBL"]
        motorBR = hardwareMap.dcMotor["motorBR"]

        batteryVoltageSensor = hardwareMap.voltageSensor.first()

        // Reverse motors

        trajectoryFollower = HolonomicPIDVAFollower(
            TRANSLATIONAL_PID,
            TRANSLATIONAL_PID,
            HEADING_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)),
            0.5
        )

        trajectorySequenceRunner = TrajectorySequenceRunner(
            trajectoryFollower, HEADING_PID,
            batteryVoltageSensor,
            ArrayList<Int>(), ArrayList<Int>(), ArrayList<Int>(), ArrayList<Int>() // TODO: Make arrays for these
        )
    }
}
