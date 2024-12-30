package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.config.Config
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
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable
import org.firstinspires.ftc.teamcode.drive.advanced.TrajectorySequenceRunnerCancelable
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder
import kotlin.math.abs

@Suppress("SpellCheckingInspection")
@Config
class SampleGoBildaPinpointMecanumDriveCancelable(hardwareMap: HardwareMap) : Drive() {
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

    // Define and get Motors
    private var motorFL = hardwareMap.dcMotor["motorFL"]
    private var motorFR = hardwareMap.dcMotor["motorFR"]
    private var motorBL = hardwareMap.dcMotor["motorBL"]
    private var motorBR = hardwareMap.dcMotor["motorBR"]

    private var motors: List<DcMotor> = listOf(motorFL, motorFR, motorBL, motorBR)

    private var batteryVoltageSensor = hardwareMap.voltageSensor.first()
    private var goBildaLocalizer = GoBildaPinpointMecanumLocalizer(hardwareMap)

    private var trajectoryFollower: TrajectoryFollower
    private var trajectorySequenceRunnerCancelable: TrajectorySequenceRunnerCancelable

    override var localizer: Localizer
        get() = goBildaLocalizer
        set(_) {}
    override val rawExternalHeading: Double
        get() = localizer.poseEstimate.heading

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
        trajectorySequenceRunnerCancelable.followTrajectorySequenceAsync(trajectorySequence)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence)
        waitForIdle()
    }

    fun setPIDFCoefficients(runMode: DcMotor.RunMode, coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
            coefficients.p, coefficients.i, coefficients.d,
            coefficients.f * 12 / batteryVoltageSensor.voltage
        )

        motors.forEach {
            (it as DcMotorEx).setPIDFCoefficients(runMode, compensatedCoefficients)
        }
    }

    fun update() {
        localizer.update()
        val signal = trajectorySequenceRunnerCancelable.update(localizer.poseEstimate, localizer.poseVelocity)
        if (signal != null) setDriveSignal(signal)
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy)
            update()
    }

    fun turn(angle: Double) {
        turnAsync(angle)
        waitForIdle()
    }

    fun turnAsync(angle: Double) {
        trajectorySequenceRunnerCancelable.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(poseEstimate)
                .turn(angle)
                .build()
        )
    }

    fun setWeightedDrivePower(drivePower: Pose2d) {
        var vel = drivePower

        if ((abs(drivePower.x) + abs(drivePower.y) + abs(drivePower.heading)) > 1) {
            // re-normalize the powers according to the weights
            val denom =
                (SampleMecanumDriveCancelable.VX_WEIGHT * abs(drivePower.x) + SampleMecanumDriveCancelable.VY_WEIGHT * abs(
                    drivePower.y
                ) + SampleMecanumDriveCancelable.OMEGA_WEIGHT * abs(drivePower.heading))

            vel = Pose2d(
                SampleMecanumDriveCancelable.VX_WEIGHT * drivePower.x,
                SampleMecanumDriveCancelable.VY_WEIGHT * drivePower.y,
                SampleMecanumDriveCancelable.OMEGA_WEIGHT * drivePower.heading
            ).div(denom)
        }

        setDrivePower(vel)
    }

    fun setMode(mode: DcMotor.RunMode) {
        motors.forEach {
            it.mode = mode
        }
    }

    val isBusy: Boolean
        get() = trajectorySequenceRunnerCancelable.isBusy

    init {
        // Reverse motors here
        motorBL.direction = DcMotorSimple.Direction.REVERSE
        motorFL.direction = DcMotorSimple.Direction.REVERSE

//        setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, MOTOR_VELO_PID)

        trajectoryFollower = HolonomicPIDVAFollower(
            TRANSLATIONAL_PID,
            TRANSLATIONAL_PID,
            HEADING_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)),
            0.5
        )

        trajectorySequenceRunnerCancelable =
            TrajectorySequenceRunnerCancelable(trajectoryFollower, HEADING_PID)
    }
}
