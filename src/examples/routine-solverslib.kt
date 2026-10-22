package org.firstinspires.ftc.teamcode.Commands.AutoCommands

import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.paths.Path
import com.pedropathing.api.Paths.curve
import com.pedropathing.api.Paths.line
import com.pedropathing.api.Paths.path
import com.pedropathing.math.Pose
import com.pedropathing.paths.interpolator.Interpolator
import com.qualcomm.robotcore.hardware.HardwareMap

import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.ParallelRaceGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand

import org.firstinspires.ftc.robotcore.external.Telemetry
import com.turtletracerlib.TurtleTracerReader
import com.turtletracerlib.pathing.ProgressTracker
import com.turtletracerlib.pathing.NamedCommands

class AutoPath(
    private val follower: Follower,
    hw: HardwareMap,
    telemetry: Telemetry,
) : SequentialCommandGroup() {
    private val p = PoseFactory.degrees()
    private val pp: TurtleTracerReader
    private val tracker: ProgressTracker

    // Poses
    private val startPoint: Pose
    private val DriveToShoot: Pose
    private val DriveToShoot_line0_control1: Pose
    private val CollectRow1: Pose
    private val CollectRow1_line1_control1: Pose
    private val ReturnToShoot: Pose

    // Path chains
    private lateinit var startPointTODriveToShoot: Path
    private lateinit var DriveToShootTOCollectRow1: Path
    private lateinit var CollectRow1TOReturnToShoot: Path

    init {
        pp = TurtleTracerReader("AutoPath.turt", hw.appContext)
        pp.onEvent("SpinUp", NamedCommands.getCommand("SpinUp"))
            .onEvent("Intake", NamedCommands.getCommand("Intake"))

        tracker = ProgressTracker(follower, telemetry)

        // Load poses
        startPoint = pp.get("startPoint")
        DriveToShoot = pp.get("DriveToShoot")
        DriveToShoot_line0_control1 = pp.get("DriveToShoot_line0_control1")
        CollectRow1 = pp.get("CollectRow1")
        CollectRow1_line1_control1 = pp.get("CollectRow1_line1_control1")
        ReturnToShoot = pp.get("ReturnToShoot")

        follower.setPose(startPoint)

        buildPaths()

        addCommands(
            InstantCommand({
                tracker.clearPathEvents()
                pp.registerLineEvents(tracker, 0, 0)
                tracker.setCurrentPath(startPointTODriveToShoot)
            }),
            ParallelRaceGroup(
                FollowPathCommand(follower, startPointTODriveToShoot),
                RunCommand({ tracker.update() })
            ),
            WaitCommand(1500),
            FollowPathCommand(follower, DriveToShootTOCollectRow1),
            InstantCommand({
                tracker.clearPathEvents()
                pp.registerLineEvents(tracker, 2, 0)
                tracker.setCurrentPath(CollectRow1TOReturnToShoot)
            }),
            ParallelRaceGroup(
                FollowPathCommand(follower, CollectRow1TOReturnToShoot),
                RunCommand({ tracker.update() })
            )
        )
    }

    fun buildPaths() {
        startPointTODriveToShoot = curve(startPoint, p.of(28.000, 22.000, 0.0), DriveToShoot)
            .linear(Math.toRadians(0.0), Math.toRadians(45.0))

        DriveToShootTOCollectRow1 = curve(DriveToShoot, p.of(30.000, 58.000, 0.0), CollectRow1)
            .tangent()

        CollectRow1TOReturnToShoot = line(CollectRow1, ReturnToShoot)
            .linear(Math.toRadians(90.0), Math.toRadians(45.0))
    }
}
