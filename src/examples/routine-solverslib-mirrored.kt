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
import com.turtletracerlib.auto.Alliance
import com.turtletracerlib.auto.AutoContext
import com.turtletracerlib.auto.AutoSelection
import com.turtletracerlib.auto.MirrorPolicy
import com.turtletracerlib.auto.TurtleAuto

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

@TurtleAuto(name = "AutoPath", drawnFor = Alliance.RED, mirror = MirrorPolicy.FOLLOW_ALLIANCE, overrideMirror = true)
class AutoPath(
    private val follower: Follower,
    hw: HardwareMap,
    telemetry: Telemetry,
    autoContext: AutoContext,
) : SequentialCommandGroup() {
    // The driver's alliance, for paths drawn for RED: mirrored when the
    // driver picks the other one.
    private val ctx = autoContext.drawnFor(Alliance.RED).mirroring(MirrorPolicy.FOLLOW_ALLIANCE)
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
        AutoSelection.record(ctx) // The alliance it runs for, and whether it's mirrored, for TeleOp

        pp = TurtleTracerReader("AutoPath.turt", hw.appContext).mirrored(ctx.mirror())
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
        startPointTODriveToShoot = curve(startPoint, DriveToShoot_line0_control1, DriveToShoot)
            .linear(ctx.heading(Math.toRadians(0.0)), ctx.heading(Math.toRadians(45.0)))

        DriveToShootTOCollectRow1 = curve(DriveToShoot, CollectRow1_line1_control1, CollectRow1)
            .tangent()

        CollectRow1TOReturnToShoot = line(CollectRow1, ReturnToShoot)
            .linear(ctx.heading(Math.toRadians(90.0)), ctx.heading(Math.toRadians(45.0)))
    }
}
