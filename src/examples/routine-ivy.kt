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

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands

import com.turtletracerlib.pathing.ProgressTracker
import com.turtletracerlib.pathing.NamedCommands
import com.turtletracerlib.TurtleTracerReader

/**
 * Call `Scheduler.reset()` in init(), start it with
 * `AutoPath(follower, hardwareMap).command().schedule()`, and call
 * `follower.update()` and `Scheduler.execute()` every loop.
 */
class AutoPath(
    private val follower: Follower,
    hw: HardwareMap,
) {
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

        tracker = ProgressTracker(follower, null)

        // Load poses
        startPoint = pp.get("startPoint")
        DriveToShoot = pp.get("DriveToShoot")
        DriveToShoot_line0_control1 = pp.get("DriveToShoot_line0_control1")
        CollectRow1 = pp.get("CollectRow1")
        CollectRow1_line1_control1 = pp.get("CollectRow1_line1_control1")
        ReturnToShoot = pp.get("ReturnToShoot")

        follower.setPose(startPoint)
    }

    fun buildPaths() {
        startPointTODriveToShoot = curve(startPoint, p.of(28.000, 22.000, 0.0), DriveToShoot)
            .linear(Math.toRadians(0.0), Math.toRadians(45.0))

        DriveToShootTOCollectRow1 = curve(DriveToShoot, p.of(30.000, 58.000, 0.0), CollectRow1)
            .tangent()

        CollectRow1TOReturnToShoot = line(CollectRow1, ReturnToShoot)
            .linear(Math.toRadians(90.0), Math.toRadians(45.0))
    }

    fun command(): Command {
        buildPaths()
        return Groups.sequential(
            Commands.instant {
                tracker.clearPathEvents()
                pp.registerLineEvents(tracker, 0, 0)
                tracker.setCurrentPath(startPointTODriveToShoot)
            },
            Groups.race(
                PedroCommands.follow(follower, startPointTODriveToShoot),
                Commands.infinite { tracker.update() }
            ),
            Commands.waitMs(1500.0),
            PedroCommands.follow(follower, DriveToShootTOCollectRow1),
            Commands.instant {
                tracker.clearPathEvents()
                pp.registerLineEvents(tracker, 2, 0)
                tracker.setCurrentPath(CollectRow1TOReturnToShoot)
            },
            Groups.race(
                PedroCommands.follow(follower, CollectRow1TOReturnToShoot),
                Commands.infinite { tracker.update() }
            )
        )
    }
}
