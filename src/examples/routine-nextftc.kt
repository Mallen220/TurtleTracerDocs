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

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand

import com.turtletracerlib.pathing.ProgressTracker
import com.turtletracerlib.pathing.NamedCommands
import com.turtletracerlib.TurtleTracerReader

class AutoPath(
    private val follower: Follower,
    hw: HardwareMap,
) : Command() {
    private val p = PoseFactory.degrees()
    private val pp: TurtleTracerReader
    private val tracker: ProgressTracker
    private var group: Command? = null

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

    override fun start() {
        buildPaths()
        val sequence = SequentialGroup(
            InstantCommand {
                tracker.clearPathEvents()
                pp.registerLineEvents(tracker, 0, 0)
                tracker.setCurrentPath(startPointTODriveToShoot)
            },
            ParallelRaceGroup(
                LambdaCommand("FollowPath").setStart { follower.follow(startPointTODriveToShoot) }.setIsDone { !follower.isBusy() },
                LambdaCommand("Update").setUpdate { tracker.update() }.setIsDone { false }
            ),
            Delay(1.500),
            LambdaCommand("FollowPath").setStart { follower.follow(DriveToShootTOCollectRow1) }.setIsDone { !follower.isBusy() },
            InstantCommand {
                tracker.clearPathEvents()
                pp.registerLineEvents(tracker, 2, 0)
                tracker.setCurrentPath(CollectRow1TOReturnToShoot)
            },
            ParallelRaceGroup(
                LambdaCommand("FollowPath").setStart { follower.follow(CollectRow1TOReturnToShoot) }.setIsDone { !follower.isBusy() },
                LambdaCommand("Update").setUpdate { tracker.update() }.setIsDone { false }
            )
        )
        group = sequence
        sequence.start()
    }

    override fun update() {
        group?.update()
    }

    override fun stop(interrupted: Boolean) {
        group?.stop(interrupted)
    }

    override val isDone: Boolean get() = group?.isDone ?: false
}
