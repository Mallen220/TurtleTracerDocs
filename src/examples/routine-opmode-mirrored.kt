package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.pedro.Constants
import com.turtletracerlib.pathing.NamedCommands
import com.turtletracerlib.pathing.ProgressTracker

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.TelemetryManager
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.paths.Path
import com.pedropathing.api.Paths.curve
import com.pedropathing.api.Paths.line
import com.pedropathing.api.Paths.path
import com.pedropathing.math.Pose
import com.pedropathing.paths.interpolator.Interpolator
import com.turtletracerlib.auto.Alliance
import com.turtletracerlib.auto.AutoConfig
import com.turtletracerlib.auto.AutoContext
import com.turtletracerlib.auto.AutoRegistry
import com.turtletracerlib.auto.AutoSelector
import com.turtletracerlib.auto.MirrorAxes
import com.turtletracerlib.auto.MirrorPolicy

@Autonomous(name = "Turtle Tracer Autonomous", group = "Autonomous")
@Configurable // Panels
class TurtleTracerAutonomous : OpMode() {
    private lateinit var panelsTelemetry: TelemetryManager // Panels Telemetry instance
    lateinit var follower: Follower // Pathing follower instance
    private val p = PoseFactory.degrees()
    private lateinit var tracker: ProgressTracker // Progress tracker instance for event markers
    private var pathState = 0 // Current autonomous path state (state machine)
    private lateinit var pathTimer: ElapsedTime // Timer for path state machine
    private lateinit var paths: Paths // Paths defined in the Paths class
    private lateinit var selector: AutoSelector<AutoContext> // Alliance menu shown during init
    private lateinit var ctx: AutoContext // The alliance picked, to mirror the paths for

    override fun init() {
        panelsTelemetry = PanelsTelemetry.telemetry
        // ...
        panelsTelemetry.debug("Status", "Initialized")
        panelsTelemetry.update(telemetry)

        follower = Constants.create(hardwareMap)

        pathTimer = ElapsedTime()
        // Alliance menu: D-pad left/right picks the alliance before Start.
        selector = AutoSelector(
            AutoRegistry<AutoContext>().add("Turtle Tracer Autonomous") { it },
            AutoConfig.builder()
                .baseAlliance(Alliance.RED) // The side these paths are drawn for
                .mirrorPolicy(MirrorPolicy.FOLLOW_ALLIANCE) // Mirrored when the driver picks the other one
                .mirrorAxes(MirrorAxes.BOTH)
                .build(),
        )

        tracker = ProgressTracker(follower, telemetry)
    }

    override fun init_loop() {
        selector.update(gamepad1, telemetry) // Draw the alliance menu
    }

    override fun start() {
        ctx = selector.buildSelected() // The alliance picked, and whether to mirror
        follower.setPose(ctx.pose(p.of(9.000, 25.000, 0.000)))
        paths = Paths(ctx) // Build paths for that alliance
    }

    override fun loop() {
        follower.update() // Update follower
        tracker.update() // Update tracker
        pathState = autonomousPathUpdate() // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState)
        panelsTelemetry.debug("X", follower.pose().x())
        panelsTelemetry.debug("Y", follower.pose().y())
        panelsTelemetry.debug("Heading", follower.pose().heading())
        panelsTelemetry.update(telemetry)
    }

    class Paths(ctx: AutoContext) {
        val DriveToShoot: Path
        val CollectRow1: Path
        val ReturnToShoot: Path

        init {
            DriveToShoot = curve(
                ctx.pose(p.of(9.000, 25.000, 0.0)),
                ctx.pose(p.of(28.000, 22.000, 0.0)),
                ctx.pose(p.of(48.000, 48.000, 0.0))
            ).linear(ctx.heading(Math.toRadians(0.0)), ctx.heading(Math.toRadians(45.0)))

            CollectRow1 = curve(
                ctx.pose(p.of(48.000, 48.000, 0.0)),
                ctx.pose(p.of(30.000, 58.000, 0.0)),
                ctx.pose(p.of(34.000, 78.000, 0.0))
            ).tangent()

            ReturnToShoot = line(
                ctx.pose(p.of(34.000, 78.000, 0.0)),
                ctx.pose(p.of(48.000, 48.000, 0.0))
            ).linear(ctx.heading(Math.toRadians(90.0)), ctx.heading(Math.toRadians(45.0)))
        }

        companion object {
            private val p = PoseFactory.degrees()
        }
    }

    fun autonomousPathUpdate(): Int {
        when (pathState) {
            0 -> {
                follower.follow(paths.DriveToShoot)
                tracker.clearPathEvents()
                tracker.onParametric(0, 0.400, NamedCommands.getCommand("SpinUp"))
                tracker.setCurrentPath(paths.DriveToShoot)
                setPathState(1)
            }
            1 -> {
                if(!follower.isBusy()) {
                    setPathState(2)
                }
            }
            2 -> {
                setPathState(3)
            }
            3 -> {
                if(pathTimer.milliseconds() > 1500) {
                    setPathState(4)
                }
            }
            4 -> {
                follower.follow(paths.CollectRow1)
                tracker.clearPathEvents()
                tracker.setCurrentPath(paths.CollectRow1)
                setPathState(5)
            }
            5 -> {
                if(!follower.isBusy()) {
                    setPathState(6)
                }
            }
            6 -> {
                follower.follow(paths.ReturnToShoot)
                tracker.clearPathEvents()
                tracker.onParametric(0, 0.500, NamedCommands.getCommand("Intake"))
                tracker.setCurrentPath(paths.ReturnToShoot)
                setPathState(7)
            }
            7 -> {
                if(!follower.isBusy()) {
                    setPathState(8)
                }
            }
            8 -> {
                requestOpModeStop()
                pathState = -1
            }
        }
        return pathState
    }

    fun setPathState(pState: Int) {
        pathState = pState
        pathTimer.reset()
    }
}
