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

    override fun init() {
        panelsTelemetry = PanelsTelemetry.telemetry
        // ...
        panelsTelemetry.debug("Status", "Initialized")
        panelsTelemetry.update(telemetry)

        follower = Constants.create(hardwareMap)
        follower.setPose(p.of(9.000, 25.000, 0.000))

        pathTimer = ElapsedTime()
        paths = Paths() // Build paths

        tracker = ProgressTracker(follower, telemetry)
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

    class Paths {
        val DriveToShoot: Path
        val CollectRow1: Path
        val ReturnToShoot: Path

        init {
            DriveToShoot = curve(
                p.of(9.000, 25.000, 0.0),
                p.of(28.000, 22.000, 0.0),
                p.of(48.000, 48.000, 0.0)
            ).linear(Math.toRadians(0.0), Math.toRadians(45.0))

            CollectRow1 = curve(
                p.of(48.000, 48.000, 0.0),
                p.of(30.000, 58.000, 0.0),
                p.of(34.000, 78.000, 0.0)
            ).tangent()

            ReturnToShoot = line(
                p.of(34.000, 78.000, 0.0),
                p.of(48.000, 48.000, 0.0)
            ).linear(Math.toRadians(90.0), Math.toRadians(45.0))
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
