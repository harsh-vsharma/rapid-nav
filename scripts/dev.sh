#!/usr/bin/env bash
# Build and test the workspace inside the Docker image from docker/Dockerfile. Optional: on
# Ubuntu 24.04 with ROS 2 Jazzy, build natively instead (docs/setup.md).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=rapid-nav-dev:jazzy
VOLUME=rapid-nav-ws

# -t only when stdout is a terminal
TTY=""
if [ -t 1 ]; then
    TTY="-t"
fi

usage() {
    cat <<EOF
usage: scripts/dev.sh <command>

  image           build the image $IMAGE from docker/Dockerfile
  build [args]    colcon build of src/ (inside the image, into the volume $VOLUME)
  test            colcon test of src/, then the test results
  deps            rosdep check of the package dependencies
  run <command>   run one command with ROS and the workspace sourced
  smoke [args]    headless simulation + sim_check; the bundle goes to sim_check_docker_<time>/
                  (args go to sim_check; "--odom-tf wheel" also starts the simulation in wheel mode)
  planner-smoke [arena|maze] [--sim]
                  planning stack + planner_check; offline (static localisation) on the map, or with
                  --sim the simulation with AMCL (arena only); the bundle goes to
                  planner_check_docker_<map>_<time>/
  nav-smoke <tour|unmapped> <dwa|dwb|rpp>
                  navigation stack in the simulation + nav_run (goal runs scored on ground truth);
                  the bundle goes to nav_run_<scenario>_<controller>_<time>/
  shell           open an interactive bash with the same mounts
  clean           delete the volume $VOLUME
EOF
}

# Run one shell command line inside the image, with ROS and the workspace sourced.
in_container() {
    local cmd="$1"
    docker run --rm $TTY -v "$REPO":/repo -v "$VOLUME":/ws -w /ws "$IMAGE" \
        bash -c "source /opt/ros/jazzy/setup.bash && { [ ! -f /ws/install/setup.bash ] || source /ws/install/setup.bash; } && $cmd"
}

# Quote arguments so that they survive the trip through bash -c (prints a leading space per argument).
quoted() {
    local out="" a
    for a in ${1+"$@"}; do
        out="$out $(printf '%q' "$a")"
    done
    printf '%s' "$out"
}

case "${1:-}" in
    image)
        docker build -t "$IMAGE" -f "$REPO/docker/Dockerfile" "$REPO/docker"
        ;;
    build)
        shift
        # At most two compile jobs: a C++ translation unit that includes the Nav2 headers takes about 1 GB, and the
        # Docker VM has about 4.8 GiB, so more jobs risk running out of memory.
        in_container "MAKEFLAGS=-j2 colcon build --base-paths /repo/src --build-base /ws/build --install-base /ws/install --parallel-workers 2 --event-handlers console_cohesion+$(quoted ${1+"$@"})"
        ;;
    test)
        # Show the test results even when some test fails, then exit with colcon test's code.
        # colcon creates no results folder when no package has tests (src/ is empty before S1).
        in_container "colcon test --base-paths /repo/src --build-base /ws/build --install-base /ws/install --test-result-base /ws/test_results --return-code-on-test-failure --event-handlers console_cohesion+; rc=\$?; if [ -d /ws/test_results ]; then colcon test-result --test-result-base /ws/test_results --verbose; fi; exit \$rc"
        ;;
    deps)
        in_container "rosdep check --from-paths /repo/src --ignore-src --rosdistro jazzy"
        ;;
    run)
        shift
        if [ "$#" -eq 0 ]; then
            usage >&2
            exit 2
        fi
        in_container "$*"
        ;;
    smoke)
        shift
        stamp="$(date +%Y%m%d-%H%M%S)"
        bundle="/repo/sim_check_docker_$stamp"
        odom_arg=""
        case " $* " in
            *" --odom-tf wheel "*) odom_arg=" odom_tf:=wheel" ;;
        esac
        # Start the headless simulation, wait up to 90 s for /clock, run sim_check into the bundle,
        # copy the launch log into it, stop the simulation and exit with sim_check's code.
        smoke_cmd="mkdir -p $bundle; ros2 launch rapid_nav_bringup sim.launch.py headless:=true rviz:=false$odom_arg > /ws/smoke_launch.log 2>&1 & launch_pid=\$!; ready=0; for i in \$(seq 1 90); do if timeout 10 ros2 topic echo --once /clock --no-daemon > /dev/null 2>&1; then ready=1; break; fi; sleep 1; done; if [ \$ready -eq 1 ]; then ros2 run rapid_nav_tools sim_check --profile docker --out $bundle$(quoted ${1+"$@"}); rc=\$?; else echo 'no /clock within 90 s' >&2; rc=1; fi; cp /ws/smoke_launch.log $bundle/; kill -INT \$launch_pid 2>/dev/null; sleep 10; exit \$rc"
        in_container "$smoke_cmd"
        ;;
    planner-smoke)
        shift
        map=arena
        sim=0
        for arg in ${1+"$@"}; do
            case "$arg" in
                arena|maze) map="$arg" ;;
                --sim) sim=1 ;;
                *) echo "planner-smoke: unknown argument $arg" >&2; exit 2 ;;
            esac
        done
        stamp="$(date +%Y%m%d-%H%M%S)"
        if [ "$sim" -eq 1 ]; then
            if [ "$map" != arena ]; then
                echo "planner-smoke: --sim runs the arena simulation only" >&2
                exit 2
            fi
            bundle="/repo/planner_check_docker_arena_$stamp"
            launch_cmd="ros2 launch rapid_nav_bringup sim.launch.py headless:=true rviz:=false planning:=true"
            check_cmd="ros2 run rapid_nav_tools planner_check --mode sim --map arena --profile docker --out $bundle"
        else
            bundle="/repo/planner_check_docker_${map}_$stamp"
            launch_cmd="ros2 launch rapid_nav_navigation planning.launch.py localization:=static use_sim_time:=false map:=$map"
            check_cmd="ros2 run rapid_nav_tools planner_check --mode offline --map $map --profile docker --out $bundle"
        fi
        # Start the planning stack in the background, run planner_check (it waits for the planner server
        # and the map), copy the launch log into the bundle, stop the stack and exit with planner_check's code.
        smoke_cmd="mkdir -p $bundle; $launch_cmd > /ws/planner_launch.log 2>&1 & launch_pid=\$!; $check_cmd; rc=\$?; kill -INT \$launch_pid 2>/dev/null; sleep 10; cp /ws/planner_launch.log $bundle/; exit \$rc"
        in_container "$smoke_cmd"
        ;;
    nav-smoke)
        shift
        if [ "$#" -ne 2 ]; then
            echo "usage: scripts/dev.sh nav-smoke <tour|unmapped> <dwa|dwb|rpp>" >&2
            exit 2
        fi
        scenario="$1"
        controller="$2"
        case "$scenario" in
            tour|unmapped) ;;
            *) echo "nav-smoke: unknown scenario $scenario" >&2; exit 2 ;;
        esac
        case "$controller" in
            dwa|dwb|rpp) ;;
            *) echo "nav-smoke: unknown controller $controller" >&2; exit 2 ;;
        esac
        stamp="$(date +%Y%m%d-%H%M%S)"
        bundle="/repo/nav_run_${scenario}_${controller}_$stamp"
        launch_cmd="ros2 launch rapid_nav_bringup sim.launch.py headless:=true rviz:=false navigation:=true controller:=$controller"
        check_cmd="ros2 run rapid_nav_tools nav_run --scenario $scenario --controller $controller --profile docker --out $bundle"
        # Start the navigation stack in the background, run nav_run (it waits for the clock and the
        # navigation action server), copy the launch log into the bundle, stop the stack and exit
        # with nav_run's code.
        smoke_cmd="mkdir -p $bundle; $launch_cmd > /ws/nav_launch.log 2>&1 & launch_pid=\$!; $check_cmd; rc=\$?; kill -INT \$launch_pid 2>/dev/null; sleep 10; cp /ws/nav_launch.log $bundle/; exit \$rc"
        in_container "$smoke_cmd"
        ;;
    shell)
        docker run --rm -it -v "$REPO":/repo -v "$VOLUME":/ws -w /ws "$IMAGE" \
            bash -c "source /opt/ros/jazzy/setup.bash && { [ ! -f /ws/install/setup.bash ] || source /ws/install/setup.bash; }; exec bash -i"
        ;;
    clean)
        docker volume rm "$VOLUME"
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac
