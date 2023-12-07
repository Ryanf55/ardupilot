import launch_pytest
import pytest
from ardupilot_dds_tests.plane_waypoint_follower import main as main_plane_wp

from launch import LaunchDescription

from launch_pytest.tools import process as process_tools


@launch_pytest.fixture
def launch_sitl_copter_dds_udp(sitl_copter_dds_udp):
    """Fixture to create the launch description."""
    sitl_ld, sitl_actions = sitl_copter_dds_udp

    ld = LaunchDescription(
        [
            sitl_ld,
            launch_pytest.actions.ReadyToTest(),
        ]
    )
    actions = sitl_actions
    yield ld, actions


@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp)
def test_plane_wp_follower(launch_context, launch_sitl_copter_dds_udp):
    """Test a guided waypoint mission."""
    _, actions = launch_sitl_copter_dds_udp
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=2)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=2)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=2)

    with pytest.raises(SystemExit) as e:
        main_plane_wp()
    assert e.type == SystemExit
    assert e.value.code == 0
