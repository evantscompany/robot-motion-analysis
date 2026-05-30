import subprocess
import signal

processes = []

node_files = [

    '../phase07_rviz_path/path_visualizer.py',

    '../phase08_waypoint_marker/waypoint_marker.py',

    '../phase09_active_waypoint/active_waypoint.py',

    '../phase10_heading_marker/heading_marker.py',

    '../phase11_waypoint_route/waypoint_route.py',

    '../phase13_fake_robot_simulator/fake_robot_simulator.py'
]

for node in node_files:
    print(f'[START]{node}')

    p = subprocess.Popen(
        ['python3',node]
    )

    processes.append(p)

try:
    for p in processes:
        p.wait()

except KeyboardInterrupt:
    print('\n[INFO] Shutting down...')

    for p in processes:
        p.send_signal(signal.SIGINT)
        p.wait()
    
    print('[INFO] All nodes stopped')
