#!/usr/bin/env python3
import roslibpy
import time

def call_service(client, service_name):
    """Call a std_srvs/Trigger service and print the response."""
    service = roslibpy.Service(client, service_name, 'std_srvs/Trigger')
    request = roslibpy.ServiceRequest({})  # Trigger service takes no arguments

    print(f"[ROS] Calling service: {service_name}")
    result = service.call(request)
    print(f"[ROS] Response: success={result['success']}, message='{result['message']}'")
    return result['success']

GRIP_TIME = 2.0  # s to wait after a gripper command for the fingers to finish moving

def open_gripper(client):
    """Open the OnRobot gripper and wait for it. Raises if the service reports failure."""
    if not call_service(client, '/onrobot/open'):
        raise RuntimeError("Gripper failed to open")
    time.sleep(GRIP_TIME)

def close_gripper(client):
    """Close the OnRobot gripper and wait for it. Raises if the service reports failure."""
    if not call_service(client, '/onrobot/close'):
        raise RuntimeError("Gripper failed to close")
    time.sleep(GRIP_TIME)

def main():
    ros_host = '192.168.27.1'  # Replace with your ROS bridge IP
    ros_port = 9090            # rosbridge_websocket port

    client = roslibpy.Ros(host=ros_host, port=ros_port)
    try:
        client.run()
        if not client.is_connected:
            raise RuntimeError("Failed to connect to rosbridge.")

        print("[ROS] Connected to rosbridge.")

        open_gripper(client)
        close_gripper(client)

    finally:
        client.terminate()
        print("[ROS] Disconnected.")

if __name__ == '__main__':
    main()
