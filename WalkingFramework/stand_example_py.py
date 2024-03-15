from bindings.lib import go2_py as go2
import time
def main():
    go2.ChannelFactory.InstanceInit("eno2")
    robot = go2.Go2();
    robot.init_robot_state_client()
    while (robot.query_service_status("sport_mode")):
        robot.activate_service("sport_mode", 0)
        print("Try to deactivate the service sport mode")
        time.sleep(1)
    robot.init()
    print("finished")
    while True:
        time.sleep(10)
if __name__ == "__main__":
    main()