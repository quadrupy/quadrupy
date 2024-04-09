from bindings.lib import go2_py as go2
import time
def main():
    go2.ChannelFactory.InstanceInit("enxd8eb97b7f0da")
    custom = go2.Custom();
    custom.init_robot_state_client()
    while (custom.query_service_status("sport_mode")):
        custom.activate_service("sport_mode", 0)
        print("Try to deactivate the service sport mode")
        time.sleep(1)
    custom.init()
    print("finished")
    while True:
        time.sleep(10)
if __name__ == "__main__":
    main()