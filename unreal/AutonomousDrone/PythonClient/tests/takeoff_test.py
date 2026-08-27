import time

import cosysairsim as airsim


def main():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    client.enableApiControl(True)
    client.armDisarm(True)

    try:
        print("이륙")
        client.takeoffAsync().join()

        print("5초간 호버링")
        client.hoverAsync().join()
        time.sleep(5)

        print("착륙")
        client.landAsync().join()
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)


if __name__ == "__main__":
    main()
