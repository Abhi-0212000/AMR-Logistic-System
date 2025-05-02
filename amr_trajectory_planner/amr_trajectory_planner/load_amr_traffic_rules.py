# #!/usr/bin/env python3

import ctypes
import lanelet2
import os
import sys


def get_amr_traffic_rules():
    """
    Load AMR traffic rules library and return traffic rules object
    """
    # Find the library
    lib_name = "libamr_traffic_rules.so"

    # Check common locations
    possible_paths = [
        os.path.expanduser(f"~/ros2_ws/install/amr_navigation_system/lib/{lib_name}"),
        os.path.expanduser(
            f"~/ros2_ws/install/amr_navigation_system/lib/amr_navigation_system/{lib_name}"
        ),
        f"/opt/ros/humble/lib/{lib_name}",
        f"/opt/ros/humble/lib/amr_navigation_system/{lib_name}",
        f"/ros2_ws/install/amr_navigation_system/lib/{lib_name}",
        f"/ros2_ws/install/amr_navigation_system/lib/amr_navigation_system/{lib_name}",
    ]

    library_loaded = False

    # Print Python search paths for debugging
    print("Python search paths:")
    for path in sys.path:
        print(f"  {path}")

    # Try to load the library
    for path in possible_paths:
        if os.path.exists(path):
            try:
                print(f"Attempting to load: {path}")
                ctypes.CDLL(path)
                print(f"Successfully loaded {path}")
                library_loaded = True
                break
            except Exception as e:
                print(f"Error loading {path}: {e}")

    if not library_loaded:
        print("Could not find or load AMR traffic rules library")

        # Print available traffic rules for debugging
        try:
            print("Available traffic rules:")
            for rule in lanelet2.traffic_rules.available():
                print(f"  {rule[0]}, {rule[1]}")
        except Exception as e:
            print(f"Could not get available traffic rules: {e}")

        raise RuntimeError("AMR traffic rules library not loaded")

    # Now create the traffic rules
    try:
        rules = lanelet2.traffic_rules.create("germany", "amr")
        print("Successfully created AMR traffic rules!")
        return rules
    except Exception as e:
        print(f"Error creating traffic rules: {e}")
        raise


if __name__ == "__main__":
    # Test the function
    try:
        rules = get_amr_traffic_rules()
        print(f"Participant: {rules.participant()}")
        print(f"Location: {rules.location()}")
    except Exception as e:
        print(f"Failed: {e}")
