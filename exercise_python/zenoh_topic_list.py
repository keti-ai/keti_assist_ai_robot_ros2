
import argparse
import json
import zenoh


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--host", default="robot-arm.local"
    )
    parser.add_argument(
        "--port", type=int, default=7447
    )
    args = parser.parse_args()

    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5(
        "connect/endpoints",
        json.dumps([f"tcp/{args.host}:{args.port}"])
    )

    with zenoh.open(config) as session:
        print(f"Connected: {args.host}:{args.port}")

        # ROS2 bridge administrative routes
        selector = "@/*/ros2/route/**"

        topics = set()
        services = set()
        actions = set()
        raw_routes = []

        for reply in session.get(selector, timeout=5.0):
            if reply.ok is None:
                continue

            sample = reply.ok
            key = str(sample.key_expr)
            raw_routes.append(key)

            # Example:
            # @/<bridge-id>/ros2/route/topic/pub/joint_states
            # @/<bridge-id>/ros2/route/service/srv/...
            parts = key.split("/")

            try:
                idx = parts.index("route")
                category = parts[idx + 1]
                direction = parts[idx + 2]
                name = "/".join(parts[idx + 3:])
            except (ValueError, IndexError):
                continue

            if category == "topic":
                topics.add("/" + name.lstrip("/"))

            elif category == "service":
                services.add("/" + name.lstrip("/"))

            elif category == "action":
                actions.add("/" + name.lstrip("/"))

        print("\n=== ROS2 TOPICS ===")
        for topic in sorted(topics):
            print(topic)

        print("\n=== ROS2 SERVICES ===")
        for service in sorted(services):
            print(service)

        print("\n=== ROS2 ACTIONS ===")
        for action in sorted(actions):
            print(action)

        print("\n=== RAW ROUTES ===")
        for route in sorted(raw_routes):
            print(route)

        print(
            f"\nRoutes: {len(raw_routes)} | "
            f"Topics: {len(topics)} | "
            f"Services: {len(services)} | "
            f"Actions: {len(actions)}"
        )


if __name__ == "__main__":
    main()
