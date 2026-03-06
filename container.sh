#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONTAINER_NAME="keti_ros2_container"
SYS_TYPE="nvidia"       # or cpu, jetpack

# Function to display help
show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  help                    Show this help message"
    echo "  create                  Create and start container (image → new container)"
    echo "  start                   Start existing container (already created)"
    echo "  enter                   Enter the running container"
    echo "  stop                    Stop the container"
    echo "  remove                  Stop and remove the container"
    echo ""
    echo "Examples:"
    echo "  $0 create               Create and start container"
    echo "  $0 start                Start existing container"
    echo "  $0 enter                Enter the running container"
    echo "  $0 stop                 Stop container"
    echo "  $0 remove               Remove container"
}

# create: 이미지로 새 컨테이너 생성 후 시작 (기존 start 역할)
create_container() {
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi
    echo -f "Creating and starting ${CONTAINER_NAME} container..."
    docker compose -f "${SCRIPT_DIR}/docker-compose.${SYS_TYPE}.yml" up -d
}

# start: 이미 존재하는 컨테이너만 시작 (새로 만들지 않음)
start_container() {
    if docker ps -a | grep -q "$CONTAINER_NAME"; then
        echo -f "Starting existing ${CONTAINER_NAME} container..."
        docker compose -f "${SCRIPT_DIR}/docker-compose.${SYS_TYPE}.yml" start
    else
        echo "Error: Container does not exist. Run '$0 create' first."
        exit 1
    fi
}

# Function to enter the container
enter_container() {
    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi
    docker exec -it "$CONTAINER_NAME" bash
}

# stop: 컨테이너 중지만 (삭제 안 함)
stop_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi
    echo -f "Stopping ${CONTAINER_NAME} container..."
    docker compose -f "${SCRIPT_DIR}/docker-compose.${SYS_TYPE}.yml" stop
}

# remove: 컨테이너 중지 후 삭제
remove_container() {
    echo "Warning: This will stop and remove the container. All unsaved data in the container will be lost."
    read -p "Are you sure you want to continue? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker compose -f "${SCRIPT_DIR}/docker-compose.${SYS_TYPE}.yml" down
        echo "Container removed."
    else
        echo "Operation cancelled."
        exit 0
    fi
}

# Main command handling
case "$1" in
    "help")
        show_help
        ;;
    "create")
        create_container
        ;;
    "start")
        start_container
        ;;
    "enter")
        enter_container
        ;;
    "stop")
        stop_container
        ;;
    "remove")
        remove_container
        ;;
    *)
        echo "Error: Unknown command"
        show_help
        exit 1
        ;;
esac