pipeline {
    agent any

    environment {
        IMAGE_NAME = "ros2_ci:latest"
        CONTAINER_NAME = "ros2_ci_build"
        DISPLAY = ":0"
        QT_X11_NO_MITSHM = "1"
    }

    stages {
        stage('Checkout') {
            steps {
                checkout scm
            }
        }

        stage('Build Docker Image') {
            steps {
                echo "Building Docker image for ROS 2 CI..."
                sh '''
                    docker build -t ${IMAGE_NAME} .
                '''
            }
        }

        stage('Run ROS 2 Simulation and Tests') {
            steps {
                echo "Running ROS 2 Docker container..."
                sh '''
                docker run --name ${CONTAINER_NAME} --rm \
                    -e DISPLAY=${DISPLAY} \
                    -e QT_X11_NO_MITSHM=${QT_X11_NO_MITSHM} \
                    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                    ${IMAGE_NAME} /bin/bash -c "
                        set -e
                        source /opt/ros/humble/setup.bash &&
                        source /root/ros2_ws/install/setup.bash &&

                        echo 'Launching Gazebo simulation...' &&
                        ros2 launch fastbot_gazebo one_fastbot_room.launch.py

                        echo 'Waiting for simulation topics...' &&
                        timeout 60 bash -c 'until ros2 topic list | grep -q /odom; do sleep 2; echo Waiting for Gazebo...; done'

                        echo 'Starting fastbot_action_server node...' &&
                        source /root/ros2_ws/install/setup.bash &&
                        ros2 run fastbot_waypoints fastbot_action_server &
                        ACTION_PID=$!

                        echo 'Waiting for action server to become available...' &&
                        timeout 30 bash -c 'until ros2 node list | grep -q fastbot_action_server; do sleep 2; echo Waiting for fastbot_action_server...; done'

                        echo 'Running tests...' &&
                        source /root/ros2_ws/install/setup.bash &&
                        colcon test --packages-select fastbot_waypoints --event-handler=console_direct+

                        echo 'Cleaning up processes...' &&
                        pkill -f fastbot_action_server || true &&
                        pkill -f gzserver || true &&
                        pkill -f gzclient || true &&
                        pkill -f robot_state_publisher || true &&
                        pkill -f ros2 || true &&
                        sleep 3
                    "
                '''
            }
        }
    }

    post {
        success {
            echo "Build and test completed successfully!"
        }
        failure {
            echo "Build or test failed. Check the Console Output above for details."
        }
        always {
            echo "Cleaning up dangling containers..."
            sh "docker rm -f ${CONTAINER_NAME} || true"
        }
    }
}
