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

        stage('Run Simulation and Tests') {
            steps {
                echo "Running simulation and ROS 2 tests inside container..."
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
                            ros2 launch fastbot_gazebo one_fastbot_room.launch.py &
                            GAZEBO_PID=$!

                            echo 'Waiting for /odom topic (Gazebo startup)...'
                            timeout 90 bash -c 'until ros2 topic list | grep -q /odom; do sleep 2; echo Waiting...; done'

                            echo 'Running fastbot_action_server node...'
                            ros2 run fastbot_waypoints fastbot_action_server &

                            echo 'Running ROS 2 tests for fastbot_waypoints...'
                            colcon test --packages-select fastbot_waypoints --event-handlers console_direct+

                            echo 'Cleaning up Gazebo process...'
                            kill $GAZEBO_PID || true
                            wait $GAZEBO_PID || true
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
