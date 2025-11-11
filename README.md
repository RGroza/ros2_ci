# Starting Jenkins ROS2 CI

Jenkins should already by running from the previous startup for ROS1 CI, so you can disregard this.

https://i-00cf9b4f4db24e827.robotigniteacademy.com/976738aa-dd96-4aa1-94f0-5eb849e10624/jenkins/

---

#### Run the `start_jenkins.sh` script:

```bash
cd ~/ros2_ws/src/ros2_ci
```
```bash
bash start_jenkins.sh
```

#### Login to Jenkins:
Username: `admin`  
Password: `1234`

---

#### Please note:

Please let me know if the Jenkins URL is different than this:

https://i-00cf9b4f4db24e827.robotigniteacademy.com/976738aa-dd96-4aa1-94f0-5eb849e10624/jenkins/

The GitHub Webhook for automatic build triggering was not working with the current version of Jenkins, so I had to use a GitHub Action workflow. As a result, the Jenkins URL needs to be persistent to work. Otherwise, the GitHub Action config will need to be updated with the new URL.