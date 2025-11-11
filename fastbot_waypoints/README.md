## Passing Conditions

The passing conditions for position and rotation tests require a larger tolerance to be set:

*fastbot_waypoints/src/test/test_end_position.cpp*

Line 72:
```
    EXPECT_LT(dist_error, 0.15);
```

---

*fastbot_waypoints/src/test/test_end_rotation.cpp*

Line 84:
```
    EXPECT_LT(err_yaw, 0.4);
```

---

## Failing Conditions

For the failing conditions, change the tolerances in the python testing scripts to lower values:

*fastbot_waypoints/src/test/test_end_position.cpp*

Line 72:
```
    EXPECT_LT(dist_error, 0.001);
```

---

*fastbot_waypoints/src/test/test_end_rotation.cpp*

Line 84:
```
    EXPECT_LT(err_yaw, 0.001);
```