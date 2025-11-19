# Ryder Robots State Manager

Controls state for ryder robots subscribers, and consumers

When activate nodes without lifecycle manager the following command must be ran where node_name is the specific node
that needs to be ran.

```bash
ros2 lifecycle set /rr_state_manager configure
ros2 lifecycle set /rr_state_manager deactivate
ros2 lifecycle set /rr_state_manager cleanup
ros2 lifecycle set /rr_state_manager shutdown
```
