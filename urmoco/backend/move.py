def handle_move(config, state, robot, urmoco_out_queue, dfmoco_out_queue):
    # A move can timeout, we want to check this and stop if we do so
    state["move"]["time_elapsed_seconds"] += state["cycle"]["prev_duration_seconds"]
    timeout_seconds = config.get('robot.move_timeout_seconds')
    if state["move"]["time_elapsed_seconds"] > timeout_seconds:
        robot.stop()
        state["move"]["active"] = False
        state["move"]["target_joints"] = None
        state["move"]["time_elapsed_seconds"] = 0
        urmoco_out_queue.put({"type": "move_timeout"})

        if state["shooting"]:
            state["frame"] = -1
            dfmoco_out_queue.put({
                "type": "is_moving",
                "payload": {
                    "is_moving": state["move"]["active"]
                }
            })
            dfmoco_out_queue.put({
                "type": "set_frame",
                "payload": {
                    "current_frame": state["frame"]
                }
            })
            return

    # If we didn't timeout, we check if we reached our target
    target_distance = robot.get_joints_distance(state["move"]["target_joints"])
    target_distance_threshold = config.get('robot.target_distance_threshold')
    if target_distance < target_distance_threshold:
        state["move"]["active"] = False
        state["move"]["target_joints"] = None
        state["move"]["time_elapsed_seconds"] = 0
        urmoco_out_queue.put({"type": "move_success"})

        if state["shooting"]:
            dfmoco_out_queue.put({
                "type": "is_moving",
                "payload": {
                    "is_moving": state["move"]["active"]
                }
            })
            dfmoco_out_queue.put({
                "type": "set_frame",
                "payload": {
                    "current_frame": state["move"]["target_frame"]
                }
            })
