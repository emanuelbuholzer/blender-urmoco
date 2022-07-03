def handle_move(config, state, robot, ur_out_q, df_out_q):
    target_distance = robot.get_joints_distance(state["move"]["target_joints"])
    target_distance_threshold = config.get('robot.target_distance_threshold')
    if target_distance < target_distance_threshold:
        state["move"]["active"] = False
        state["move"]["target_joints"] = None
        state["move"]["time_elapsed_seconds"] = 0

        response = {
            'type': 'move_success',
            'payload': {
                'frame': state["move"]["target_frame"]
            }
        }
        ur_out_q.put(response)
        df_out_q.put(response)
    else:
        state["move"]["time_elapsed_seconds"] += state["cycle"]["prev_duration_seconds"]

        timeout_seconds = config.get('robot.move_timeout_seconds')
        if state["move"]["time_elapsed_seconds"] > timeout_seconds:
            robot.stop()
            state["move"]["active"] = False
            state["move"]["target_joints"] = None
            state["move"]["time_elapsed_seconds"] = 0
            ur_out_q.put({"type": "move_timeout"})
            df_out_q.put({"type": "move_timeout"})
