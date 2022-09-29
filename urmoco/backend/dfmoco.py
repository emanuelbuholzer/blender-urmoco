def handle_dfmoco_request(dfmoco_req, state, dfmoco_out_queue, urmoco_out_queue):
    if dfmoco_req["type"] == "move_to_frame":
        urmoco_out_queue.put(dfmoco_req)
        dfmoco_out_queue.put(dfmoco_req)

    if dfmoco_req["type"] == "is_moving":
        dfmoco_out_queue.put({
            "type": "is_moving",
            "payload": {
                "is_moving": state["move"]["active"]
            }
        })
