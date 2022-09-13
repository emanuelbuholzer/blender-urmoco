def get_operators(config, scheduler):
    urmoco_in_queue = scheduler.ur_in_q
    urmoco_out_queue = scheduler.ur_out_q

    operators = []

    from .freedrive import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .power_off import get_operators
    operators += get_operators(config, scheduler, urmoco_in_queue, urmoco_out_queue)

    from .shooting import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .stop import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .sync import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .transfer import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .unlock import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .startup import get_operators
    operators += get_operators(config, scheduler, urmoco_in_queue, urmoco_out_queue)

    return operators
